import argparse
import os
import logging
import json
import time
import cv2
import numpy as np
from threading import Thread

from FlightUtils.utils import *
from FlightUtils.Capture.FlightCapture import FlightCapture
from FlightUtils.Mavlink.MavlinkHandler import MavlinkHandler
from FlightUtils.Server.FlightServer import run as run_flight_server
from OpticalFlow import OpticalFlow
from helpers import *

from ThermalTools import compute_thermal_bracket, raw_to_thermal_frame

CAMERA_ROLL = None
CAMERA_PITCH = None
CAMERA_YAW = None
R_c_a = None

def loadCameraCalibration(camera):
    global R_c_a
    global CAMERA_ROLL
    global CAMERA_PITCH
    global CAMERA_YAW

    try:
        config = camera.getDeviceConfig()

        # TODO: Fix this shit
        # Set the global variables
        CAMERA_ROLL = float(config['camera_roll'])
        CAMERA_PITCH = float(config['camera_pitch'])
        CAMERA_YAW = float(config['camera_yaw'])

        # Camera intrinsics
        cam_mtx = config['camera_matrix']
        cam_mtx = np.array(cam_mtx)

        # Distortion coefficients
        dst = np.array(config['distortion_coefficients'])

        # Aircraft to camera rotation matrix
        R_c_a = buildRotationMatrix(CAMERA_ROLL, CAMERA_PITCH, CAMERA_YAW, degrees=True)

        log(f"Camera Calibration loaded")
        log(f"Roll: {CAMERA_ROLL}")
        log(f"Pitch: {CAMERA_PITCH}")
        log(f"Yaw: {CAMERA_YAW}")
        log(f"Intrinsics: {cam_mtx}")
        log(f"Distortion: {dst}")
        log(f"Rotation matrix: {R_c_a}")

        return cam_mtx, dst

    except Exception as e:
        print(f"Exception thrown loading camera calibration: {e}")
        log(f"Exception thrown loading camera calibration: {e}")
        return None, None


parser = argparse.ArgumentParser(description="Basic visual odometry script")

parser.add_argument('-c', '--calibration-directory', type=str, required=True,
                    help="Path to calibration directory containing both the camera calibration and the visual nav configuration")
parser.add_argument('-s', '--serial-port', type=str, required=True, help="Device path of the MAVLink serial port")
parser.add_argument('-b', '--baudrate', type=int, required=False, default=115200, help="Serial baudrate")
parser.add_argument('-o', '--output', type=str, required=True, help="Output directory for results")
parser.add_argument('-i', '--camera-id', type=str, required=True, help="Camera ID to use")
parser.add_argument('-m', '--secondary-camera-id', type=str, required=False, help="Secondary camera ID - this camera will only record")
parser.add_argument('-t', '--max-n-tracks', type=int, required=False, default=20,
                    help="Maximum number of ground tracking points")


parser.add_argument('-g', '--graph', action='store_true', help="Enable graphs")
parser.add_argument('-d', '--display', action='store_true', help="Enable video display")
parser.add_argument('-e', '--server', action='store_true', help="Enables server if set")
parser.add_argument('--sitl', action='store_true', help="Use SITL instead of Serial interface")
parser.add_argument('-r', '--record-video', action='store_true', help="Record video")
parser.add_argument('--rerun', action='store_true', help="Enable rerun")
parser.add_argument('--stream', action='store_true', help="Enable RTSP video stream")
parser.add_argument('--stream-port', type=int, default=8888, help="Port number to stream video with")
parser.add_argument('--stream-quality', type=int, default=10, help="Compression quality of video stream")
parser.add_argument('--stream-rate', type=float, default=10, help="Video stream rate (Hz)")


VERSION = "1.0.0"

args = parser.parse_args()


default_config = {
    "image_width_pixels": 1920,
    "image_height_pixels": 1080,
    "camera_roll": 0,
    "camera_pitch": 0,
    "camera_yaw": 90
}

CAMERA_ID = args.camera_id
SECONDARY_CAMERA_ID = args.secondary_camera_id
MAV_SERIAL_PORT = args.serial_port
MAV_SERIAL_BAUDRATE = args.baudrate
GRAPH = args.graph
DISPLAY = args.display
RECORD = args.record_video
RERUN_ENABLED = args.rerun
VIDEO_STREAM = args.stream
VIDEO_STREAM_PORT = args.stream_port
VIDEO_STREAM_QUALITY = args.stream_quality
VIDEO_STREAM_RATE = args.stream_rate

if RERUN_ENABLED:
    import rerun as rr
    rr.init("Visual Odom")
    rr.connect_tcp()

# Load camera calibration
CAMERA_CALIBRATION = os.path.join(args.calibration_directory, "camera_calibration.json")
if not os.path.exists(CAMERA_CALIBRATION):
    logging.error(f"NAVIGATION: Camera calibration path does not exist: {CAMERA_CALIBRATION}")

INTRINSIC_MATRIX = None
DISTORTION_COEFFICIENTS = None

VISUAL_NAV_CONFIG = os.path.join(args.calibration_directory, "config.json")
if not os.path.exists(args.calibration_directory):
    os.makedirs(args.calibration_directory)

if not os.path.exists(VISUAL_NAV_CONFIG):
    logging.warning(f"NAVIGATION: Could not find stellar config file at {VISUAL_NAV_CONFIG}, using default settings")
    with open(VISUAL_NAV_CONFIG, 'w') as f:
        json.dump(default_config, f, indent=4)

update = False
with open(VISUAL_NAV_CONFIG, 'r') as f:
    dat = json.load(f)
    for key, value in default_config.items():
        if not key in dat:
            logging.warning(f"NAVIGATION: Config key {key} not found. Using default value: {value}")
            dat.update({key: value})
            update = True

    IMAGE_WIDTH = int(dat['image_width_pixels'])
    IMAGE_HEIGHT = int(dat['image_height_pixels'])



if update:
    with open(VISUAL_NAV_CONFIG, 'w') as f:
        json.dump(dat, f, indent=4)



COMMAND_SELECT_CAMERA = 1
COMMAND_RESET = 2
COMMAND_SET_VALUE = 3


class KalmanFilter:
    def __init__(self, x0, y0, vx0, vy0, q_vx, q_vy):
        self.x = [x0, y0, vx0, vy0]
        self.xhat = self.x
        self.Q = np.diag([0, 0, q_vx, q_vy])

        self.H = np.array([[0, 0, 1, 0],[0, 0, 0, 1]])
        self.P = self.Q
        self.y = [0, 0]
        self.S = np.diag([0, 0, 0, 0])
        self.last_update = time.time()
    
    def predict(self):

        dt = time.time() - self.last_update
        self.last_update = time.time()

        x_0 = self.x[0] + dt * self.x[2]
        x_1 = self.x[1] + dt * self.x[3]


        Q = np.zeros((4, 4))
        Q_upper = self.P[2:4, 2:4] * dt
        Q[0:2, 0:2] = Q_upper
        Q[2:4, 2:4] = self.Q[2:4, 2:4]
        self.Q = Q


        return [x_0, x_1, self.x[2], self.x[3]], self.P + self.Q
    
    def update(self, z, R):
        self.xhat, self.Phat = self.predict()

        self.y = z - self.H @ self.xhat
        self.S = self.H @ self.Phat @ self.H.T + R
        self.K = self.Phat @ self.H.T @ np.linalg.inv(self.S)
        self.x = self.xhat + self.K @ self.y

        self.P = (np.eye(4) - self.K @ self.H) @ self.Phat

        """
        print(f"Q: {self.Q}")
        print(f"Phat: {self.Phat}")
        print(f"y: {self.y}")
        print(f"S: {self.S}")
        print(f"K: {self.K}")
        print(f"x: {self.x}")
        print(f"P: {self.P}")
        """

        return self.x


    def get_state(self):
        return self.x



class NavigationController:
    csv_header = "frame%video_frame_index%state%timestamp%camera_roll%camera_pitch%camera_yaw\n"

    def __init__(self):
        global INTRINSIC_MATRIX
        global DISTORTION_COEFFICIENTS

        camera_list = []
        if SECONDARY_CAMERA_ID is not None:
            camera_list = [CAMERA_ID, SECONDARY_CAMERA_ID]
        else:
            camera_list = [CAMERA_ID]

        log(f"Camera list: {camera_list}")
        print(f"Camera list: {camera_list}")
        self.fc = FlightCapture(record=RECORD, log_flight_data=True, rgb_camera_list=camera_list)
        self.mv = MavlinkHandler(self.fc, port=args.serial_port, baud=args.baudrate, log_flight_data=True,
                                 log_fast_attitude=True)

        self.mv.set_mav_cmd_user_1_callback(self.mav_cmd_user_1_callback)


        if args.server:
            server_thread = Thread(target=run_flight_server, args=[self.fc])
            server_thread.daemon = True
            server_thread.start()
        
        
        if VIDEO_STREAM:
            from VideoStream import VideoStreamServer
            self.video_stream_server = VideoStreamServer('0.0.0.0', VIDEO_STREAM_PORT, VIDEO_STREAM_QUALITY)
            self.last_stream_frame_time = time.time()
            self.stream_interval = 1.0 / float(VIDEO_STREAM_RATE)
        

        self.camera_instance = self.fc.getCameraInstance(CAMERA_ID)
        self.mav_data = self.camera_instance.mavlink_data
        INTRINSIC_MATRIX, DISTORTION_COEFFICIENTS = loadCameraCalibration(self.camera_instance)

        # Minimum number of frames to use when computing optical flow
        self.num_frames = 5

        # Optical flow object
        self.of = OpticalFlow(point_mask_radius=100, maximum_track_len=30)

        self.T = np.array([0, 0, 0])

        self.min_temp = 0
        self.max_temp = 50

        self.origin_lat = 0
        self.origin_lon = 0

        # Flag - set this to tell the system to reset
        self.reset_flag = False



    def mav_cmd_user_1_callback(self, param1, param2, param3, param4, param5, param6, param7):
        log("NAVIGATION: Received a command from GCS")
        log(f"{param1}, {param2}, {param3}, {param4}, {param5}, {param6}, {param7}")

        try:
            if int(param1) == COMMAND_SELECT_CAMERA:
                camera_id = int(param2)
                if camera_id == 1:
                    cam = self.fc.getCameraInstance(CAMERA_ID)
                elif camera_id == 2:
                    cam = self.fc.getCameraInstance(SECONDARY_CAMERA_ID)

                if cam is None:
                    print(f"Selected camera {camera_id} is not available")
                    log(f"Selected camera {camera_id} is not available")
                    self.mv.conn.mav.command_ack_send(31010, 1)
                    self.mv.conn.mav.statustext_send(4, "Selected camera is not available".encode())
                    return

                print(f"Camera instance updated to {camera_id}")
                log(f"Camera instance updated to {camera_id}")
                self.loadCameraCalibration(cam)
                self.camera_instance = cam
                self.mv.conn.mav.command_ack_send(31010, 0)

                # RGB Camera
                if camera_id == 1:
                    self.of = OpticalFlow(point_mask_radius=100, maximum_track_len=30)

                # Thermal Camera
                else:
                    self.of = OpticalFlow(point_mask_radius=20, maximum_track_len=10)

            elif int(param1) == COMMAND_RESET:
                self.reset_flag = True
                self.mv.conn.mav.command_ack_send(31010, 0)



        except Exception as e:
            print(e)
            log(e)

            # 2 == MAV_RESULT_DENIED
            self.mv.conn.mav.command_ack_send(31010, 2)
            # 4 == MAV_SEVERITY_WARNING
            self.mv.conn.mav.statustext_send(4, str(e).encode())


    def get_camera_rotation_matrix(self):
        global R_c_a

        # Rotation matrix - NED - aircraft frame
        R_a_n = buildRotationMatrix(self.mav_data.roll, self.mav_data.pitch, self.mav_data.yaw, degrees=False)

        # Rotation matrix of camera in NED frame
        R_c_n = R_c_a @ R_a_n

        return R_c_n

    def compute_world_location(self, K, R, alt, x):

        X = R.T @ np.linalg.inv(K) @ x
        alpha = alt / X[0,2]
        X = alpha * X
        return X
    
    def compute_translation_rotation(self, K, R1, R2, alt, x1s, x2s):

        # Compute the point locations in camera frame of reference
        X1s = []

        for x1 in x1s:
            X = R.T @ np.linalg.inv(K) @ x
            alpha = alt / X[0,2]
            X = alpha * X
            X1s.append(X)

        mean_point = np.mean(X1s, axis=0)


    def rigid_transform_3D(self, p, q):
        if len(p) < 2:
            return None, None

        p = np.array(p)
        q = np.array(q)

        N = p.shape[0]; # total points

        centroid_p = np.mean(p, axis=0)
        centroid_q = np.mean(q, axis=0)

        p = p.reshape((N,3)).T
        q = q.reshape((N,3)).T

        # centre the points
        X = p - np.tile(centroid_p, N)
        Y = q - np.tile(centroid_q, N)

        H = X @ Y.T

        U, S, Vt = np.linalg.svd(H)

        det = np.linalg.det(Vt.T @ U.T)

        d = [1]*(Vt.shape[0]-1)
        d.append(round(det))

        D = np.diag(d)

        R = Vt.T @ D @ U.T

        t = centroid_q - R @ centroid_p

        return R, t

    def _reset(self):
        print("RESETTING")
        log("RESETTING")
        self.origin_lat = self.mav_data.lat
        self.origin_lon = self.mav_data.lon
        self.kf = None
    
    def get_global_coords(self):
        if self.kf is None:
            return None, None
        
        state = self.kf.get_state()
        if state is None:
            return None, None
        
        origin = (self.origin_lat, self.origin_lon)
        if None in origin:
            return None, None

        x = state[0]
        y = state[1]

        distance = np.sqrt(x**2.0 + y**2.0)
        direction = np.arctan2(y, x)

        lat, lon = inverse_haversine(origin, distance, direction, unit=Unit.METERS)

        return lat, lon

    

    def rectifyImage(self, frame, intrinsic_matrix, distortion_coefficients):
        """
        Rectify an image. Returns the rectified frame, and the new camera intrinsic matrix
        """
        size = (frame.shape[1], frame.shape[0])
        new_intrinsics, roi = cv2.getOptimalNewCameraMatrix(intrinsic_matrix, distortion_coefficients, size, 0, size)
        frame = cv2.undistort(frame, intrinsic_matrix, distortion_coefficients, None, new_intrinsics)
        return frame, new_intrinsics

    def run(self):
        global INTRINSIC_MATRIX
        global DISTORTION_COEFFICIENTS
        last_frame_time = 0

        # Process covariance
        q_vx = 0.1
        q_vy = 0.1
        self.kf = None

        while True:
            if self.reset_flag:
                self._reset()
                self.reset_flag = False

            frame = self.camera_instance.drain_last_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            frame, intrinsics = self.rectifyImage(frame, INTRINSIC_MATRIX, DISTORTION_COEFFICIENTS)

            if self.camera_instance.device_type == 'thermal':
                min_temp, max_temp = compute_thermal_bracket(frame)
                if min_temp > self.min_temp:
                    self.min_temp += 0.5
                if min_temp < self.min_temp:
                    self.min_temp -= 0.5

                if max_temp > self.max_temp:
                    self.max_temp += 0.5
                if max_temp < self.max_temp:
                    self.max_temp -= 0.5

                frame = raw_to_thermal_frame(frame, self.min_temp, self.max_temp)


            # Rotation matrix of the camera in NED coordinates
            R_c_n = self.get_camera_rotation_matrix()

            # Fetch altitude from the autopilot
            agl = self.mav_data.agl

            # Ensure we have attitude data
            if (0 in [self.mav_data.roll, self.mav_data.pitch, self.mav_data.yaw]):
                print("Waiting for attitude data")
                log("Waiting for attitude data")
                time.sleep(0.1)
                continue

            # Ensure we have altitude data
            if agl == 0:
                print(self.mav_data.as_dict())
                log(self.mav_data.as_dict())
                print("Waiting for AGL data")
                log("Waiting for AGL data")
                continue
        
            if 0 in (self.origin_lat, self.origin_lon):
                self.origin_lat = self.mav_data.lat
                self.origin_lon = self.mav_data.lon


            # Calculate the groudtruth velocity
            mav_velocity_x = self.mav_data.groundspeed * np.cos(np.radians(self.mav_data.groundcourse))
            mav_velocity_y = self.mav_data.groundspeed * np.sin(np.radians(self.mav_data.groundcourse))

            # Initialize the Kalman filter (using the current estimate)
            if self.kf is None:
                self.kf = KalmanFilter(0, 0, mav_velocity_x, mav_velocity_y, q_vx, q_vy)

            # Run the optical flow - this returns a frame with drawing on it for display purposes
            display_frame = self.of.pass_frame(frame, R_c_n, agl)

            # Amount of time since last iteration
            dt = time.time() - last_frame_time
            last_frame_time = time.time()


            dxs = []
            dys = []
            dts = []
            vxs = []
            vys = []


            As = []
            Bs = []

            current_frame_idx = self.of.get_current_frame_index()

            # For each track in our optical flow object
            for t in self.of.tracks:

                # Get the first frame from this track
                u0, v0, R0, agl0, t0, idx0 = t[0]

                # Get the final frame from this track
                u1, v1, R1, agl1, t1, idx1 = t[-1]

                # Ensure that the total number of frames is greater than the specified minimum
                if len(t) > self.num_frames:

                    # Image position of first and last track
                    x0 = np.array([u0, v0, 1])
                    x1 = np.array([u1, v1, 1])

                    # Compute the theoretical world coordinates of these points
                    X0 = self.compute_world_location(intrinsics, R0, agl0, x0)
                    X1 = self.compute_world_location(intrinsics, R1, agl1, x1)

                    A = X0
                    B = self.compute_world_location(intrinsics, R0, agl0, x1)
                    As.append(np.array(A).T)
                    Bs.append(np.array(B).T)

                    # Negative because we computed the world motion, not the aircraft motion
                    dx = -(X1[0,0] - X0[0,0])
                    dy = -(X1[0,1] - X0[0,1])
                    dt = t1-t0


                    dxs.append(dx)
                    dys.append(dy)
                    dts.append(dt)
                    vxs.append(dx/dt)
                    vys.append(dy/dt)

            if len(dxs)>2 and len(dys)>2 and len(dts)>2:

                mean_dx = np.mean(dxs)
                mean_dy = np.mean(dys)
                mean_dt = np.mean(dts)

                var_x = np.std(vxs)
                var_y = np.std(vys)

                x_vel = np.mean(vxs)
                y_vel = np.mean(vys)

                sum = 0
                for i in range(len(vxs)):
                    sum += (vxs[i] - x_vel) * (vys[i] - y_vel)
                cov = sum / (len(vxs)-1)


                _, t = self.rigid_transform_3D(As, Bs)
                if t is not None:
                    t = t / np.mean(dts)

                R = np.array([[var_x**2.0, cov],[cov, var_y**2.0]]) * 4
                z = [x_vel, y_vel]
                ret = self.kf.update(z, R)

                mavlink_cov = np.array([var_x**2.0/10000, cov/10000, 0, cov/10000, var_y**2.0/10000, 0, 0, 0, 0])
                self.mv.conn.mav.vision_speed_estimate_send(int(time.time()*1000000), x_vel, y_vel, 0, mavlink_cov, current_frame_idx % 255)


                if RERUN_ENABLED:

                    if t is not None:
                        rr.log("/velocity/x/new", rr.Scalar(-t[0]))
                        rr.log("/velocity/y/new", rr.Scalar(-t[1]))
                    else:
                        rr.log("/velocity/x/new", rr.Clear(recursive=True))
                        rr.log("/velocity/y/new", rr.Clear(recursive=True))

                    local_pos_x = self.mav_data.local_pos_n
                    local_pos_y = self.mav_data.local_pos_e
                    estimated_pos_x = ret[0]
                    estimated_pos_y = ret[1]
                    rr.log("/position/x/groundtruth", rr.Scalar(local_pos_x))
                    rr.log("/position/y/groundtruth", rr.Scalar(local_pos_y))
                    rr.log("/position/x/estimated", rr.Scalar(estimated_pos_x))
                    rr.log("/position/y/estimated", rr.Scalar(estimated_pos_y))



                    covmat = self.kf.P[0:2, 0:2]
                    eigvals, eigvecs = np.linalg.eig(covmat)
                    semimajor_idx = np.argmax(eigvals)
                    semiminor_idx = np.argmin(eigvals)
                    semimajor_eigenvalue = eigvecs[semimajor_idx]
                    semiminor_eigenvalue = eigvecs[semiminor_idx]
                    theta = np.arctan2(semimajor_eigenvalue[1], semimajor_eigenvalue[0])
                    rot = rr.RotationAxisAngle([0, 0, 1], radians=theta)

                    cov = rr.Ellipsoids3D(centers=[[estimated_pos_y, estimated_pos_x, 0]], half_sizes=[eigvals[0], eigvals[1], 0], rotation_axis_angles=rot)
                    true_loc = rr.Points3D([[local_pos_y, local_pos_x, 0]], radii=1, colors=[255, 155, 155])
                    estimated_loc = rr.Points3D([[estimated_pos_y, estimated_pos_x, 0]], radii=1, colors=[155, 155, 255])

                    rr.log("/spatial/estimated", estimated_loc)
                    rr.log("/spatial/true", true_loc)
                    rr.log("/spatial/covariance_ellipse", cov)

                    rr.log("/position/x/covariance", rr.Scalar(self.kf.P[0,0]))
                    rr.log("/position/y/covariance", rr.Scalar(self.kf.P[1,1]))




                    rr.log("/velocity/x/groundtruth", rr.Scalar(mav_velocity_x))
                    rr.log("/velocity/y/groundtruth", rr.Scalar(mav_velocity_y))

                    rr.log("/velocity/x/visual_nav", rr.Scalar(x_vel))
                    rr.log("/velocity/y/visual_nav", rr.Scalar(y_vel))

                    rr.log("/velocity/x/visual_nav_filtered", rr.Scalar(ret[2]))
                    rr.log("/velocity/y/visual_nav_filtered", rr.Scalar(ret[3]))

                    bins = np.arange(-30, 30, 0.5)
                    x_vel_hist, edges = np.histogram(vxs, bins=bins)
                    y_vel_hist, edges = np.histogram(vys, bins=bins)


                    rr.log("/velocity/x/histogram", rr.BarChart(x_vel_hist))
                    rr.log("/velocity/y/histogram", rr.BarChart(y_vel_hist))




            try:
                state = self.kf.get_state()
                velocity_x = state[2]
                velocity_y = state[3]
                velocity_abs = np.sqrt(velocity_x**2.0 + velocity_y**2.0)
                direction = np.degrees(np.arctan2(velocity_y, velocity_x))
                direction = direction % 360

                txt_vel = f"Velocity Estimate: {velocity_abs:.2f}"
                txt_hdg = f"Heading Estimate: {direction:.0f}"

                lat, lon = self.get_global_coords()
                self.mv.conn.mav.global_vision_position_estimate_send(int(time.time()*1000000), lat, lon, 0, 0, 0, direction, [0]*21, current_frame_idx % 255)

                if DISPLAY or VIDEO_STREAM:
                    if display_frame.shape[0] < 400:
                        display_frame = cv2.resize(display_frame, (int(display_frame.shape[1]*3), int(display_frame.shape[0]*3)))


                    display_frame = draw_text_with_background(display_frame, txt_vel, cv2.FONT_HERSHEY_SIMPLEX, (20, 30), 1, 2, (255, 255, 255), (0,0,0), 2, 2)
                    display_frame = draw_text_with_background(display_frame, txt_hdg, cv2.FONT_HERSHEY_SIMPLEX, (20, 70), 1, 2, (255, 255, 255), (0,0,0), 2, 2)


                    if DISPLAY:
                        cv2.imshow("frame", display_frame)
                        cv2.waitKey(1)
                    
                    if VIDEO_STREAM:
                        if time.time() - self.last_stream_frame_time > self.stream_interval:
                            self.video_stream_server.send_frame(display_frame)
                            self.last_stream_frame_time = time.time()
                    
            except Exception as e:
                print("Error occurred: ")
                log("Error occurred: ")
                print(e)
                log(e)





if __name__ == "__main__":
    nv = NavigationController()
    nv.run()



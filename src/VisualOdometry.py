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

def loadCameraCalibration(filepath):
    try:
        with open(filepath, 'r') as f:
            dat = json.load(f)

        # Camera intrinsics
        cam_mtx = dat['camera_matrix']
        cam_mtx = np.array(cam_mtx)

        # Distortion coefficients
        dst = np.array(dat['distortion_coefficients'])

        return cam_mtx, dst

    except Exception as e:
        print(f"Exception thrown: {e}")
        return None, None


parser = argparse.ArgumentParser(description="Basic celestial positioning module")

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

if RERUN_ENABLED:
    import rerun as rr
    rr.init("Visual Odom")
    rr.connect_tcp()

# Load camera calibration
CAMERA_CALIBRATION = os.path.join(args.calibration_directory, "camera_calibration.json")
if not os.path.exists(CAMERA_CALIBRATION):
    logging.error(f"NAVIGATION: Camera calibration path does not exist: {CAMERA_CALIBRATION}")

intrinsic_matrix, distortion_coefficients = loadCameraCalibration(CAMERA_CALIBRATION)
CAMERA_INTRINSICS_FLATTENED = list(intrinsic_matrix.flatten())

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
    CAMERA_ROLL = float(dat['camera_roll'])
    CAMERA_PITCH = float(dat['camera_pitch'])
    CAMERA_YAW = float(dat['camera_yaw'])

# Aircraft to camera rotation matrix
R_c_a = buildRotationMatrix(CAMERA_ROLL, CAMERA_PITCH, CAMERA_YAW, degrees=True)


if update:
    with open(VISUAL_NAV_CONFIG, 'w') as f:
        json.dump(dat, f, indent=4)


class KalmanFilter:
    def __init__(self, vx0, vy0, q_vx, q_vy):
        self.x = [vx0, vy0]
        self.xhat = self.x
        self.Q = np.diag([q_vx, q_vy])

        self.F = np.diag([1, 1])
        self.H = np.diag([1, 1])
        self.P = self.Q
        self.y = [0, 0]
        self.S = np.diag([0, 0])
    
    def predict(self):
        return self.x, self.P + self.Q
    
    def update(self, z, R):
        self.xhat, self.Phat = self.predict()

        self.y = z - self.H @ self.xhat
        self.S = self.H @ self.Phat @ self.H.T + R
        self.K = self.Phat @ self.H.T @ np.linalg.inv(self.S)
        self.x = self.xhat + self.K @ self.y
        self.P = (np.eye(2) - self.K @ self.H) @ self.Phat

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




class NavigationController:
    csv_header = "frame%video_frame_index%state%timestamp%camera_roll%camera_pitch%camera_yaw\n"

    def __init__(self):

        if SECONDARY_CAMERA_ID is not None:
            rgb_camera_list = [CAMERA_ID, SECONDARY_CAMERA_ID]
        else:
            rgb_camera_list = [CAMERA_ID]

        log(f"Camera list: {rgb_camera_list}")
        print(f"Camera list: {rgb_camera_list}")
        self.fc = FlightCapture(record=RECORD, log_flight_data=True, rgb_camera_list=rgb_camera_list)
        self.mv = MavlinkHandler(self.fc, port=args.serial_port, baud=args.baudrate, log_flight_data=True,
                                 log_fast_attitude=True)

        if args.server:
            server_thread = Thread(target=run_flight_server, args=[self.fc])
            server_thread.daemon = True
            server_thread.start()

        self.camera_instance = self.fc.getCameraInstance(CAMERA_ID)
        self.mav_data = self.camera_instance.mavlink_data

        self.of = OpticalFlow(point_mask_radius=100, maximum_track_len=30)

        self.T = np.array([0, 0, 0])


    def get_camera_rotation_matrix(self):

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

    
    def run(self):
        last_frame_time = 0

        # Process covariance
        q_x = 0.1
        q_y = 0.1
        self.kf = KalmanFilter(1, 1, q_x, q_y)

        while True:
            frame = self.camera_instance.drain_last_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            R_c_n = self.get_camera_rotation_matrix()
            agl = self.mav_data.agl

            if (0 in [self.mav_data.roll, self.mav_data.pitch, self.mav_data.yaw]):
                print("Waiting for attitude data")
                continue
            if agl == 0:
                print(self.mav_data.as_dict())
                print("Waiting for AGL data")
                continue

            display_frame = self.of.pass_frame(frame, R_c_n, agl)

            dt = time.time() - last_frame_time
            last_frame_time = time.time()


            if frame is None:
                time.sleep(0.01)
                continue


            dxs = []
            dys = []
            dts = []
            vxs = []
            vys = []


            As = []
            Bs = []

            num_frames = 5
            current_frame_idx = self.of.get_current_frame_index()
            for t in self.of.tracks:
                """
                u0, v0, R0, agl0, t0, idx0 = [None]*6
                u1, v1, R1, agl1, t1, idx1 = [None]*6
  
                have_track_0 = False
                have_track_1 = False
                for u, v, R, agl, _t, idx in t:
                    if idx == current_frame_idx - num_frames:
                        u0, v0, R0, agl0, t0, idx0 = u, v, R, agl, _t, idx
                        have_track_0 = True

                    if idx == current_frame_idx:
                        u1, v1, R1, agl1, t1, idx1 = u, v, R, agl, _t, idx
                        have_track_1 = True


                if have_track_0 and have_track_1:
                """

                u0, v0, R0, agl0, t0, idx0 = t[0]
                u1, v1, R1, agl1, t1, idx1 = t[-1]

                if len(t) > num_frames:

                    x0 = np.array([u0, v0, 1])
                    x1 = np.array([u1, v1, 1])

                    X0 = self.compute_world_location(intrinsic_matrix, R0, agl0, x0)
                    X1 = self.compute_world_location(intrinsic_matrix, R1, agl1, x1)

                    A = X0
                    B = self.compute_world_location(intrinsic_matrix, R0, agl0, x1)
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

            if len(dxs)>0 and len(dys)>0 and len(dts)>0:

                mean_dx = np.mean(dxs)
                mean_dy = np.mean(dys)
                mean_dt = np.mean(dts)

                var_x = np.std(vxs)
                var_y = np.std(vys)

                x_vel = np.mean(vxs)
                y_vel = np.mean(vys)

                _, t = self.rigid_transform_3D(As, Bs)
                if t is not None:
                    t = t / np.mean(dts)

                R = np.diag([var_x**2.0, var_y**2.0])
                z = [x_vel, y_vel]
                ret = self.kf.update(z, R)


                if RERUN_ENABLED:

                    if t is not None:
                        rr.log("/velocity/x/new", rr.Scalar(-t[0]))
                        rr.log("/velocity/y/new", rr.Scalar(-t[1]))
                    else:
                        rr.log("/velocity/x/new", rr.Clear(recursive=True))
                        rr.log("/velocity/y/new", rr.Clear(recursive=True))

                    mav_velocity_x = self.mav_data.groundspeed * np.cos(np.radians(self.mav_data.groundcourse))
                    mav_velocity_y = self.mav_data.groundspeed * np.sin(np.radians(self.mav_data.groundcourse))

                    rr.log("/velocity/x/groundtruth", rr.Scalar(mav_velocity_x))
                    rr.log("/velocity/y/groundtruth", rr.Scalar(mav_velocity_y))

                    rr.log("/velocity/x/visual_nav", rr.Scalar(x_vel))
                    rr.log("/velocity/y/visual_nav", rr.Scalar(y_vel))

                    rr.log("/velocity/x/visual_nav_filtered", rr.Scalar(ret[0]))
                    rr.log("/velocity/y/visual_nav_filtered", rr.Scalar(ret[1]))

                    bins = np.arange(-30, 30, 0.5)
                    x_vel_hist, edges = np.histogram(vxs, bins=bins)
                    y_vel_hist, edges = np.histogram(vys, bins=bins)


                    rr.log("/velocity/x/histogram", rr.BarChart(x_vel_hist))
                    rr.log("/velocity/y/histogram", rr.BarChart(y_vel_hist))





            cv2.imshow("frame", display_frame)
            cv2.waitKey(1)





if __name__ == "__main__":
    nv = NavigationController()
    nv.run()



import bge
import numpy as np
from math import *
from collections import OrderedDict
import time
import socket
import numpy as np
import bpy
import gpu
import redis
import struct
import mathutils


PICAM = {
    'sensor_width': 3.691,
    'sensor_height': 2.813,
    'focal_length': 3.04,
    'clip_start': 0.01,
    'clip_end': 10000
}

FT_TO_M = 0.3048


def haversine(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance in kilometers between two points 
    on the earth (specified in decimal degrees)
    """

    # convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371 # Radius of earth in kilometers
    return c * r

def bearing(lat1, lon1, lat2, lon2):
    """
    Calculate the bearing from point 1 to point 2
    """
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    dlon = lon2 - lon1

    y = sin(dlon) * cos(lat2)
    x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
    b = atan2(y, x)

    bearing = degrees(b)

    return bearing


def get_xy_offset_from_origin_mercator(origin_lat, origin_lon, target_lat, target_lon):

    target_range = haversine(origin_lat, origin_lon, target_lat, target_lon) * 1000

    # Mercator projection requires a scaling by cos(lat)
    target_range = target_range / cos(radians(origin_lat))

    target_bearing = bearing(origin_lat, origin_lon, target_lat, target_lon)

    offset_x = target_range * sin(radians(target_bearing))
    offset_y = target_range * cos(radians(target_bearing))

    return offset_x, offset_y


def rotation_matrix(phi, theta, psi, degrees=True):
    """
    phi = roll (degrees)
    theta = pitch (degrees)
    psi = yaw (degrees)
    """
    if degrees:
        phi = radians(phi)
        theta = radians(theta)
        psi = radians(psi)

    C11 = cos(theta) * cos(psi)
    C12 = cos(theta) * sin(psi)
    C13 = -sin(theta)
    C21 = -cos(phi) * sin(psi) + sin(phi) * sin(theta) * cos(psi)
    C22 = cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi)
    C23 = sin(phi) * cos(theta)
    C31 = sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi)
    C32 = -sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi)
    C33 = cos(phi) * cos(theta)
    C = np.matrix(np.array([[C11, C12, C13], [C21, C22, C23], [C31, C32, C33]]))
    return C



def dcm_to_euler_angle(x):
    """
    Converts a DCM into its Euler angles (radians)

    :ret phi: roll
    :ret theta: pitch
    :ret psi: yaw
    """
    phi = atan2(x[1, 2], x[2, 2])
    theta = -asin(x[0, 2])
    psi = atan2(x[0, 1], x[0, 0])
    return phi, theta, psi


#def cv_to_blender_camera_euler_angles(x, y, z):

class JSBSimCameraController(bge.types.KX_PythonComponent):
    mavlink_port = "tcp:localhost:5762"
    mavlink_port = "udp:localhost:14550"

    jsbsim_port = 5510
    jsbsim_ip = "127.0.0.1"

    args = OrderedDict([])

    def start(self, args):
        print("Initializing")

        self.context = bpy.context

        self.C_cam_ac = rotation_matrix(0, 0, 0)

        # Redis
        self.r = None
        self.redis_id = 'camera1'
        self.r = redis.Redis(host="localhost", port=6379, db=0)

        # Blender Scene
        self.scene = self.context.scene
        self.camera = self.scene.camera

        # Configure the camera
        self.camera.data.lens_unit = "MILLIMETERS"
        self.camera.data.sensor_width = PICAM['sensor_width']
        self.camera.data.sensor_height = PICAM['sensor_height']
        self.camera.data.lens = PICAM['focal_length']
        self.camera.data.clip_start = PICAM['clip_start']
        self.camera.data.clip_end = PICAM['clip_end']


        # Setup the socket to receive from JSBSim
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.jsbsim_ip, self.jsbsim_port))

        # Get the scene origin point
        self.scene_origin_lat = self.scene['latitude']
        self.scene_origin_lon = self.scene['longitude']

        if self.scene_origin_lon is None or self.scene_origin_lat is None:
            print(f"Failed to find the GeoScene origin")
            self.cleanup()
            return {'FINISHED'}

        print(f"Scene origin set to {self.scene_origin_lat} {self.scene_origin_lon}")


    def to_redis(self, arr):
        h, w = arr.shape[:2]

        print(arr.dtype)
        print(arr.shape)

        shape = struct.pack('>II', h, w)
        encoded = shape + arr.tobytes()

        self.r.set(self.redis_id, encoded)
        return


    def update(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            data = data.decode()

            values = data.split(",")
            values = [x.strip() for x in values]

            time = float(values[0])
            altitude = float(values[1]) * FT_TO_M + 0.1 # to keep above the ground
            roll = np.radians(float(values[2]))
            pitch = np.radians(float(values[3]))
            yaw = np.radians(float(values[4]))
            aoa_deg = float(values[5])
            beta_deg = float(values[6])
            lat_geocentric = np.radians(float(values[7]))
            lon = float(values[8])
            lat = float(values[9])

            C_ac = rotation_matrix(roll, pitch, yaw, degrees=False)
            C_cam = np.matmul(self.C_cam_ac, C_ac)
            cam_roll, cam_pitch, cam_yaw = dcm_to_euler_angle(C_cam)

            x, y = get_xy_offset_from_origin_mercator(self.scene_origin_lat, self.scene_origin_lon, lat, lon)

            self.camera.rotation_euler = (cam_pitch, cam_roll, -cam_yaw)
            rotmat = self.camera.rotation_euler.to_matrix()
            self.camera.location = (x, y, altitude)

            self.object.worldPosition = (x, y, altitude)
            orientation = rotation_matrix(cam_pitch, cam_roll, -cam_yaw)
            self.object.worldOrientation = rotmat

            try:
                ac_roll, ac_pitch, ac_yaw = dcm_to_euler_angle(C_ac)
                ac = self.scene.objects['aircraft']
                ac.rotation_euler = (-ac_roll+pi/2, ac_pitch, -ac_yaw-pi/2)
                ac.location = (x, y, altitude + 0.5)

            # Aircraft not found
            except KeyError:
                pass

        # Don't hog the CPU cycles waiting for data
        except TimeoutError:
            pass

        # The first transmission contains labels
        except ValueError:
            pass

    def cleanup(self):
        self.sock.close()

        if self.r is not None and self.r:
            if self.r.connection is not None:
                self.r.connection.disconnect()


    def __del__(self):
        self.cleanup()
    


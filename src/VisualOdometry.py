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
parser.add_argument('-r', '--record-video', action='store_true', help="Record video if enabled")
parser.add_argument('-e', '--server', action='store_true', help="Enables server if set")
parser.add_argument('--sitl', action='store_true', help="Use SITL instead of Serial interface")


VERSION = "1.0.0"

args = parser.parse_args()

default_config = {
    "image_width_pixels": 1920,
    "image_height_pixels": 1080,
}

CAMERA_ID = args.camera_id
SECONDARY_CAMERA_ID = args.secondary_camera_id
MAV_SERIAL_PORT = args.serial_port
MAV_SERIAL_BAUDRATE = args.baudrate
GRAPH = args.graph
DISPLAY = args.display
RECORD = args.record_video

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

if update:
    with open(VISUAL_NAV_CONFIG, 'w') as f:
        json.dump(dat, f, indent=4)


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


    def run(self):
        while True:
            frame = self.camera_instance.drain_last_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            cv2.imshow("frame", frame)
            cv2.waitKey(1)




if __name__ == "__main__":
    nv = NavigationController()
    nv.run()



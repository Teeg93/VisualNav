import threading

import pymavlink.dialects.v20.ardupilotmega
from pymavlink import mavutil
import time
import socket
import numpy as np
from .Geo import *

import bpy
import gpu

import importlib

try:
    import BlenderGIS
except ModuleNotFoundError:
    try:
        BlenderGIS = importlib.import_module('BlenderGIS-master')
    except ModuleNotFoundError:
        print("WARNING: Could not locate BlenderGIS!")


target_messages = [
    'LOCAL_POSITION_NED',
    'GLOBAL_POSITION_INT',
    'ATTITUDE',
    'SIMSTATE'
]

PICAM = {
    'sensor_width': 3.691,
    'sensor_height': 2.813,
    'focal_length': 3.04,
    'clip_start': 0.01,
    'clip_end': 10000
}


#C_blend_reg = rotation_matrix(180, 0, 0)
C_cam_ac = rotation_matrix(0, 0,0)

class BlenderMavlinkOperator(bpy.types.Operator):
    bl_idname = 'mavlink.camera'
    bl_label = 'Mavlink interface for controlling Blender Camera'

    mavlink_port = "tcp:localhost:5762"
    mavlink_port = "udp:localhost:14550"


    jsbsim_port = 5510
    jsbsim_ip = "127.0.0.1"

    def __init__(self):
        self.tx = 0
        self.ty = 0
        self.tz = 0
        self.rx = 0
        self.ry = 0
        self.rz = 0


        # The Blender scene
        self.scene = None
        self.camera = None
        self.conn = None

    def modal(self, context, event):
        if event.type in ['ESC']:
            print("STOPPING MAVLINK")
            self.cleanup()
            return {'FINISHED'}

        if event.type == "TIMER":
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
                C_cam = np.matmul(C_cam_ac, C_ac)
                cam_roll, cam_pitch, cam_yaw = dcm_to_euler_angle(C_cam)

                x, y = get_xy_offset_from_origin_mercator(self.scene_origin_lat, self.scene_origin_lon, lat, lon)

                self.camera.rotation_euler = (cam_pitch, cam_roll, -cam_yaw)
                self.camera.location = (x, y, altitude)

                try:
                    ac_roll, ac_pitch, ac_yaw = dcm_to_euler_angle(C_ac)
                    ac = context.scene.objects['aircraft']
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



        return {'RUNNING_MODAL'}

    def invoke(self, context, event):
        self.scene = context.scene
        self.camera = self.scene.camera

        self.camera.data.lens_unit = "MILLIMETERS"
        self.camera.data.sensor_width = PICAM['sensor_width']
        self.camera.data.sensor_height = PICAM['sensor_height']
        self.camera.data.lens = PICAM['focal_length']
        self.camera.data.clip_start = PICAM['clip_start']
        self.camera.data.clip_end = PICAM['clip_end']



        self.conn = mavutil.mavlink_connection(self.mavlink_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1)
        self.sock.bind((self.jsbsim_ip, self.jsbsim_port))

        #TODO: Check this, probably need exception conditions if there is no geoscene or BlenderGIS available
        self.geoscene = BlenderGIS.geoscene.GeoScene()
        self.scene_origin_lon, self.scene_origin_lat = self.geoscene.getOriginGeo()


        if self.scene_origin_lon is None or self.scene_origin_lat is None:
            print(f"Failed to find the GeoScene origin")
            return {'FINISHED'}

        print(f"Scene origin set to {self.scene_origin_lat} {self.scene_origin_lon}")

        print(f"Mavlink waiting for heartbeat on port {self.mavlink_port}")
        heartbeat = self.conn.wait_heartbeat(timeout=5)
        if heartbeat is None:
            print(f"Failed to connect")
            return {'FINISHED'}
        else:

            """
            # Request fast attitude updates
            self.conn.mav.command_long_send(target_system=1,
                                            target_component=0,
                                            command=511,
                                            confirmation=0,
                                            param1=30,
                                            param2=100000,
                                            param3=0,
                                            param4=0,
                                            param5=0,
                                            param6=0,
                                            param7=0)
            """

            self._timer = context.window_manager.event_timer_add(1 / 30, window=context.window)
            context.window_manager.modal_handler_add(self)
            return {'RUNNING_MODAL'}

    def handle_mav_msg(self, m):
        if m.get_type() == "ATTITUDE":
            self.camera.rotation_euler = (m.roll, m.pitch, m.yaw)
            print(f"camera rotation set: {m.roll, m.pitch, m.yaw}")

    def cleanup(self):
        if self.conn is not None:
            self.conn.close()


def menu_func(self, context):
    self.layout.operator(BlenderMavlinkOperator.bl_idname, text="Mavlink operator for controlling camera")


def register():
    bpy.utils.register_class(BlenderMavlinkOperator)
    bpy.types.VIEW3D_MT_view.append(menu_func)

def unregister():
    bpy.utils.unregister_class(BlenderMavlinkOperator)
    bpy.types.VIEW3D_MT_view.remove(menu_func)



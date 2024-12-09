import threading

import pymavlink.dialects.v20.ardupilotmega
from pymavlink import mavutil
import time
import socket
import numpy as np

import bpy

target_messages = [
    'LOCAL_POSITION_NED',
    'GLOBAL_POSITION_INT',
    'ATTITUDE',
    'SIMSTATE'
]


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
            print("RIGHT CLICK -- STOPPING")
            self.cleanup()
            return {'FINISHED'}

        if event.type == "TIMER":
            try:
                data, addr = self.sock.recvfrom(1024)

                data = data.decode()

                values = data.split(",")
                values = [x.strip() for x in values]

                time = float(values[0])
                altitude = float(values[1])
                roll = np.radians(float(values[2]))
                pitch = np.radians(float(values[3]))
                yaw = np.radians(float(values[4]))
                aoa_deg = float(values[5])
                beta_deg = float(values[6])
                lat_geocentric = np.radians(float(values[7]))
                lon = float(values[8])
                lat = float(values[9])

                #print(f"Lat: {lat}, Lon: {lon}, roll: {roll}, pitch: {pitch}, yaw: {yaw}")

                self.camera.rotation_euler = (roll, pitch, yaw)

            except TimeoutError:
                pass


        return {'RUNNING_MODAL'}

    def invoke(self, context, event):
        self.scene = context.scene
        self.camera = self.scene.camera
        self.conn = mavutil.mavlink_connection(self.mavlink_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1)
        self.sock.bind((self.jsbsim_ip, self.jsbsim_port))

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



import threading

import pymavlink.dialects.v20.ardupilotmega
from pymavlink import mavutil
import time

import bpy

target_messages = [
    'LOCAL_POSITION_NED',
    'GLOBAL_POSITION_INT',
    'ATTITUDE',
    'SIMSTATE'
]

class BlenderMavlink:
    def __init__(self, scene, port, baud=None):
        self.tx = 0
        self.ty = 0
        self.tz = 0
        self.rx = 0
        self.ry = 0
        self.rz = 0

        self.port = port
        self.baud = baud

        # The Blender scene
        self.scene = scene
        self.camera = self.scene.camera

        self.conn = mavutil.mavlink_connection(port, baud=baud)

        print(f"Mavlink waiting for heartbeat on port {port}")
        heartbeat = self.conn.wait_heartbeat()
        if heartbeat is None:
            print(f"Failed to connect")
        else:
            print(heartbeat)
            print(f"Mavlink connected")


        self._f_run = False
        self._exit = False
        self.t = threading.Thread(target=self._run)
        self.t.daemon = True
        self.t.start()

    def run(self):
        self._f_run = True
    
    def stop(self):
        self.f_run = False
    
    def exit(self):
        self._exit = True


    
    def _run(self):

        while True:
            if self._exit:
                print("Exiting...")
                break

            if not self._f_run:
                time.sleep(0.1)
                continue

            try:
                msg = self.conn.recv_match(type=target_messages, blocking=False)
                if msg is not None:
                    self.handle_mav_msg(msg)
                else:
                    time.sleep(0.02)

            
            except KeyboardInterrupt:
                print("Exiting...")
                break
            
            except Exception as e:
                print("ERROR: Unhandled exception: ")
                print(e)


    def handle_mav_msg(self, m):

        print(f"Received a message: {m}")
        if m.get_type() == "ATTITUDE":
            self.camera.rotation_euler = (m.roll, m.pitch, m.yaw)
            bpy.ops.render.render()

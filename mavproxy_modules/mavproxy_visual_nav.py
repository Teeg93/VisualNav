#!/usr/bin/env python
import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings


class VisualNav(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(VisualNav, self).__init__(mpstate, "VisualNav", "")
        self.status_callcount = 0
        self.boredom_interval = 10  # seconds
        self.last_bored = time.time()

        self.packets_mytarget = 0
        self.packets_othertarget = 0

        self.add_command('visual_nav', self.cmd_visual_nav, "visual navigation module", ['cam', 'reset'])
        self.time_boot_ms = 0

        self.last_map_update_time = 0
        self.last_map_update_interval = 2

        self.estimated_lat = 0
        self.estimated_lon = 0
        self.estimated_yaw = 0

        self.target_heading = 0
        self.target_distance = 0

        self.map_vehicle_name = "visual-nav"
        self.map_vehicle_colour = "orange"
        self.map_vehicle_type = "plane"
        try:
            self.module('map').create_vehicle_icon(self.map_vehicle_name, self.map_vehicle_colour)
        except:
            print("Ensure map is loaded")
            raise(Exception)

        self.last_position_update = 0


    def usage(self):
        '''show help on command line options'''
        return "Usage: visual_nav < cam | reset >"

    def cmd_visual_nav(self, args):
        if len(args) == 0:
            print(self.usage())
            return

        if args[0].lower() == "cam":

            if len(args) < 2:
                print("Not enough arguments")
                print("Usage: visual_nav cam < rgb | thermal >")
                return

            if not args[1].lower() in ("rgb", "thermal"):
                print(f"Invalid argument: {args[1]}")
                print("Usage: visual_nav cam < rgb | thermal >")
                return

            if args[1].lower() == "rgb":
                self.master.mav.command_long_send(self.target_system, 191, 31010, 0, 1, 1, 0, 0, 0, 0, 0)

            if args[1].lower() == "thermal":
                self.master.mav.command_long_send(self.target_system, 191, 31010, 0, 1, 2, 0, 0, 0, 0, 0)
        
        if args[0].lower() == "reset":
            self.master.mav.command_long_send(self.target_system, 191, 31010, 0, 2, 0, 0, 0, 0, 0, 0)



    def idle_task(self):
        '''called rapidly by mavproxy'''
        if time.time() - self.last_map_update_time > self.last_map_update_interval:
            self.last_map_update_time = time.time()
            if self.estimated_lat and self.estimated_lon:
                try:
                    self.module('map').update_vehicle_icon_to_loc(self.map_vehicle_name, self.map_vehicle_type, self.map_vehicle_colour, display=True, latlon=(self.estimated_lat, self.estimated_lon), yaw=self.estimated_yaw)
                except:
                    self.module('map').create_vehicle_icon(self.map_vehicle_name, self.map_vehicle_colour)
                    self.module('map').map.set_position(self.map_vehicle_name, (self.estimated_lat, self.estimated_lon), rotation=self.estimated_yaw, label=self.map.vehicle_name)

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        if m.get_type() == "GLOBAL_VISION_POSITION_ESTIMATE":
            self.estimated_lat = m.x
            self.estimated_lon = m.y
            self.estimated_yaw = m.yaw



def init(mpstate):
    '''initialise module'''
    return VisualNav(mpstate)

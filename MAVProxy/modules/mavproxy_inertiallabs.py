#!/usr/bin/env python
'''InertialLabs'''

import time, os

from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil
import sys, traceback

class InertiallabsModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(InertiallabsModule, self).__init__(mpstate, "inertiallabs", public=True)
        self.add_command('enable_gnss', self.cmd_enable_gnss, "enable_gnss")
        self.add_command('disable_gnss', self.cmd_disable_gnss, "disable_gnss")
        self.add_command('start_vg3dclb_flight', self.cmd_start_vg3d_calibration_in_flight, "start_vg3dclb_flight")
        self.add_command('stop_vg3dclb_flight', self.cmd_stop_vg3d_calibration_in_flight, "stop_vg3dclb_flight")

    def cmd_enable_gnss(self, args):
        self.master.mav.command_long_send(
            self.master.target_system,  # target_system
            self.master.target_component, # target_component
            mavutil.mavlink.MAV_CMD_INERTIALLABS_AHRS_SEND,
            0, # confirmation
            mavutil.mavlink.ENABLE_GNSS, # param1
            0, # param2
            0, # param3
            0, # param4
            0, # param5
            0, # param6
            0) # param7

    def cmd_disable_gnss(self, args):
        self.master.mav.command_long_send(
            self.master.target_system,  # target_system
            self.master.target_component, # target_component
            mavutil.mavlink.MAV_CMD_INERTIALLABS_AHRS_SEND,
            0, # confirmation
            mavutil.mavlink.DISABLE_GNSS, # param1
            0, # param2
            0, # param3
            0, # param4
            0, # param5
            0, # param6
            0) # param7

    def cmd_start_vg3d_calibration_in_flight(self, args):
        self.master.mav.command_long_send(
            self.master.target_system,  # target_system
            self.master.target_component, # target_component
            mavutil.mavlink.MAV_CMD_INERTIALLABS_AHRS_SEND,
            0, # confirmation
            mavutil.mavlink.START_VG3D_CALIBRATION_IN_FLIGHT, # param1
            0, # param2
            0, # param3
            0, # param4
            0, # param5
            0, # param6
            0) # param7

    def cmd_stop_vg3d_calibration_in_flight(self, args):
        self.master.mav.command_long_send(
            self.master.target_system,  # target_system
            self.master.target_component, # target_component
            mavutil.mavlink.MAV_CMD_INERTIALLABS_AHRS_SEND,
            0, # confirmation
            mavutil.mavlink.STOP_VG3D_CALIBRATION_IN_FLIGHT, # param1
            0, # param2
            0, # param3
            0, # param4
            0, # param5
            0, # param6
            0) # param7


def init(mpstate):
    return InertiallabsModule(mpstate)

#!/usr/bin/env python
'''
RC Receiver Module
Robert Clarke, December 2017

This module retrieves RC_CHANNELS_RAW packets from a designated source and
forwards them to master as RC_OVERRIDE packets, allowing use of a PixHawk as a
PC radio adapter.
'''

import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings


class RCReceiver(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(RCReceiver, self).__init__(mpstate, "RCReceiver", "")
        self.receiver_settings = mp_settings.MPSettings(
            [ ('source'  , str, "com3"),
              ('baudrate', int, 57600 )
              ]
            )
        self.override = [0]*16
        self.last_override = [0]*16
        self.override_period = mavutil.periodic_event(20)
        self.connection = mavutil.mavlink_connection(self.receiver_settings.source,
                                       autoreconnect=True,
                                       source_system=self.settings.source_system,
                                       baud=self.receiver_settings.baudrate)

    def idle_task(self):
        # Check for message from other end and check if it is an RC_CHANNELS_RAW
        msg = self.connection.recv_msg()
        if (msg is not None):
            if (msg.get_type() == 'RC_CHANNELS_RAW'):
                # Store the new input as override values
                self.override[msg.port + 0] = msg.chan1_raw
                self.override[msg.port + 1] = msg.chan2_raw
                self.override[msg.port + 2] = msg.chan3_raw
                self.override[msg.port + 3] = msg.chan4_raw
                self.override[msg.port + 4] = msg.chan5_raw
                self.override[msg.port + 5] = msg.chan6_raw
                self.override[msg.port + 6] = msg.chan7_raw
                self.override[msg.port + 7] = msg.chan8_raw
        # Check if need to send override
        if self.override_period.trigger():
            if (self.override != [0]*16 or            # If have received a RAW packet
                self.override != self.last_override):
                self.last_override = self.override[:]
                self.send_rc_override()

    def send_rc_override(self):
        # RC_CHANNELS_OVERRIDE only allows for the first 8 channels
        chan8 = self.override[:8]
        self.master.mav.rc_channels_override_send(self.target_system,
                                                  self.target_component,
                                                  *chan8)

def init(mpstate):
    '''initialise module'''
    return RCReceiver(mpstate)

#!/usr/bin/env python
'''
RC Forwarding Module
Robert Clarke, December 2017

This module forwards RC_CHANNELS_RAW packets to a designated output as RC_OVERRIDE allowing
use of a PixHawk as a PC radio adapter.
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


class RCForwarder(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(RCForwarder, self).__init__(mpstate, "RCForwarder", "")
        self.forwarder_settings = mp_settings.MPSettings(
            [ ('device'  , str, "udpin:0.0.0.0:14551"),
              ('baudrate', int, 57600                )
              ]
            )
        self.override = [0]*16
        self.last_override = [0]*16
        self.override_period = mavutil.periodic_event(20)
        self.connection = mavutil.mavlink_connection(self.forwarder_settings.device,
                                       autoreconnect=True,
                                       source_system=self.settings.source_system,
                                       baud=self.forwarder_settings.baudrate)

    def idle_task(self):
        # Check for message from other end
        # Has side effect of setting up port for sending
        msg = self.connection.recv_msg()
        if self.override_period.trigger():
            if (self.override != [0]*16 or
                self.override != self.last_override):
                self.last_override = self.override[:]
                self.send_rc_override()

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        if m.get_type() == 'RC_CHANNELS_RAW':
            self.override[m.port + 0] = m.chan1_raw
            self.override[m.port + 1] = m.chan2_raw
            self.override[m.port + 2] = m.chan3_raw
            self.override[m.port + 3] = m.chan4_raw
            self.override[m.port + 4] = m.chan5_raw
            self.override[m.port + 5] = m.chan6_raw
            self.override[m.port + 6] = m.chan7_raw
            self.override[m.port + 7] = m.chan8_raw
            #print(self.override)

    def send_rc_override(self):
        chan8 = self.override[:8]
        self.connection.mav.rc_channels_override_send(1,1,*chan8)
        #print('Sent override')

def init(mpstate):
    '''initialise module'''
    return RCForwarder(mpstate)

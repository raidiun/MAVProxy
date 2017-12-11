#!/usr/bin/env python
'''Trajectory uploader'''

from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil
import pymavlink.dialects.v20.ardupilotmega as apm


class TrajectoryUploader(mp_module.MPModule):
    def __init__(self, mpstate):
        super(TrajectoryUploader, self).__init__(mpstate, "trajectory", "Trajectory uploading module")
        self.add_command('trajectory', self.handle_command, "trajectory module", ['upload (TRAJECTORY_FILE)'])
        self.trajectory_item_message_list = []
        
    def mavlink_packet(self, msg):
        '''Handle mavlink packet'''
        # If is TRAJECTORY_REQUEST, respond with appropriate TRAJECTORY_ITEM
        #print(msg.get_type())
        if msg.get_type() == 'TRAJECTORY_REQUEST':
            self.master.mav.send(self.trajectory_item_message_list[int(msg.seq)])
    
    def handle_command(self,args):
        if args[0] == 'upload':
            self.cmd_upload(args[1:])
    
    def cmd_upload(self,args):
        '''Upload a trajectory file to the UAV'''
        trajFile = open(args[0],'r')
        next(trajFile) # Get rid of header line
        seq = 0
        for line in trajFile:
            # Add line to storage
            fields = [ field.strip() for field in line.split(',')]
            self.trajectory_item_message_list.append(apm.MAVLink_trajectory_item_message(
                self.target_system,\
                self.target_component,\
                seq, \
                int(fields[0]),   \
                int(fields[1]),   \
                int(fields[2]),   \
                int(fields[3]),   \
                int(fields[4]),   \
                float(fields[5]), \
                float(fields[6]), \
                float(fields[7])  \
                ))
            seq += 1
        # Now all lines are stored, send a TRAJECTORY_COUNT message to initiate read
        msg = apm.MAVLink_trajectory_count_message(self.target_system,self.target_component,seq)
        self.master.mav.send(msg)
        
        
def init(mpstate):
    '''initialise module'''
    return TrajectoryUploader(mpstate)

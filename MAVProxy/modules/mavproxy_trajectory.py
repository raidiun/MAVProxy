#!/usr/bin/env python
'''Trajectory uploader'''

from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil
import pymavlink.dialects.v20.ardupilotmega as apm


class TrajectoryUploader(mp_module.MPModule):
    def __init__(self, mpstate):
        super(TrajectoryUploader, self).__init__(mpstate, "trajectory", "Trajectory uploading module")
        self.add_command('trajectory', self.handle_command, "Up/download or clear a trajectory", ['<help|upload|clear>','upload (FILENAME)'])
        self.trajectory_item_message_list = []
        self.count = 0
        
    def mavlink_packet(self, msg):
        '''Handle mavlink packet'''
        if msg.get_type() == 'TRAJECTORY_REQUEST':
        # TRAJECTORY_REQUEST, respond with appropriate TRAJECTORY_ITEM
            if msg.seq > len(self.trajectory_item_message_list):
                    self.master.mav.send(apm.MAVLink_trajectory_ack_message(
                        self.target_system,   \
                        self.target_component,\
                        apm.TRAJECTORY_ERROR))
                    return
            self.master.mav.send(self.trajectory_item_message_list[int(msg.seq)])
        
        if msg.get_type() == 'TRAJECTORY_ACK':
            # TRAJECTORY_ACK, log response
            if msg.response == apm.TRAJECTORY_ERROR:
                response = 'TRAJECTORY_ERROR'
            elif msg.response == apm.TRAJECTORY_ACCEPTED:
                response = 'TRAJECTORY_ACCEPTED'
            else:
                response = 'Unknown: {}'.format(msg.response)
            print('Vehicle responded with {}'.format(response))
        
        if msg.get_type() == 'TRAJECTORY_COUNT':
            # TRAJECTORY_COUNT, begin requesting items
            self.count = msg.count
            self.master.mav.send(apm.MAVLink_trajectory_request_message(
                self.target_system,\
                self.target_component,\
                0))
        
        if msg.get_type() == 'TRAJECTORY_ITEM':
            # TRAJECTORY_ITEM, print and if not last, request next
            print(msg)
            nextSeq = msg.seq + 1
            if nextSeq >= self.count:
                self.master.mav.send(apm.MAVLink_trajectory_ack_message(
                    self.target_system,   \
                    self.target_component,\
                    apm.TRAJECTORY_ACCEPTED))
                return
            self.master.mav.send(apm.MAVLink_trajectory_request_message(
                self.target_system,\
                self.target_component,\
                nextSeq))
    
    def handle_command(self,args):
        if len(args) == 0:
            self.print_help()
            return
        if args[0] == 'help':
            self.print_help()
        if args[0] == 'upload':
            self.cmd_upload(args[1:])
        if args[0] == 'clear':
            self.cmd_clear()
        if args[0] == 'list':
            self.cmd_list()
    
    def print_help(self):
        print('trajectory <help|list|clear>')
        print('trajectory upload <filename>')
    
    def cmd_upload(self,args):
        '''Upload a trajectory file to the UAV'''
        trajFile = open(args[0],'r')
        next(trajFile) # Get rid of header line
        seq = 0
        for line in trajFile:
            # Add line to storage
            fields = [field.strip() for field in line.split(',')]
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
    
    def cmd_clear(self):
        '''Clear the UAV's trajectory'''
        msg = apm.MAVLink_trajectory_clear_all_message(self.target_system,self.target_component)
        self.master.mav.send(msg)
    
    def cmd_list(self):
        '''Retrieve trajectory items from the UAV'''
        msg = apm.MAVLink_trajectory_request_list_message(self.target_system,self.target_component)
        self.master.mav.send(msg)
    
def init(mpstate):
    '''initialise module'''
    return TrajectoryUploader(mpstate)

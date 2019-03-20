#!/usr/bin/env python

import rospy
import dynamic_reconfigure.client as reconfig

# import ros msgs
from std_msgs.msg import Int8
from ema_common_msgs.msg import Stimulator

# global variables
global stim_current
global pulse_width
global update_values

stim_current = 0
pulse_width = 0
update_values = False

# everytime sth changes in the server 
def server_callback(config):
    global stim_current
    global pulse_width
    update_values = True

    if config['current'] - stim_current > 2:
        stim_current += 2
    else:
        stim_current = config['current']


def main():
    global stimMsg
    global stim_current
    global pulse_width
    global update_values

    # init matrix node
    rospy.init_node('matrix', anonymous=False)

    # communicate with the dynamic server
    dyn_params = reconfig.Client('server', config_callback = server_callback)

    # get node config
    config_dict = rospy.get_param('/ema/matrix')

    StimChannels = config_dict['channels']
    StimMode = 'single'
    StimFreq = config_dict['freq']

    # build basic stimulator message
    stimMsg = Stimulator()
    
    # list published topics
    pub = {}
    pub['singlepulse'] = rospy.Publisher('stimulator/single_pulse', Stimulator, queue_size=10)
    pub['signal'] = rospy.Publisher('control/stimsignal', Int8, queue_size=10)
    
    # define loop rate (in hz)
    rate = rospy.Rate(StimFreq)

    # node loop
    while not rospy.is_shutdown():
        for n, channel in enumerate(StimChannels):
            # # parameters update
            # if update_values is True:
            #     params = { 'current' : stim_current, 'pulse_width' : pulse_width }
            #     dyn_params.update_configuration(params)
            #     update_values = False

            stimMsg.channel = channel
            stimMsg.mode = StimMode
            stimMsg.pulse_width = pulse_width
            stimMsg.pulse_current = stim_current

            # send stimulator update
            pub['singlepulse'].publish(stimMsg)
            
            # send signal update
            pub['signal'].publish(StimPulseCurrent)

            # wait for next control loop
            rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

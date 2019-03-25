#!/usr/bin/env python

import rospy
import dynamic_reconfigure.client as reconfig

# import ros msgs
from std_msgs.msg import Int8, Int8MultiArray
from ema_common_msgs.msg import Stimulator

# global variables
global onoff
global stim_current
global pulse_width

onoff = False
stim_current = 0
pulse_width = 0

# come here everytime sth changes in the server 
def server_callback(config):
    global stim_current
    global pulse_width
    global onoff

    # assign updated server parameters to global vars 
    # refer to the server node for constraints
    onoff = config['ON_OFF']
    stim_current = config['Current']
    pulse_width = config['Pulse_Width']

def main():
    global stimMsg
    global stim_current
    global pulse_width
    global onoff

    # init matrix node
    rospy.init_node('matrix', anonymous=False)

    # communicate with the dynamic server
    dyn_params = reconfig.Client('server', config_callback = server_callback)

    # get node config
    config_dict = rospy.get_param('/ema/matrix')

    # list published topics
    pub = {}
    pub['singlepulse'] = rospy.Publisher('stimulator/single_pulse', Stimulator, queue_size=10)
    pub['signal'] = rospy.Publisher('matrix/stimsignal', Int8, queue_size=10)
    pub['channels'] = rospy.Publisher('matrix/channel_vec', Int8MultiArray, queue_size=10)

    # initialize stimulation parameters
    StimChannels = config_dict['channels']
    StimMode = 'single'
    StimFreq = config_dict['freq']

    # each electrode's signal - visualization
    channel_vec = Int8MultiArray()
    channel_vec.data = [0]*(len(StimChannels)+1) # first element not used

    # build basic stimulator message
    stimMsg = Stimulator()
    
    # define loop rate (in hz)
    stim_rate = rospy.Rate(StimFreq)

    # node loop
    while not rospy.is_shutdown():
            for n, channel in enumerate(StimChannels):

                if onoff: # stimulation switch                    
                    stimMsg.channel = [channel]
                    stimMsg.mode = [StimMode]
                    stimMsg.pulse_width = [pulse_width]
                    stimMsg.pulse_current = [stim_current]

                    # send stimulator update
                    pub['singlepulse'].publish(stimMsg)

                    # send current signal update for visual purposes
                    pub['signal'].publish(stim_current)

                    # updates electrode signal
                    channel_vec.data[n+1] = 1 # [index] is the actual channel number

                else:
                    pub['signal'].publish(0)

                # send electrode updates - visualization
                pub['channels'].publish(channel_vec)
                channel_vec.data[n+1] = 0

                # wait for next control loop
                stim_rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

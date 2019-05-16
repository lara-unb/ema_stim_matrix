#!/usr/bin/env python

import rospy
import dynamic_reconfigure.client as reconfig

# import ros msgs
from std_msgs.msg import Int8, UInt16, Int8MultiArray
from ema_common_msgs.msg import Stimulator

# global variables
global onoff
global stim_current
global pulse_width

onoff = False
stim_current = 40
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

    start = 0 # stores start time of each repetition
    repeat_std = 12 # repeat the sequence for 2 min
    repeat = 12 
    state = 'off'; # off, wait, stim, over
    checktime = True
    progressive = 0.0 # for current ramp
    progressive_steps = 1/(StimFreq*0.5) # 0.5s for up/down ramp

    # node loop
    while not rospy.is_shutdown():

        if state is 'off':
            print state
            # interface checkbox on
            if onoff:
                # starting stimulation sequence
                state = 'wait'
                checktime = True
                repeat = repeat_std
            # send updates for visual purposes
            pub['signal'].publish(0)
            pub['channels'].publish(channel_vec) 
            # try to keep the loop in a constant frequency
            stim_rate.sleep()
            continue

        elif state is 'wait':
            print state, repeat
            if checktime:
                start = rospy.get_time()
                checktime = False
            if (rospy.get_time() - start) >= 5.0: 
                # wait is over stimulation now
                state = 'stim'
                progressive = 0.0
            if not onoff:
                # interface checkbox off
                state = 'off'
            # send updates for visual purposes
            pub['signal'].publish(0)
            pub['channels'].publish(channel_vec)
            # try to keep the loop in a constant frequency
            stim_rate.sleep()
            continue

        elif state is 'stim':
            print state
            if not onoff:
                # interface checkbox off
                state = 'off'
                # send updates for visual purposes
                pub['signal'].publish(0)
                pub['channels'].publish(channel_vec)
                # try to keep the loop in a constant frequency
                stim_rate.sleep()
                continue
            if (rospy.get_time() - start) >= 10.0:
                repeat -= 1
                if repeat <= 0:
                    # exit sequence
                    state = 'over'
                else:
                    state = 'wait'
                    checktime = True
                # send updates for visual purposes
                pub['signal'].publish(0)
                pub['channels'].publish(channel_vec)
                # try to keep the loop in a constant frequency
                stim_rate.sleep()
                continue
            for n, channel in enumerate(StimChannels):

                # up ramp from 5s to 5.5s
                if (rospy.get_time() - start) <= 5.5: 
                    progressive += progressive_steps
                # down ramp from 9.5s to 10s
                elif (rospy.get_time() - start) >= 9.5:
                    progressive -= progressive_steps
                else:
                    progressive = 1.0

                progressive = abs(progressive)

                stimMsg.channel = [channel]
                stimMsg.mode = [StimMode]
                stimMsg.pulse_width = [pulse_width]
                stimMsg.pulse_current = [int(progressive*stim_current)]
                # updates electrode signal
                channel_vec.data[n+1] = 1 # [index] is the actual channel number
                # send stimulator update
                pub['singlepulse'].publish(stimMsg)
                # send updates for visual purposes
                pub['signal'].publish(int(progressive*stim_current))
                pub['channels'].publish(channel_vec)
                # reset channel signal
                channel_vec.data[n+1] = 0
                # try to keep the loop in a constant frequency
                stim_rate.sleep()
            continue

        elif state is 'over':
            print state
            if not onoff:
                # interface checkbox off
                state = 'off'
            # send updates for visual purposes
            pub['signal'].publish(0)
            pub['channels'].publish(channel_vec)
            # try to keep the loop in a constant frequency
            stim_rate.sleep()
            continue

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

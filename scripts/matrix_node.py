#!/usr/bin/env python

import rospy
import dynamic_reconfigure.client as reconfig

# import ros msgs
from std_msgs.msg import String, Int8, UInt16, Int8MultiArray
from ema_common_msgs.msg import Stimulator

# global variables
global onoff
global stim_current
global pulse_width
global StimChannels
global channel_vec
global stim_rate
global progressive_steps

onoff = False
stim_current = 40
pulse_width = 0
StimChannels = [1,2,3,4]
channel_vec = Int8MultiArray()
channel_vec.data = [0]*(len(StimChannels)+1)
stim_rate = 0
progressive_steps = 1/(48*0.5)

# come here everytime sth changes in the server 
def server_callback(config):
    global stim_current
    global pulse_width
    global onoff
    global StimChannels
    global channel_vec
    global stim_rate
    global progressive_steps

    # assign updated server parameters to global vars 
    # refer to the server node for constraints
    onoff = config['ON_OFF']
    stim_current = config['Current']
    pulse_width = config['Pulse_Width']
    StimChannels = map(int, config['Channels'].split(','))
    channel_vec.data = [0]*(len(StimChannels)+1)
    stim_rate = rospy.Rate(config['Frequency'])
    progressive_steps = 1/(config['Frequency']*0.5)

def main():
    global stimMsg
    global stim_current
    global pulse_width
    global onoff
    global StimChannels
    global channel_vec
    global stim_rate
    global progressive_steps

    # init matrix node
    rospy.init_node('matrix', anonymous=False)

    # communicate with the dynamic server
    dyn_params = reconfig.Client('server', config_callback = server_callback)

    # list published topics
    pub = {}
    pub['singlepulse'] = rospy.Publisher('stimulator/single_pulse', Stimulator, queue_size=10)
    pub['signal'] = rospy.Publisher('matrix/stimsignal', Int8, queue_size=10)
    pub['channels'] = rospy.Publisher('matrix/channel_vec', Int8MultiArray, queue_size=10)
    pub['state'] = rospy.Publisher('matrix/state', String, queue_size=10)

    # build basic stimulator message
    stimMsg = Stimulator()

    # define initial loop rate (in hz)
    stim_rate = rospy.Rate(48)

    # initialize loop variables
    start = 0 # stores start time of each repetition
    repeat_max = 12 # repeat the sequence for 2 min
    repeat_now = 12 # current repetition number
    state = 'off'; # off, wait, stim, over
    checktime = True # Is a new repetition starting now?
    progressive = 0.0 # for current ramp

    # node loop
    while not rospy.is_shutdown():

        if state is 'off':
            # interface checkbox on
            if onoff:
                # starting stimulation sequence
                state = 'wait'
                checktime = True
                repeat_now = repeat_max
            # send updates for visual purposes
            pub['signal'].publish(0)
            pub['channels'].publish(channel_vec) 
            pub['state'].publish(state)
            # try to keep the loop in a constant frequency
            stim_rate.sleep()
            continue

        elif state is 'wait':
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
            pub['state'].publish(state+' | '+str(repeat_now)+' mtg')
            # try to keep the loop in a constant frequency
            stim_rate.sleep()
            continue

        elif state is 'stim':
            if not onoff:
                # interface checkbox off
                state = 'off'
                # send updates for visual purposes
                pub['signal'].publish(0)
                pub['channels'].publish(channel_vec)
                pub['state'].publish(state+' | '+str(repeat_now)+' mtg')
                # try to keep the loop in a constant frequency
                stim_rate.sleep()
                continue
            if (rospy.get_time() - start) >= 10.0:
                repeat_now -= 1
                if repeat_now <= 0:
                    # exit sequence
                    state = 'over'
                else:
                    state = 'wait'
                    checktime = True
                # send updates for visual purposes
                pub['signal'].publish(0)
                pub['channels'].publish(channel_vec)
                pub['state'].publish(state+' | '+str(repeat_now)+' mtg')
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
                stimMsg.mode = ['single']
                stimMsg.pulse_width = [pulse_width]
                stimMsg.pulse_current = [int(progressive*stim_current)]
                # updates electrode signal
                channel_vec.data[n+1] = 1 # [index] is the actual channel number
                # send stimulator update
                pub['singlepulse'].publish(stimMsg)
                # send updates for visual purposes
                pub['signal'].publish(int(progressive*stim_current))
                pub['channels'].publish(channel_vec)
                pub['state'].publish(state+' | '+str(repeat_now)+' mtg')
                # reset channel signal
                channel_vec.data[n+1] = 0
                # try to keep the loop in a constant frequency
                stim_rate.sleep()
            continue

        elif state is 'over':
            if not onoff:
                # interface checkbox off
                state = 'off'
            # send updates for visual purposes
            pub['signal'].publish(0)
            pub['channels'].publish(channel_vec)
            pub['state'].publish('S: '+state)
            # try to keep the loop in a constant frequency
            stim_rate.sleep()
            continue

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

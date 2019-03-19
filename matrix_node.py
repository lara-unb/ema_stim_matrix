#!/usr/bin/env python

import rospy
import ema.modules.control as control
import dynamic_reconfigure.client as reconfig

# import ros msgs
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import Int8
from std_msgs.msg import Int32MultiArray
from ema_common_msgs.msg import Stimulator

# import utilities
from math import pi
from tf import transformations

# global variables
global on_off
global angle
global speed
global speed_ref
global speed_err
global time
global pw_left
global pw_right

global left_current
global right_current
global update_values

left_current = 0
right_current = 0
update_values = False

on_off = False
angle = [0,0]
speed = [0,0]
speed_ref = 300
speed_err = [0,0]
time = [0,0]
pw_left = [0,0]
pw_right = [0,0]

def server_callback(config):
    global left_current
    global right_current
    global update_values
    update_values = True

    # print('Server callback', config['current_left'], config['current_right'], left_current, right_current)

    if config['current_left'] - left_current > 2:
        left_current += 2
    else:
        left_current = config['current_left']


    if config['current_right'] - right_current > 2:
        right_current += 2
    else:
        right_current = config['current_right']

    # print('End server callback', config['current_left'], config['current_right'], left_current, right_current)

def pedal_callback(data):
    # get timestamp
    time.append(data.header.stamp)

    # get angle position
    qx,qy,qz,qw = data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w
    euler = transformations.euler_from_quaternion([qx, qy, qz, qw], axes='rzyx')

    x = euler[2]
    y = euler[1]

    # correct issues with more than one axis rotating
    if y >= 0:
        y = (y/pi) * 180
        if abs(x) > (pi*0.5):
            y = 180-y            
    else:
        y = (y/pi) * 180
        if abs(x) > (pi*0.5):
            y = 180 - y
        else:
            y = 360 + y

    angle.append(y)

    # get angular speed
    speed.append(data.angular_velocity.y*(180/pi))

    # get error
    speed_err.append(speed_ref - speed[-1])

    # print latest
    # print time[-1], angle[-1], speed[-1], speed_err[-1]

def remote_callback(data):
    global stimMsg
    
    global on_off
    
    if data == Int8(1):
        if on_off == False:
            on_off = True
        
        stimMsg.pulse_current[0] += 1
        stimMsg.pulse_current[1] += 1
        rospy.loginfo("Stimulator current is now %d", stimMsg.pulse_current[0])
    elif data == Int8(2):
        if on_off == True:
            stimMsg.pulse_current[0] -= 1
            stimMsg.pulse_current[1] -= 1
        
        if stimMsg.pulse_current[0] < 6:
            on_off = False
            rospy.loginfo("Turned off controller")
        else:
            rospy.loginfo("Stimulator current is now %d", stimMsg.pulse_current[0])
    elif data == Int8(3):
        on_off = False
        stimMsg.pulse_current[0] = 5
        stimMsg.pulse_current[1] = 5
        rospy.loginfo("Turned off controller")

def main():
    global stimMsg
    global update_values

    # init matrix node
    rospy.init_node('matrix', anonymous=False)

    # communicate with the dynamic server
    dyn_params = reconfig.Client('server', config_callback = server_callback)

    # get node config
    config_dict = rospy.get_param('/ema/matrix')

    StimChannels = config_dict['channels']
    StimMode = config_dict['mode']
    StimPulseWidth = config_dict['pulse_width']
    StimPulseCurrent = 0
    StimFreq = config_dict['freq']

    # build basic stimulator message
    stimMsg = Stimulator()

    # build basic stimulator signal message for plotting purposes
    signalMsg = Int32MultiArray()
    signalMsg.data = []
    
    # list published topics
    pub = {}
    # pub['cclpulses'] = rospy.Publisher('stimulator/ccl_update', Stimulator, queue_size=10)
    pub['singlepulse'] = rospy.Publisher('stimulator/single_pulse', Stimulator, queue_size=10)
    pub['signal'] = rospy.Publisher('control/stimsignal', Int32MultiArray, queue_size=10)
    
    # define loop rate (in hz)
    rate = rospy.Rate(StimFreq)

    # node loop
    while not rospy.is_shutdown():
        for n, channel in enumerate(StimChannels):
            # parameters update
            if update_values is True:
                params = { 'current_left' : left_current, 'current_right' : right_current }
                dyn_params.update_configuration(params)
                update_values = False

            stimMsg.pulse_current = [progressive[0]*left_current, progressive[1]*right_current]
            # print(stimMsg.pulse_current)

            stimMsg.channel = channel
            stimMsg.mode = StimMode[n]
            stimMsg.pulse_width = StimPulseWidth[n]
            stimMsg.pulse_current = StimPulseCurrent

            # send stimulator update
            pub['singlepulse'].publish(stimMsg)
            
            # send signal update
            signalMsg.data = [progressive[0]*left_current, progressive[1]*right_current]
            pub['signal'].publish(signalMsg)

            # wait for next control loop
            rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

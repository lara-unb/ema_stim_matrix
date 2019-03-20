#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from ema_stim_matrix.cfg import DynamicParamsConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request:\n\
Current: 		{current}    \n\
Pulse Width: 	{pulse_width}\n""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node('server', anonymous=False)

    srv = Server(DynamicParamsConfig, callback)
    rospy.spin()

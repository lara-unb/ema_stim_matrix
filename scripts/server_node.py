#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from ema_stim_matrix.cfg import DynamicParamsConfig

global prev_config

# used to compare past values
prev_config = {'Current': 0, 'Pulse_Width': 500}

def callback(config, level):
    global prev_config

    # prevents the user from abruptly increasing the current
    if config['Current'] - prev_config['Current'] > 2:
        config['Current'] = prev_config['Current'] + 2

    prev_config['Current'] = config['Current']

    return config

if __name__ == "__main__":
    rospy.init_node('server', anonymous=False)

    srv = Server(DynamicParamsConfig, callback)
    rospy.spin()

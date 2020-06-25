#!/usr/bin/env python

"""

Particularly, this code establishes the dynamic reconfiguration server: it
watches and defines rules for dynamic parameter changes.

The ROS node runs this code. It should make all the necessary
communication/interaction with ROS and it shouldn't deal with minor details.
For example, it would be used to publish a filtered sensor measurement as
a ROS message to other ROS nodes instead of stablishing the serial comm
and treating that raw measurement. For more information, check:
http://wiki.ros.org/dynamic_reconfigure/Tutorials/SettingUpDynamicReconfigureForANode%28python%29

"""

import rospy
import dynamic_reconfigure.server
from ema_stim_matrix.cfg import MatrixServerConfig  # pkgname.cfg, cfgfilenameConfig

# global variables
global prev_config

# used to compare past values
prev_config = {'Current': 40, 'Pulse_Width': 0}


# defines the rules for parameter changes
def callback(config, level):
    global prev_config

    # prevents the user from abruptly increasing the current
    if config['Current'] - prev_config['Current'] > 2:
        config['Current'] = prev_config['Current'] + 2

    prev_config['Current'] = config['Current']

    return config


def main():
    # init dynamic reconfigure server node
    rospy.init_node('reconfig')
    
    srv = dynamic_reconfigure.server.Server(
            MatrixServerConfig, callback)  # cfgfilenameConfig, callback name
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

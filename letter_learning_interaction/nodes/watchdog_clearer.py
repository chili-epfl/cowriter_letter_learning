#!/usr/bin/env python2
'''Publish watchdog clears on the /watchdog_clear/device_name topic every
time_between_clears seconds.
'''

#@TODO possibly better to just use rospy.sleep instead of the watchdog clearer class

import rospy
from letter_learning_interaction.watchdog import WatchdogClearer

if __name__ == "__main__":
    #parse arguments
    import argparse
    parser = argparse.ArgumentParser(description='Publish watchdog clears \
    on the /watchdog_clear/device_name topic.');
    parser.add_argument('device_name', action="store", type=str,
                    help='name of device which the watchdog is monitoring');
    parser.add_argument('time_between_clears', action="store", type=float,
                    help='name of device which the watchdog is monitoring');
    args = parser.parse_args();

    rospy.init_node(args.device_name + '_watchdog_clearer');

    wdc=WatchdogClearer('watchdog_clear/'+args.device_name, args.time_between_clears);
    rospy.loginfo('Starting new watchdog_clearer for '+args.device_name);


    while not rospy.is_shutdown():
        pass

    wdc.stop();

#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from time import time
import signal
import sys

linear_vel = 0.1
angular_vel = 0.2
thr = 1

class Robot():

    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.scan = rospy.wait_for_message('scan', LaserScan)
        self.rate = rospy.Rate(10)
        self.print_scan()
        self.avoid()

    def print_scan(self):
        print('==============')
        print('Left')
        print(self.scan.ranges[90])
        print('Front')
        print(self.scan.ranges[0])
        print('Right')
        print(self.scan.ranges[270])

    def avoid(self):
        move = Twist()

        if self.scan.ranges[0] and self.scan.ranges[15] and self.scan.ranges[345] < thr:
            move.linear.x = 0.0
            move.angular.z = angular_vel
            rospy.loginfo('Stop!')
        else:
            move.linear.x = linear_vel
            move.angular.z = 0.0
        
        self.pub.publish(move)
        self.rate.sleep()

    def shutdownhook(self, sig, frame):
        move = Twist()
        self.pub.publish(move)
        self.rate.sleep()
        rospy.signal_shutdown('shutdown')
        print('Shutdown Robot!')

def main():
    rospy.init_node('turtlebot3_obstacle', disable_signals = True)

    try:
        while not rospy.is_shutdown():
            robot = Robot()
            signal.signal(signal.SIGINT, robot.shutdownhook)
            signal.signal(signal.SIGTERM, robot.shutdownhook)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

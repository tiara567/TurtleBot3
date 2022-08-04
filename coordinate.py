#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2
import numpy as np
import signal
import sys

linear_vel = 0.1
angular_vel = 0.1
target = (0,0)

class Robot():

    def __init__(self):
        odom = rospy.wait_for_message('odom', Odometry)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        self.rate = rospy.Rate(1)

        self.x = odom.pose.pose.position.x
        self.y = odom.pose.pose.position.y

        self.rot = odom.pose.pose.orientation
        (self.roll, self.pitch, self.theta) = euler_from_quaternion([self.rot.x, self.rot.y, self.rot.z, self.rot.w])

        self.message()
        self.movement()

    def message(self):
        print('Coordinate, Angle')
        print([self.x , self.y, self.theta])

    def movement(self):
        move = Twist()
        goal = Point()
        goal.x = target[0]
        goal.y = target[1]

        inc_x = goal.x - self.x
        inc_y = goal.y - self.y
        distance_to_goal = np.sqrt(inc_x * inc_x + inc_y * inc_y)

        final_goal = atan2(inc_y, inc_x)
        angle_to_goal = final_goal - self.theta

        print('Distance to Goal')
        print(distance_to_goal)
        print('Angle to Goal')
        print(abs(angle_to_goal))

        if abs(angle_to_goal) > 0.1:
            move.angular.z = angular_vel
            move.linear.x = 0.0
            self.pub.publish(move)
        else:
            move.angular.z = 0.0
            self.pub.publish(move)
            if distance_to_goal >= 0.05:
                move.linear.x = linear_vel
            else:
                move.linear.x = 0.0
                print('Goal Reached!')

            self.pub.publish(move)

        self.rate.sleep()

    def shutdownhook(self, sig, frame):
        move = Twist()
        self.pub.publish(move)
        self.rate.sleep()
        rospy.signal_shutdown('Shutdown')
        print('Shutdown Robot!')

def main():
    rospy.init_node('go_to_coordinate')

    try:
        while not rospy.is_shutdown():
            robot = Robot()
            signal.signal(signal.SIGINT, robot.shutdownhook)
            signal.signal(signal.SIGTERM, robot.shutdownhook)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

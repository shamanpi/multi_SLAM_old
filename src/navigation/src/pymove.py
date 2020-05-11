#!/usr/bin/env python

import sys
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Grabs the velocity and turning speed based upon laser measurements
def get_move(measurements, increment):
    d = 0.4
    num_meas = len(measurements)
    fleft = measurements[int(1.0 / 16.0 * num_meas):int(3.0 / 16.0 * num_meas)]
    front = measurements[int(-1.0 / 16.0 * num_meas):] + measurements[:int(1.0 / 16.0 * num_meas)]
    fright = measurements[int(13.0 / 16.0 * num_meas):int(15.0 / 16.0 * num_meas)]
    print('lens: ', len(fleft), len(front), len(fright))
    print('means: ', np.mean(fleft), np.mean(front), np.mean(fright))
    print('mins: ', np.min(fleft), np.min(front), np.min(fright))
    
    fleft = min(min(fleft), 10)
    front = min(min(front), 10)
    fright = min(min(fright), 10)

    # find wall, move forward and right
    if (front > d and fleft > d and fright > d) or (front > d and fleft < d and fright > d) or (front > d and fleft < d and fright < d):
        return 0.2, -0.4
    # follow wall, move forward
    elif front > d and fleft > d and fright < d:
        return 0.4, 0.0
    # turn left, move left
    else:
        return 0.0, 0.3

class Mover(object):
    def __init__(self, namespace):
        self.ns = namespace
        self.vel = 0
        self.turn = 0
        self.last_move = 0

        rospy.init_node('mover')
        self.pub = rospy.Publisher('%scmd_vel' % rospy.get_namespace(), Twist, queue_size=10)
        self.sub = 0

    def update_value(self, data):
        self.vel, self.turn = get_move(data.ranges, data.angle_increment)
        msg = Twist()
        msg.linear.x = self.vel
        msg.angular.z = self.turn
        self.pub.publish(msg)
        # print("Updated: ", self.vel, self.turn, self.mode, msg)

    def run(self):
        rospy.sleep(0.1)
        self.sub = rospy.Subscriber('%sscan' % rospy.get_namespace(), LaserScan, self.update_value)
        rospy.spin()


def main(argv):
    if len(argv) > 2:
        ns = argv[1]
    else:
        ns = '/tb3_0'
    movers = []
    m = Mover(ns)
    m.run()


if __name__ == '__main__':
    main(sys.argv)

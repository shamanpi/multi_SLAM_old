import sys
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

SCAN = 1
EXPLORE = 2
FOLLOW = 3
DOORWAY = 4

# Based upon objects in front of robot
def get_vel(measurements, increment):
    # Original measurements in front of the robot
    forward_meas = np.append(measurements[int(7.0 / 8.0 * len(measurements)):],
                             measurements[:int(1.0 / 8.0 * len(measurements))])
    # Correct measurements with cosine to get distance in front of sensor
    start = 7.0 / 4.0 * np.pi
    cos = []
    for i in range(len(forward_meas)):
        cos.append(np.cos( i * increment + start))
    cos = np.array(cos)
    forward_dists = forward_meas / cos

    num_nan = (sum(np.isnan(forward_dists)) + sum(np.isinf(forward_dists))) / len(forward_dists)

    # If too many nan reads or too far away, move at full speed. Scale negatively by distance
    vel = 0.1
    avg_dist = np.nanmean(forward_dists[np.invert(np.isinf(forward_dists))])
    avg_dist2 = np.mean(forward_dists)
    # print("[Vel] Avg: ", avg_dist, avg_dist2, np.mean(forward_meas), ", num nan: ", num_nan, 'len: ', len(forward_dists))
    if num_nan < 0.5 and avg_dist < 3.0:
        vel *= avg_dist - 0.5
        # print("VELING: ", vel)

    return vel


# Based upon wall / objects to left of robot
def get_turn(measurements, increment):
    # Original measurements to left of the robot
    left_meas = measurements[int(1.0 / 8.0 * len(measurements)):int(3.0 / 8.0 * len(measurements))]

    # Correct measurements with sine to get distance to left of sensor
    start = 1.0 / 4.0 * np.pi
    sin = np.array([np.sin(i * increment + start) for i in range(len(left_meas))])
    left_dists = left_meas / sin

    try:
        num_nan = (sum(np.isnan(left_dists)) + sum(np.isinf(left_dists))) / len(left_dists)
    except:
        num_nan = 0

    # If too many nan reads or too far away, turn at full speed. Scale negatively by distance
    turn = 0.2
    avg_dist = np.nanmean(left_dists[np.invert(np.isinf(left_dists))])
    if num_nan < 0.5 and avg_dist < 1.0:
        # Try to stay close to the left-hand wall
        turn *= (avg_dist - 0.5)
    else:
        turn = -1.0

    return turn


# Grabs the velocity and turning speed based upon laser measurements
def get_move(measurements, increment):
    d = 0.5
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
        return 0.2, -0.3
    # follow wall, move forward
    elif front > d and fleft < d and fright < d:
        return 0.5, 0.0
    # turn left, move left
    else:
        return 0.0, 0.3

class Mover(object):
    def __init__(self, namespace):
        self.ns = namespace
        self.vel = 0
        self.turn = 0
        self.last_move = 0
        self.mode = EXPLORE

        rospy.init_node('mover')
        self.pub = rospy.Publisher('%s/cmd_vel' % self.ns, Twist, queue_size=10)
        self.sub = 0

    def update_value(self, data):
        self.vel, self.turn = get_move(data.ranges, data.angle_increment)
        # new_turn = get_turn(data.ranges, data.angle_increment)
        # new_vel = get_vel(data.ranges, data.angle_increment)

        # if self.mode == EXPLORE:
        #     self.turn = 0.0
        #     self.vel = new_vel

        #     # If a wall is found nearby, then change to follower mode
        #     if new_turn > -1.0:
        #         self.mode = FOLLOW

        #     # If a wall in front is found, then start scanning
        #     elif new_vel < 0.05:
        #         self.last_move = data.header.stamp.secs
        #         self.mode = SCAN

        # elif self.mode == FOLLOW:
        #     self.vel = new_vel
        #     self.turn = new_turn
            
        #     # If the wall is lost, then start scanning
        #     if new_turn == -1.0:
        #         self.last_move = data.header.stamp.secs
        #         self.mode = SCAN

        #     # If a wall is approached too closely
        #     if new_vel < 0.05:
        #         self.last_move = data.header.stamp.secs
        #         self.mode = SCAN

        # elif self.mode == SCAN:
        #     self.vel = 0
        #     self.turn = new_turn

        #     # If a wall is found after turning minimal distance, start following
        #     if new_turn > -1.0 and data.header.stamp.secs - self.last_move > 1.0:
        #         self.mode = FOLLOW

        #     # If enough time spent scanning, give up and start exploring
        #     elif data.header.stamp.secs - self.last_move > 3.0:
        #         self.mode = EXPLORE

        msg = Twist()
        msg.linear.x = self.vel
        msg.angular.z = self.turn
        self.pub.publish(msg)
        m = ['', 'SCAN', 'EXPLORE', 'FOLLOW'][self.mode]
        print("MODE: %s, %3f, %3f" % (m, self.vel, self.turn))
        # print("Updated: ", self.vel, self.turn, self.mode, msg)

    def run(self):
        rospy.sleep(0.1)
        self.sub = rospy.Subscriber('%s/scan' % self.ns, LaserScan, self.update_value)
        rospy.spin()


def main(argv):
    namespace = '/tb3_0'
    if len(argv) >= 2:
        namespace = argv[1]
    m = Mover(namespace)
    m.run()


if __name__ == '__main__':
    main(sys.argv)

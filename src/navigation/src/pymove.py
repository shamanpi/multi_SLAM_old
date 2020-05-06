import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

SCAN = 1
EXPLORE = 2
FOLLOW = 3

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
    vel = 0.2
    avg_dist = np.nanmean(forward_dists[np.invert(np.isinf(forward_dists))])
    avg_dist2 = np.mean(forward_dists)
    print("[Vel] Avg: ", avg_dist, avg_dist2, np.mean(forward_meas), ", num nan: ", num_nan, 'len: ', len(forward_dists))
    if num_nan < 0.5 and avg_dist < 3.0:
        vel *= avg_dist - 0.5
        print("VELING: ", vel)

    return vel


# Based upon wall / objects to left of robot
def get_turn(measurements, increment):
    # Original measurements in front of the robot
    left_meas = measurements[int(1.0 / 8.0 * len(measurements)):int(3.0 / 8.0 * len(measurements))]

    # Correct measurements with cosine to get distance in front of sensor
    start = 1.0 / 8.0 * np.pi
    sin = np.array([np.sin(i * increment + start) for i in range(len(left_meas))])
    left_dists = left_meas / sin

    try:
        num_nan = (sum(np.isnan(left_dists)) + sum(np.isinf(left_dists))) / len(left_dists)
    except:
        num_nan = 0

    # If too many nan reads or too far away, turn at full speed. Scale negatively by distance
    turn = -0.2
    avg_dist = np.nanmean(left_dists[np.invert(np.isinf(left_dists))])
    print("[Turn] Avg: ", avg_dist, ", num nan: ", num_nan, 'len: ', len(left_dists))
    if num_nan < 0.5 and avg_dist < 1.0:
        # Try to stay close to the left-hand wall
        turn *= (avg_dist - 0.5)
        print("TURNING: ", turn)
    else:
        turn = 1.0

    return turn


class Mover(object):
    def __init__(self):
        self.vel = 0
        self.turn = 0
        self.last_move = 0
        self.mode = SCAN

        rospy.init_node('mover')
        self.pub = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10)
        self.sub = 0

    def update_value(self, data):
        # new_turn = get_turn(data.ranges, data.angle_increment)
        #
        # # If a wall is found, update the movement
        # if new_turn != 1.0:
        #     self.turn = new_turn
        #     self.last_move = data.header.stamp.secs
        # # If no wall found for more than 2 seconds, then move forward anyways
        # elif data.header.stamp.secs - self.last_move > 2.0:
        #     self.turn = 0
        # # If no wall found but for only a short time, then continue turning
        # else:
        #     self.turn = new_turn
        # print('new turn: ', new_turn, self.turn, self.last_move, data.header.stamp.secs)
        #
        # new_vel = 0.0
        # if self.turn < 1.0:
        #     new_vel = get_vel(data.ranges, data.angle_increment)
        # self.vel = new_vel
        new_turn = get_turn(data.ranges, data.angle_increment)
        new_vel = get_vel(data.ranges, data.angle_increment)

        if self.mode == EXPLORE:
            self.turn = 0.0
            self.vel = new_vel
            if new_turn < 1.0:
                self.mode = FOLLOW
            elif new_vel < 0.01:
                self.last_move = data.header.stamp.secs
                self.mode = SCAN

        elif self.mode == FOLLOW:
            self.vel = new_vel
            self.turn = new_turn
            if new_turn == 1.0:
                self.last_move = data.header.stamp.secs
                self.mode = SCAN

        elif self.mode == SCAN:
            self.vel = 0
            self.turn = new_turn
            if new_turn < 1.0:
                self.mode = FOLLOW
            elif data.header.stamp.secs - self.last_move > 3.0:
                self.mode = EXPLORE

        msg = Twist()
        msg.linear.x = self.vel
        msg.angular.z = self.turn
        self.pub.publish(msg)
        print("Updated: ", self.vel, self.turn, self.mode, msg)

    def run(self):
        rospy.sleep(0.1)
        self.sub = rospy.Subscriber('/tb3_0/scan', LaserScan, self.update_value)
        rospy.spin()


#        while not rospy.is_shutdown():
#            rospy.sleep(0.1)
#            msg = Twist()
#            msg.linear.x = self.vel
#            msg.angular.z = self.turn
#
#            self.pub.publish(msg)
#            # r.sleep()
#
def main():
    m = Mover()
    m.run()


if __name__ == '__main__':
    main()

#! /usr/bin/env python
# Ashwad Pandit

import rospy
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import sys

from feature_extraction.msg import corners_and_lines

def individualMapMaker(array_robot, figure):
	plt.clf()
	plt.figure(figure)
	
	print(len(array_robot))
	for x in my_range(3, len(array_robot), 4):
		print([array_robot[x-3], array_robot[x-1]], [array_robot[x-2], array_robot[x]])
		plt.plot([array_robot[x-3], array_robot[x-1]], [array_robot[x-2], array_robot[x]])


def my_range(start, end, step):
    while start <= end:
        yield start
        start += step

def callback(msg):
	array_robot = msg.endPoints

	plt.ion()
	plt.show()

	individualMapMaker(array_robot, 1)
	
	plt.draw()
	plt.pause(2)

	

if __name__ == '__main__':
    	rospy.init_node('visualizer', anonymous=True)
	
	rospy.Subscriber("/tb3_0/features", corners_and_lines, callback)

	rospy.spin()

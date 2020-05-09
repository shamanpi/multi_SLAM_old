#! /usr/bin/env python
# Alec Werning / werni018@umn.edu
# Two Robot Mapping Project - CSCI 5552

import rospy
from sensor_msgs.msg import LaserScan
from feature_extraction.msg import corners_and_lines


import numpy as np
import matplotlib.pyplot as plt
import sys


# plot laserscan data
def laser_scan_visualized(laserData, theta):
    plt.clf()
    ax = plt.figure(1)
    ax = plt.subplot(111, projection='polar')
    ax.plot(theta, laserData)
    ax.plot(theta, laserData)
    plt.title('Laser Scan Data (distance (meters))')
    #plt.pause(0.0000000001)

# shuffle indices and return to random points and remaining points
def sample_two_points(unmatchedCoordinates):

    point1 = np.array([np.inf, np.inf])
    while np.isinf(point1[0]) or np.isinf(point2[0]):
        index = np.arange(unmatchedCoordinates[1,:].shape[0])
        np.random.shuffle(index)
        selectedPoints = index[:2]
        remainingPoints = index[2:]
        twoPoints = unmatchedCoordinates[:,selectedPoints]
        point1 = twoPoints[:,0]
        point1.shape = (3,1)
        point2 = twoPoints[:,1]
        point2.shape = (3,1)
        testPoints = unmatchedCoordinates[:,remainingPoints]
        shuffledIndex = index
    return point1, point2, testPoints, shuffledIndex

# line definition from two points
def define_line(point1,point2):
    m = (point1[1]-point2[1])/(point1[0]-point2[0]) # delta y / delta x
    b = point1[1]-m*point1[0] # y0 - m*x0
    return m[0], b

# perpendicular intercept of a point to a lines
def find_normal_point(m, b, x, y):
    x_normal = (x + m*y - m*b)/(1 + m**2)
    y_normal = (m*x + (m**2)*y - (m**2)*b)/(1 + m**2) + b
    return x_normal, y_normal

# feature extraction - find end points of lines and corner points from LIDAR scan data
def feature_extraction(msg):

    # define function variables from laser data
    msg2 = np.array(msg.ranges) # change msg to msg.ranges
    range_min =  msg.range_min #0.119999997318 # change to msg.range_min
    range_max =  msg.range_max #3.5 # change to msg.range_max
    pointIndex = np.arange(msg2.size) # index for each point

    # draw laser scan data
    theta = np.arange(+np.pi/2, 2*np.pi+0.0175019223243+np.pi/2, 0.0175019223243)
    #laser_scan_visualized(msg2, theta)

    # rotate theta by 45 degrees
    theta = theta+np.pi/4

    # remove all finite points
    theta = theta[msg2 != np.inf]
    pointIndex = pointIndex[msg2 != np.inf]
    msg2 = msg2[msg2 != np.inf]

    # convert from radial to cartesian coordinate system
    x = np.multiply(msg2, np.cos(theta))
    y = np.multiply(msg2, np.sin(theta))
    coordinates = np.array([x,y]) # 2x360

    # plot cartesian coordiates to verify
    # c_ax = plt.figure(3)
    # plt.scatter(x,y)
    # plt.pause(0.0000000001)

    # RANSAC - find line segments
    ransacIterations = 100 # number of random lines to search along
    distanceThreshold = .03 # maximum distance for a point belong to a line
    linePointsThreshold = 30 # number of points to define a line
    #np.random.seed(2) # seed number generator
    newLines = []
    mNew = []
    endPoints = np.array([[],[],[]])
    unmatchedCoordinates = coordinates

    # add indexes
    unmatchedCoordinates = np.vstack([unmatchedCoordinates,pointIndex])

    for iLoop in range(ransacIterations):

        # check that we have enough points to define a line
        if unmatchedCoordinates[0,:].size < linePointsThreshold:
            break

        # select 2 random points that aren't infinite value
        point1, point2, testPoints, shuffledIndex = sample_two_points(unmatchedCoordinates)
        #print 'new line defined'

        # check that number of points left is enough to establish a line
        if testPoints[0,:].size < linePointsThreshold:
            break

        # define line
        m, b = define_line(point1,point2)
        # add points from line to matched points array
        matchedLine = np.append(point1, point2, axis=1)

        # plot line from two points and test points
        # plt.figure(2).clf()
        # drawLine = plt.figure(2)t
        # plt.scatter(testPoints[0,:],testPoints[1,:],label='unmatched points')
        # plt.plot(matchedLine[0,0:2],matchedLine[1,0:2],marker='x',color='r',label='two selected points')
        # plt.xlim(-3.6,3.6)
        # plt.ylim(-3.6,3.6)

        # reset matched point variables
        c=0 # points matched counter
        testPointcounter = 0 # keep track of number of points tested
        testPointIndex = [] # keep track of indices matched
        xNew = [] # new matched x points
        ynew = [] # new matched y points
        for iPoint in testPoints.transpose():
            iPoint.shape = (3,1) # reshape into column vector
            if not np.isinf(iPoint[0]):
                # find distance from iPoint to line
                d = np.abs(m*iPoint[0]-1*iPoint[1]+b) / np.sqrt(np.square(m)+np.square(1))

                # check if it is less than the threshold
                if d < distanceThreshold:
                    c = c+1
                    matchedLine = np.hstack((matchedLine,iPoint))
                    #print 'point matched to line'
                    testPointIndex = np.append(testPointIndex,testPointcounter) # mathed point! add it's index

                if c > linePointsThreshold:
                    yNew = matchedLine[1,:] # all the elements in row 2
                    xNew = matchedLine[0,:] # all the elements in row 1
                    A = np.vstack([xNew,np.ones(len(xNew))]).T # least squares estimate of the line
                    mNew,bNew = np.linalg.lstsq(A,yNew)[0]
                    m = mNew
                    b = bNew

                # keep track of which point is matched
                testPointcounter = testPointcounter +1

        # end of four loop through unmatched points

        # after looking at all points - define new line if we matched
        if c > linePointsThreshold:

            # save new line info
            newLines = np.append(newLines, [mNew,bNew])

            # find endpoints of the lines
            minLineDistance = 0
            maxLineDistance = 0
            xP =  matchedLine[0,0]
            yP = mNew*xP + bNew # point on line

            # find distance to point on line
            for iNorm in range(len(matchedLine[0,:])):
                x0 = matchedLine[0,iNorm]
                y0 = matchedLine[1,iNorm]
                x_norm, y_norm = find_normal_point(mNew, bNew, x0, y0) # point on line normal to matched point
                lineDistance = np.sign(xP-x0)*np.sqrt((xP-x0)**2 + (yP-y0)**2)
                if lineDistance < minLineDistance:
                    minLineDistance = lineDistance
                    endPoint1 = np.array([[x0],[y0],[matchedLine[2,iNorm]]])

                elif lineDistance > maxLineDistance:
                    maxLineDistance = lineDistance
                    endPoint2 = np.array([[x0],[y0],[matchedLine[2,iNorm]]])

            # define endpoint if we measured distance from the end point
            if not endPoint1[0]:
                endPoint1 = np.array([[matchedLine[0,0]],[matchedLine[1,0]],[matchedLine[2,0]]])
            if not endPoint2[0]:
                endPoint2 = np.array([[matchedLine[0,0]],[matchedLine[1,0]],[matchedLine[2,0]]])

            # add new end points of line to array
            endPoints = np.hstack([endPoints, np.hstack([endPoint1,endPoint2])])

            # remove matched points from unmatchedCoordinates
            testPointIndex = np.sort(testPointIndex)[::-1]
            for donePoint in testPointIndex:
                testPoints = np.delete(testPoints,donePoint,1)

            #reassign unmatched points
            unmatchedCoordinates = testPoints

            # plot matched points and extimated line
            # plt.scatter(xNew,yNew,label='matched points')
            # xCheck = range(-3,4,1)
            # yCheck = np.dot(xCheck,m)+b
            # plt.plot(xCheck,yCheck,color='y',label='estimated line')
            # plt.scatter(endPoints[0,-2:],endPoints[1,-2:],color='g',label='end points')

        # plt.legend()
        # plt.title('line segment identification - RANSAC iteration = ' + str(iLoop))
        #plt.show() # show each ransac iteration progress

    # corner identification
    cornerPoint = np.array([[],[]])
    possibleIntersection = np.array([[],[]])
    # look for outside points
    for iPoint in endPoints.T:
        laserIndex = int(iPoint[2])
        distanceMsmt = msg.ranges[laserIndex]

        # ensure you check correct point cw to index
        if laserIndex == 0:
            laserIndexCW = len(msg.ranges)-1
        else:
            laserIndexCW = laserIndex-1
        distanceDifferenceLeft = np.abs(msg.ranges[laserIndexCW]-distanceMsmt)/distanceMsmt

        # ensure you check correct point ccw to index
        if laserIndex == len(msg.ranges)-1:
            laserIndexCCW = 0
        else:
            laserIndexCCW = laserIndex+1
        distanceDifferenceRight = np.abs(msg.ranges[laserIndexCCW]-distanceMsmt)/distanceMsmt

        if distanceMsmt < range_max-.05: # only consider a corner if measurement is a bit less than max range
            specificPoint = iPoint[0:2]
            specificPoint.shape = (2,1)
            if distanceDifferenceLeft > .1 or distanceDifferenceRight > .1:
                cornerPoint = np.hstack([cornerPoint,specificPoint])
            else:
                possibleIntersection = np.hstack([possibleIntersection,specificPoint])

    # look for intersecting points
    if possibleIntersection[0,:].size > 1: # if more than one point can be an intersection
        for iNumber in range(possibleIntersection[0,:].size-1):
            relativeDistance = np.sqrt((possibleIntersection[0,iNumber]-possibleIntersection[0,iNumber+1:])**2 + (possibleIntersection[1,iNumber]-possibleIntersection[1,iNumber+1:])**2)
            intersectionDistance = np.array([[min(relativeDistance)], [np.argwhere(relativeDistance == min(relativeDistance))[0]]]) # need to find index and then average the points
            if intersectionDistance[0] < 0.1: #0.05
                averageCorner = (possibleIntersection[:,iNumber] + possibleIntersection[:,1+int(intersectionDistance[1])])/2
                averageCorner.shape = (2,1)
                cornerPoint = np.hstack([cornerPoint,averageCorner])

    # remove index from end points
    endPoints = endPoints[0:2,:]

    # rotate coordinates by to original frame - minus 45 degrees
    thetaR = -np.pi/4
    rot = np.array([[np.cos(thetaR), -np.sin(thetaR)],[np.sin(thetaR), np.cos(thetaR)]])
    coordinates = np.matmul(rot,coordinates)
    endPoints = np.matmul(rot, endPoints)
    cornerPoint = np.matmul(rot, cornerPoint)

    # plot end points, corners, and laser data
    # plt.figure(4)
    # plt.cla()
    # plt.scatter(coordinates[0,:],coordinates[1,:], color = 'b',label='laser data')
    # plt.scatter(endPoints[0,:],endPoints[1,:],color='red',marker='D',label='end points')
    # plt.scatter(cornerPoint[0,:],cornerPoint[1,:],marker='.',label='corner measurements',color='orange')
    # plt.legend()
    # plt.title('Features Extracted')
    # plt.annotate('#corners = ' + str(cornerPoint[0,:].size), xy=(0,0) )
    # plt.annotate('#lines = ' + str(endPoints[0,:].size/2), xy=(0,-.25) )
    # plt.ylabel('distance (meters)')
    # plt.xlabel('distance (meters)')
    # plt.ylim(-4,4)
    # plt.xlim(-4,4)
    # plt.pause(0.00000000000001)

    # reshape for publishing to ros topic
    cornerPointStack = []
    for iPoint in cornerPoint.T:
        cornerPointStack = np.hstack((cornerPointStack,iPoint))
    endPointStack = []
    for iPoint in endPoints.T:
        endPointStack = np.hstack((endPointStack,iPoint))

    # create array and publish as message
    msg2pub = corners_and_lines()
    msg2pub.corners = cornerPointStack
    msg2pub.endPoints = endPointStack
    pub.publish(msg2pub)
    return cornerPoint, endPoints


# main
if __name__=="__main__":
    print 'feature extraction started'
    if len(sys.argv) > 1:
        tb3_name = str(sys.argv[1])
        publish_name = tb3_name + '/features'
        subscriber_name = tb3_name + '/scan'
    else:
        publish_name = 'features'
        subscriber_name = 'scan'
    print publish_name

    rospy.init_node('featureExtraction')
    rospy.sleep(0.1)
    pub = rospy.Publisher(publish_name, corners_and_lines, queue_size=10)
    sub = rospy.Subscriber(subscriber_name, LaserScan, feature_extraction)

    # show new plot
    # plt.show()
    rospy.spin()

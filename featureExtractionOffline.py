#! /usr/bin/env python

# import rospy
# from sensor_msgs.msg import LaserScan


import numpy as np
import matplotlib.pyplot as plt


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
    msg2 = np.array(msg) # change msg to msg.ranges
    range_min =  0.119999997318 # change to msg.range_min
    range_max =  3.5 # change to msg.range_max
    pointIndex = np.arange(msg2.size) # index for each point

    # draw laser scan data
    theta = np.arange(+np.pi/2, 2*np.pi+0.0175019223243+np.pi/2, 0.0175019223243)
    laser_scan_visualized(msg2, theta)

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
    np.random.seed(2) # seed number generator
    newLines = []
    mNew = []
    endPoints = np.array([[],[],[]])
    unmatchedCoordinates = coordinates

    # add indexes
    unmatchedCoordinates = np.vstack([unmatchedCoordinates,pointIndex])

    for iLoop in range(ransacIterations):

        # select 2 random points that aren't infinite value
        point1, point2, testPoints, shuffledIndex = sample_two_points(unmatchedCoordinates)
        print 'new line defined'

        # check that number of points left is enough to establish a line
        if testPoints[0,:].size < linePointsThreshold:
            break

        # define line
        m, b = define_line(point1,point2)
        # add points from line to matched points array
        matchedLine = np.append(point1, point2, axis=1)

        # plot line from two points and test points
        plt.figure(2).clf()
        drawLine = plt.figure(2)
        plt.scatter(testPoints[0,:],testPoints[1,:],label='unmatched points')
        plt.plot(matchedLine[0,0:2],matchedLine[1,0:2],marker='x',color='r',label='two selected points')
        plt.xlim(-3.6,3.6)
        plt.ylim(-3.6,3.6)

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
                    print 'point matched to line'
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
            yP = mNew*xP + bNew # point on lines
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

            # add new end points of line to array
            endPoints = np.hstack([endPoints, np.hstack([endPoint1,endPoint2])])

            # remove matched points from unmatchedCoordinates
            testPointIndex = np.sort(testPointIndex)[::-1]
            for donePoint in testPointIndex:
                testPoints = np.delete(testPoints,donePoint,1)

            # reassign unmatched points
            unmatchedCoordinates = testPoints

            # plot matched points and extimated line
            plt.scatter(xNew,yNew,label='matched points')
            xCheck = range(-3,4,1)
            yCheck = np.dot(xCheck,m)+b
            plt.plot(xCheck,yCheck,color='y',label='estimated line')
            plt.scatter(endPoints[0,-2:],endPoints[1,-2:],color='g',label='end points')

        plt.legend()
        plt.title('line segment identification - RANSAC iteration = ' + str(iLoop))
        plt.show() # show each ransac iteration progress

    # corner identification
    cornerPoint = np.array([[],[]])
    possibleIntersection = np.array([[],[]])
    # look for outside points
    for iPoint in endPoints.T:
        laserIndex = int(iPoint[2])
        distanceMsmt = msg[laserIndex]
        distanceDifferenceLeft = np.abs(msg[laserIndex-1]-distanceMsmt)/distanceMsmt
        distanceDifferenceRight = np.abs(msg[laserIndex+1]-distanceMsmt)/distanceMsmt
        angle = np.arctan(iPoint[1]/iPoint[0])
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

    # plot end points, corners, and laser data
    plt.figure(4)
    plt.scatter(coordinates[0,:],coordinates[1,:], color = 'b',label='laser data')
    plt.scatter(endPoints[0,:],endPoints[1,:],color='red',marker='D',label='end points')
    plt.scatter(cornerPoint[0,:],cornerPoint[1,:],marker='.',label='corner measurements',color='orange')
    plt.legend()
    plt.title('Features Extracted')
    plt.annotate('#corners = ' + str(cornerPoint[0,:].size), xy=(0,0) )
    plt.annotate('#lines = ' + str(endPoints[0,:].size/2), xy=(0,-.25) )
    plt.ylabel('distance (meters)')
    plt.xlabel('distance (meters)')

    # give corner measurements and end points
    return cornerPoint, endPoints



msg = (1.8879776000976562, 1.9258660078048706, 1.9673950672149658, 2.017279624938965, 2.078166961669922, 2.154048442840576, 2.200705051422119, 2.2920331954956055, 2.347083330154419, 2.430659055709839, 2.5188345909118652, 2.5976178646087646, 2.7079782485961914, 2.8332178592681885, 2.9266908168792725, 3.0701611042022705, 3.1877646446228027, 3.346963405609131, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, 2.904428720474243, 2.9061169624328613, 2.8942697048187256, 2.907965660095215, 2.8955090045928955, 2.9027693271636963, 2.897113800048828, 2.9179840087890625, 2.9090330600738525, 2.9175591468811035, 2.928347587585449, 2.932648181915283, 2.9551548957824707, 2.9567039012908936, 2.9551167488098145, 2.9409215450286865, 2.9855682849884033, 2.9805004596710205, 2.995572328567505, 3.021671772003174, 3.04777455329895, 3.054672956466675, 3.0771334171295166, 3.0855231285095215, 3.117607831954956, 3.136622190475464, 3.1675829887390137, 3.190504312515259, 3.217266798019409, 3.236318588256836, 3.268538236618042, 3.313366413116455, 3.33571195602417, 3.371208667755127, 3.4153964519500732, 3.445957899093628, 3.485797882080078, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, 1.4544711112976074, 1.4489902257919312, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, 2.0424890518188477, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, 1.7902876138687134, np.inf, np.inf, np.inf, np.inf, np.inf, 1.052348256111145, 1.0531255006790161, 1.07045578956604, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, 2.331740617752075, 2.293544054031372, 2.286034107208252, 2.2512600421905518, 2.2599709033966064, 2.2399468421936035, 2.1969635486602783, 2.1818933486938477, 2.1852331161499023, 2.1651437282562256, 2.1444451808929443, 2.1451754570007324, 2.126969814300537, 2.1272406578063965, 2.1252236366271973, 2.0914621353149414, 2.0973405838012695, 2.08034348487854, 2.085036277770996, 2.069690465927124, 2.079922676086426, 2.082529067993164, 2.072939872741699, 2.0575008392333984, 2.038632869720459, 2.0494349002838135, 2.0558671951293945, 2.0569887161254883, 2.0498998165130615, 2.060706377029419, 2.0498366355895996, 2.0569214820861816, 2.0491950511932373, 2.0725326538085938, 2.052438974380493, 2.076225519180298, 2.073047161102295, 2.081836700439453, 2.0721583366394043, 2.1100542545318604, 2.0773420333862305, 2.111220359802246, 2.1092116832733154, 2.1431033611297607, 2.13727068901062, 2.1513214111328125, 2.149937152862549, 2.1664586067199707, 2.204853057861328, 2.2097725868225098, 2.223958969116211, 2.242704391479492, 2.2545735836029053, 2.2823023796081543, 2.303946018218994, 2.3085131645202637, 2.3276875019073486, 2.279001235961914, 2.210359573364258, 2.1363279819488525, 2.0891504287719727, 2.026122808456421, 1.9947614669799805, 1.915311336517334, 1.8762253522872925, 1.8422017097473145, 1.7947337627410889, 1.7796555757522583, 1.7286019325256348, 1.697383165359497, 1.6529837846755981, 1.6293509006500244, 1.5919581651687622, 1.5761104822158813, 1.530970573425293, 1.50296950340271, 1.487351655960083, 1.4638997316360474, 1.4497636556625366, 1.4491479396820068, 1.4005987644195557, 1.3932504653930664, 1.3495045900344849, 1.3512823581695557, 1.335412621498108, 1.3194692134857178, 1.3216756582260132, 1.3066561222076416, 1.2797361612319946, 1.2850958108901978, 1.2458139657974243, 1.2430222034454346, 1.2227447032928467, 1.2270766496658325, 1.2159366607666016, 1.1913384199142456, 1.2112513780593872, 1.204823613166809, 1.1790181398391724, 1.1843287944793701, 1.1686439514160156, 1.1618674993515015, 1.1714129447937012, 1.1440563201904297, 1.1461795568466187, 1.1283230781555176, 1.1210943460464478, 1.141791820526123, 1.1274551153182983, 1.1501128673553467, 1.116265058517456, 1.102065920829773, 1.1328977346420288, 1.1286060810089111, 1.1055803298950195, 1.1320915222167969, 1.1214343309402466, 1.1120949983596802, 1.1160130500793457, 1.131252646446228, 1.1182355880737305, 1.1298009157180786, 1.1182421445846558, 1.1280808448791504, 1.1291056871414185, 1.1292904615402222, 1.1286075115203857, 1.1228793859481812, 1.137526035308838, 1.1370238065719604, 1.1472442150115967, 1.1482223272323608, 1.1494367122650146, 1.1469541788101196, 1.155558466911316, 1.1726762056350708, 1.1773271560668945, 1.185150146484375, 1.1799033880233765, 1.2183228731155396, 1.2104032039642334, 1.2032501697540283, 1.2411274909973145, 1.246315598487854, 1.2507929801940918, 1.2626663446426392, 1.2661685943603516, 1.2775802612304688, 1.2817271947860718, 1.3136550188064575, 1.3394750356674194, 1.3328527212142944, 1.3528037071228027, 1.3631786108016968, 1.398754358291626, 1.382256031036377, 1.4250156879425049, 1.423039197921753, 1.4550939798355103, 1.485524296760559, 1.5084031820297241, 1.5264383554458618, 1.579175353050232, 1.5947794914245605, 1.6253061294555664, 1.6646183729171753, 1.6790202856063843, 1.7254154682159424, 1.7605977058410645, 1.8048889636993408, 1.8339180946350098, 1.8725167512893677)

# msg2 = (2.9043354988098145, 2.906235933303833, 2.8871164321899414, 2.9212331771850586, 2.9364078044891357, 2.906817674636841, 2.919865846633911, 2.9258344173431396, 2.9479153156280518, 2.9540932178497314, 2.959929943084717, 2.957230567932129, 2.9707937240600586, 2.9932382106781006, 3.0010101795196533, 3.020925998687744, 3.0118608474731445, 3.040576696395874, 3.05841326713562, 3.063279390335083, 3.0945277214050293, 3.125826358795166, 3.14068341255188, 3.1499264240264893, 3.1923999786376953, 3.199615478515625, 3.252094268798828, 3.262852907180786, 3.306264638900757, 3.348220109939575, 3.3618674278259277, 3.398838520050049, 3.430582284927368, 3.471605062484741, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, 1.4702991247177124, 1.4642237424850464, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, 2.0666091442108154, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, 1.805061936378479, np.inf, np.inf, np.inf, np.inf, np.inf, 1.069167137145996, 1.082072377204895, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, 2.329937696456909, 2.306286096572876, 2.2681221961975098, 2.2556955814361572, 2.2481026649475098, 2.2139108180999756, 2.215308666229248, 2.199082851409912, 2.1816086769104004, 2.165843963623047, 2.1444692611694336, 2.1422994136810303, 2.1313416957855225, 2.1096675395965576, 2.1090710163116455, 2.0912063121795654, 2.0778756141662598, 2.1144309043884277, 2.07482647895813, 2.0625383853912354, 2.063170909881592, 2.0717904567718506, 2.059558868408203, 2.0625386238098145, 2.06538987159729, 2.0550453662872314, 2.0357770919799805, 2.0505423545837402, 2.052006244659424, 2.0401837825775146, 2.063960313796997, 2.0440032482147217, 2.0525245666503906, 2.043628215789795, 2.0595736503601074, 2.063908576965332, 2.069246292114258, 2.059171676635742, 2.0811781883239746, 2.0651938915252686, 2.0738396644592285, 2.103273391723633, 2.1146833896636963, 2.1191859245300293, 2.123729705810547, 2.145165205001831, 2.144284963607788, 2.1708824634552, 2.1832454204559326, 2.201606512069702, 2.1967508792877197, 2.238013505935669, 2.2176196575164795, 2.22042179107666, 2.2650794982910156, 2.298957586288452, 2.3165903091430664, 2.275509834289551, 2.204719305038452, 2.1306097507476807, 2.104065179824829, 2.0423059463500977, 1.9573352336883545, 1.9303737878799438, 1.8777748346328735, 1.8372154235839844, 1.803581714630127, 1.7515147924423218, 1.7373700141906738, 1.6931283473968506, 1.6418960094451904, 1.6170916557312012, 1.5967025756835938, 1.5621219873428345, 1.5158418416976929, 1.5276679992675781, 1.4801164865493774, 1.4581806659698486, 1.4275635480880737, 1.4128082990646362, 1.4137755632400513, 1.3761701583862305, 1.3728508949279785, 1.324769377708435, 1.3228676319122314, 1.3157939910888672, 1.3068455457687378, 1.2785882949829102, 1.2840683460235596, 1.2641929388046265, 1.2484261989593506, 1.2310327291488647, 1.2315911054611206, 1.2049386501312256, 1.196150541305542, 1.1829098463058472, 1.176631212234497, 1.1966135501861572, 1.1873520612716675, 1.1604279279708862, 1.1630767583847046, 1.1513898372650146, 1.1628444194793701, 1.139439582824707, 1.1571826934814453, 1.1416635513305664, 1.1158643960952759, 1.1157498359680176, 1.1469957828521729, 1.1174031496047974, 1.1088932752609253, 1.0908818244934082, 1.1145985126495361, 1.0924185514450073, 1.0884041786193848, 1.1293375492095947, 1.109930157661438, 1.0886338949203491, 1.0908536911010742, 1.0846282243728638, 1.0932687520980835, 1.1163721084594727, 1.1091641187667847, 1.0957788228988647, 1.1188993453979492, 1.1189392805099487, 1.1041285991668701, 1.1219738721847534, 1.112269401550293, 1.1247317790985107, 1.124836802482605, 1.1405507326126099, 1.114134669303894, 1.1442862749099731, 1.1387819051742554, 1.1285794973373413, 1.1675862073898315, 1.149530291557312, 1.1914793252944946, 1.17721426486969, 1.1914196014404297, 1.1930932998657227, 1.1967774629592896, 1.2251079082489014, 1.223677396774292, 1.241024136543274, 1.2467763423919678, 1.2646878957748413, 1.240244746208191, 1.296105980873108, 1.2955365180969238, 1.3098816871643066, 1.3330631256103516, 1.349632978439331, 1.3698318004608154, 1.3877896070480347, 1.4095478057861328, 1.4279240369796753, 1.4511326551437378, 1.450565218925476, 1.4923769235610962, 1.4978573322296143, 1.5364766120910645, 1.5616438388824463, 1.5777199268341064, 1.6193535327911377, 1.6302549839019775, 1.687504529953003, 1.6946029663085938, 1.7349472045898438, 1.8143668174743652, 1.8539183139801025, 1.8830533027648926, 1.9215643405914307, 1.9747108221054077, 2.0174567699432373, 2.0992558002471924, 2.1386733055114746, 2.2172038555145264, 2.2852206230163574, 2.347177743911743, 2.4375672340393066, 2.508974075317383, 2.6150856018066406, 2.6998167037963867, 2.8269219398498535, 2.9407660961151123, 3.0970327854156494, 3.2217986583709717, 3.403921127319336, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, 2.913327932357788, 2.9088351726531982, 2.8851568698883057, 2.8984084129333496)




feature_extraction(msg)
plt.show()

# rospy.init_node('scan_values')
# sub = rospy.Subscriber('/scan', LaserScan, feature_extraction)
# rospy.spin()

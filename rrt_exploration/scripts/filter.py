#!/usr/bin/env python

# --------Include modules---------------
from copy import copy, deepcopy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
import tf
from numpy import array, vstack, delete
from functions import gridValue, informationGain, unvalid
from sklearn.cluster import MeanShift
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
# Subscribers' callbacks------------------------------
mapData = OccupancyGrid()
frontiers = []
global1 = OccupancyGrid()
global2 = OccupancyGrid()
global3 = OccupancyGrid()
globalmaps = []


def callBack(data, args):
    global frontiers, min_distance
    transformedPoint = args[0].transformPoint(args[1], data)
    # x = [array([transformedPoint.point.x, transformedPoint.point.y])]
    x = [array([data.point.x, data.point.y])]
    if len(frontiers) > 0:
        frontiers = vstack((frontiers, x))
    else:
        frontiers = x


def mapCallBack(data):
    global mapData
    mapData = data



# Node----------------------------------------------
def node():
    global frontiers, mapData, global1, global2, global3, globalmaps
    rospy.init_node('filter', anonymous=False)

    # fetching all parameters
    map_topic = rospy.get_param('~map_topic', '/global_map')
    threshold = rospy.get_param('~costmap_clearing_threshold', 70)
    info_radius = rospy.get_param('~info_radius',
                                  1.0)  # this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
    goals_topic = rospy.get_param('~goals_topic', '/detected_points')
    n_robots = rospy.get_param('~n_robots', 1)
    rateHz = rospy.get_param('~rate', 100)

    rate = rospy.Rate(rateHz)
    # -------------------------------------------
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)

    # ---------------------------------------------------------------------------------------------------------------

    for i in range(0, n_robots):
        globalmaps.append(OccupancyGrid())

    # wait if map is not received yet
    while (len(mapData.data) < 1):
        pass
    # wait if any of robots' global costmap map is not received yet

    # for i in range(0, n_robots):
    #     while (len(globalmaps[i].data) < 1):
    #         pass

    global_frame = "/" + mapData.header.frame_id

    tfLisn = tf.TransformListener()
    # for i in range(0, n_robots):
    #     tfLisn.waitForTransform(global_frame[1:], 'robot_' + str(i + 1) + '/base_link', rospy.Time(0),
    #                             rospy.Duration(10.0))

    rospy.Subscriber(goals_topic, PointStamped, callback=callBack, callback_args=[tfLisn, global_frame[1:]])
    pub = rospy.Publisher('frontiers', Marker, queue_size=10)
    pub2 = rospy.Publisher('centroids', Marker, queue_size=10)
    filterpub = rospy.Publisher('filtered_points', PoseArray, queue_size=10)

    rospy.loginfo("the map and global costmaps are received")

    # wait if no frontier is received yet
    while len(frontiers) < 1:
        pass

    temppoint = PointStamped()
    temppoint.header.frame_id = mapData.header.frame_id
    temppoint.header.stamp = rospy.Time(0)
    temppoint.point.z = 0.0

    arraypoints = PoseArray()
    arraypoints.header.frame_id = mapData.header.frame_id
    arraypoints.header.stamp = rospy.Time(0)
    tempPoint = Pose()
    tempPoint.position.z = 0.0
    # -------------------------------------------------------------------------
    # ---------------------     Main   Loop     -------------------------------
    # -------------------------------------------------------------------------
    while not rospy.is_shutdown():
        # -------------------------------------------------------------------------
        # Clustering frontier points
        centroids = []
        front = copy(frontiers)
        if len(front) > 1:
            ms = MeanShift(bandwidth=2)
            ms.fit(front)
            centroids = ms.cluster_centers_  # centroids array is the centers of each cluster

        # if there is only one frontier no need for clustering, i.e. centroids=frontiers
        if len(front) == 1:
            centroids = front
        # frontiers = copy(centroids)
        # -------------------------------------------------------------------------
        # clearing old frontiers
        z = 0
        while z < len(centroids):
            cond = False
            temppoint.point.x = centroids[z][0];
            temppoint.point.y = centroids[z][1];

            for i in range(0, n_robots):
                # transformedPoint = tfLisn.transformPoint(globalmaps[i].header.frame_id, temppoint)
                # x = array([transformedPoint.point.x, transformedPoint.point.y])
                # cond = (gridValue(globalmaps[i], x) > threshold) or cond
                x = [centroids[z][0], centroids[z][1]]
                cond = unvalid(mapData, x) or cond

            if (cond or (informationGain(mapData, [centroids[z][0], centroids[z][1]], info_radius * 0.5)) < 0.2):
                centroids = delete(centroids, (z), axis=0)
                z = z - 1
            z += 1
        # delete useless frontiers
        frontiers = copy(centroids)
        rospy.loginfo(" filtered centroids size: " + str(len(centroids)))
        # -------------------------------------------------------------------------
        # publishing
        arraypoints.poses = []
        for i in centroids:
            tempPoint.position.x = i[0]
            tempPoint.position.y = i[1]
            # warn : must use deep copy
            arraypoints.poses.append(deepcopy(tempPoint))
        filterpub.publish(arraypoints)

        # markers settings
        points = Marker()
        points_clust = Marker()
        # Set the frame ID and timestamp.  See the TF tutorials for information on these.
        points.header.frame_id = mapData.header.frame_id
        points.header.stamp = rospy.Time(0)

        points.ns = "markers2"
        points.id = 0

        points.type = Marker.POINTS

        # Set the marker action for latched frontiers.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        points.action = Marker.ADD;

        points.pose.orientation.w = 1.0

        points.scale.x = 0.6
        points.scale.y = 0.6

        points.color.r = 255.0 / 255.0
        points.color.g = 255.0 / 255.0
        points.color.b = 0.0 / 255.0

        points.color.a = 1;
        points.lifetime = rospy.Duration();

        points_clust.header.frame_id = mapData.header.frame_id
        points_clust.header.stamp = rospy.Time.now()

        points_clust.ns = "markers3"
        points_clust.id = 4

        points_clust.type = Marker.POINTS

        # Set the marker action for centroids.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        points_clust.action = Marker.ADD;
        points_clust.pose.orientation.w = 1.0;
        points_clust.scale.x = 0.8;
        points_clust.scale.y = 0.8;
        points_clust.color.r = 0.0 / 255.0
        points_clust.color.g = 255.0 / 255.0
        points_clust.color.b = 0.0 / 255.0
        points_clust.color.a = 1;
        points_clust.lifetime = rospy.Duration();
        pp = []
        p = Point()
        p.z = 0;

        for q in range(0, len(frontiers)):
            p.x = frontiers[q][0]
            p.y = frontiers[q][1]
            pp.append(deepcopy(p))
        points.points = pp
        pp = []
        for q in range(0, len(centroids)):
            p.x = centroids[q][0]
            p.y = centroids[q][1]
            pp.append(deepcopy(p))
        points_clust.points = pp

        pub.publish(points)
        pub2.publish(points_clust)
        rate.sleep()


# -------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

# --------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import tf
from geometry_msgs.msg import PoseArray, Quaternion
from time import time
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from functions import robot, informationGain, getYawToUnknown, discount
from numpy.linalg import norm
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

# Subscribers' callbacks------------------------------
mapData = OccupancyGrid()
frontiers = []
global1 = OccupancyGrid()
global2 = OccupancyGrid()
global3 = OccupancyGrid()
globalmaps = []


def callBack(data):
    global frontiers
    frontiers = []
    for pose in data.poses:
        frontiers.append(array([pose.position.x, pose.position.y]))


def mapCallBack(data):
    global mapData
    mapData = data


# Node----------------------------------------------

def node():
    global frontiers, mapData, global1, global2, global3, globalmaps
    rospy.init_node('assigner', anonymous=False)

    # fetching all parameters
    map_topic = rospy.get_param('~map_topic', '/global_map')
    info_radius = rospy.get_param('~info_radius',
                                  1.0)  # this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
    info_multiplier = rospy.get_param('~info_multiplier', 3.0)
    hysteresis_radius = rospy.get_param('~hysteresis_radius', 20.0)  # at least as much as the laser scanner range
    hysteresis_gain = rospy.get_param('~hysteresis_gain',
                                      2.0)  # bigger than 1 (biase robot to continue exploring current region
    frontiers_topic = rospy.get_param('~frontiers_topic', '/filtered_points')
    n_robots = rospy.get_param('~n_robots', 1)
    delay_after_assignement = rospy.get_param('~delay_after_assignement', 0.5)
    rateHz = rospy.get_param('~rate', 100)

    rate = rospy.Rate(rateHz)
    # -------------------------------------------
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    rospy.Subscriber(frontiers_topic, PoseArray, callBack)
    assigned_pub = rospy.Publisher('assigned_centroid', Marker, queue_size=10)
    goal_pub = rospy.Publisher('goal_point', PoseStamped, queue_size=1)

# ---------------------------------------------------------------------------------------------------------------

    # wait if no frontier is received yet
    while len(frontiers) < 1:
        pass
    centroids = copy(frontiers)
    # wait if map is not received yet
    while (len(mapData.data) < 1):
        pass


    # get initial position
    listener0 = tf.TransformListener()
    cond0 = 0;
    while cond0 == 0:
        try:
            (trans0, rot0) = listener0.lookupTransform('/odom', '/base_link', rospy.Time(0))
            cond0 = 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            cond0 == 0

    p = Point()
    p.x = 0;
    p.y = 0;
    p.z = 0;
    # -------------------------------------------------------------------------
    # ---------------------     Main   Loop     -------------------------------
    # -------------------------------------------------------------------------
    while not rospy.is_shutdown():
        centroids = copy(frontiers)
        # -------------------------------------------------------------------------
        # Get information gain for each frontier point
        infoGain = []
        for ip in range(0, len(centroids)):
            infoGain.append(informationGain(mapData, [centroids[ip][0], centroids[ip][1]], info_radius))
        # decrease info gain of centroid that overlap with current goal centroid
        infoGain = discount(mapData, array([p.x, p.y]), centroids, infoGain, info_radius)

        # -------------------------------------------------------------------------
        # # get number of available/busy robots
        # na = []  # available robots --> last goal point is arrived
        # nb = []  # busy robots --> last goal point isn't arrived
        # for i in range(0, n_robots):
        #     if (robots[i].getState() == 1):
        #         nb.append(i)
        #     else:
        #         na.append(i)
        # rospy.loginfo("available robots: " + str(na))
        # -------------------------------------------------------------------------
        # # get dicount and update informationGain
        # for i in nb + na:
        #     infoGain = discount(mapData, robots[i].assigned_point, centroids, infoGain, info_radius)
        # -------------------------------------------------------------------------
        # update position
        listener = tf.TransformListener()
        cond = 0;
        while cond == 0:
            try:
                (trans, rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond == 0
        current_position = array([trans[0] - trans0[0], trans[1] - trans0[1]])

        revenue_record = []
        centroid_record = []

        for ip in range(0, len(centroids)):
            cost = norm(current_position - centroids[ip])
            threshold = 1
            information_gain = infoGain[ip]
            if (norm(current_position - centroids[ip]) <= hysteresis_radius):
                information_gain *= hysteresis_gain
            else:
                information_gain *= threshold
            # rospy.loginfo("information_gain: " + str(information_gain))
            # rospy.loginfo("navi_cost: " + str(cost))
            revenue = information_gain * info_multiplier - cost
            revenue_record.append(revenue)
            centroid_record.append(centroids[ip])

        # rospy.loginfo("revenue record: " + str(revenue_record))
        # rospy.loginfo("centroid record: " + str(centroid_record))

        # -------------------------------------------------------------------------
        winner_id = revenue_record.index(max(revenue_record))
        # robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
        rospy.loginfo("  chosen centroid ID:  " + str(centroid_record[winner_id]) +
                      "  revenue value :  " + str(max(revenue_record)))
        # publish assigned centroid
        points = Marker()
        # Set the frame ID and timestamp.  See the TF tutorials for information on these.
        points.header.frame_id = mapData.header.frame_id
        points.header.stamp = rospy.Time(0)

        points.id = 0

        points.type = Marker.POINTS

        # Set the marker action for latched frontiers.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        points.action = Marker.ADD;

        points.pose.orientation.w = 1.0

        points.scale.x = 1.5
        points.scale.y = 1.5

        points.color.r = 255.0 / 255.0
        points.color.g = 255.0 / 255.0
        points.color.b = 0.0 / 255.0

        points.color.a = 1;
        points.lifetime = rospy.Duration();


        pp = []
        p.x = centroid_record[winner_id][0]
        p.y = centroid_record[winner_id][1]
        pp.append(copy(p))
        points.points = pp
        assigned_pub.publish(points)



        yaw = getYawToUnknown(mapData, centroid_record[winner_id], info_radius)
        q = quaternion_from_euler(0.0, 0.0, yaw)
        # publisher goal point
        goal_point = PoseStamped()
        goal_point.header.frame_id = mapData.header.frame_id
        goal_point.header.stamp = rospy.Time(0)
        goal_point.pose.position.x = centroid_record[winner_id][0]
        goal_point.pose.position.y = centroid_record[winner_id][1]
        goal_point.pose.orientation = Quaternion(*q)
        goal_pub.publish(goal_point)

        # -------------------------------------------------------------------------
        rate.sleep()

# -------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass

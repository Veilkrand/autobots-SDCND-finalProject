#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from operator import itemgetter

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.timer = 0
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.currenPosition = None
        self.waypoints = []
        rospy.spin()

    def pose_cb(self, msg):
        self.currenPosition = msg.pose.position
        self.updateWaypoints()

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distances(self, waypoints, current_position):
        """
        Returns an iterator with distances from current position to each waypoint
        """
        # Get the coordinates for all waypoint objects
        get_coordinates = lambda waypoint: waypoint.pose.pose.position
        waypoint_coordinates = map(get_coordinates, waypoints)

        # Calculate all distances
        c = current_position
        dist_to_a = lambda a: math.sqrt((a.x-c.x)**2 + (a.y-c.y)**2  + (a.z-c.z)**2)
        distances = map(dist_to_a, waypoint_coordinates)

        return distances


    ###
    ### Publishing the next LOOKAHEAD_WPS waypoints to final_waypoints
    ###
    def updateWaypoints(self):
        # Make sure the current position is not the deafult initialization
        if self.currenPosition is not None:

            #calculate distances from current position to each waypoint
            distances = self.distances(self.waypoints, self.currenPosition)

            # find index of closest waypoint, which has the shortest distance
            closest_index, _ = min(enumerate(distances), key=itemgetter(1))

            # Use the waypoints following the current waypoint as the next
            # short term goals.
            next_waypoint = min(closest_index + 1, len(self.waypoints))
            last_waypoint = min(next_waypoint + LOOKAHEAD_WPS, len(self.waypoints))
            next_waypoints = self.waypoints[next_waypoint:last_waypoint]

            self.final_waypoints_pub.publish(next_waypoints)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

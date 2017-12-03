#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray, TrafficLight
from std_msgs.msg import Int32
from operator import itemgetter

import math
import yaml
import copy

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
TL_DETCTOR_EABLED = False

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        config_string = rospy.get_param("/traffic_light_config")
        self.trafic_light_positions = yaml.load(config_string)['stop_line_positions']

        self.timer = 0
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # Subscriber for /traffic_waypoint and /obstacle_waypoint below
        if TL_DETCTOR_EABLED:
            rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        # Suscriber to the traffic light information from the simulator
        if not TL_DETCTOR_EABLED:
            rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.sim_traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.prev_index = -1
        self.current_position = None
        self.waypoints = []
        self.green_waypoints = []
        self.red_waypoints = []
        rospy.spin()

    def pose_cb(self, msg):
        self.current_position = msg.pose.position
        self.updateWaypoints()

    def waypoints_cb(self, msg):
        self.green_waypoints = msg.waypoints
        # clone waypoints to initialize red_waypoints
        self.red_waypoints = copy.deepcopy(msg.waypoints)

        # use traffic light locations to adjust red_waypoints' velocities
        for trafic_light in self.trafic_light_positions:
            tl_position = Point(x = trafic_light[0], y = trafic_light[1], z = 0)
            closest_wp_index = self.get_closest_waypoint_to_point(self.red_waypoints, tl_position)

            #Car will stop in the waypoint right before the stoplight
            stop_index = max(0, closest_wp_index - 1)
            self.red_waypoints = self.decelerate(self.red_waypoints, closest_wp_index, max_decel = 0.5)

        #Initialize waypoint assuming red light ahead.
        self.waypoints = self.red_waypoints

    def update_waypoints(self, traffic_light_state):
        if traffic_light_state == TrafficLight.GREEN:
            # When a green light is detected the car should follow the original
            # waypoint velocitites set by the /base_waypoint topic.
            self.waypoints = self.green_waypoints
        else:
            # When a green light is not detected, the car should follow
            # waypoints that gently stop at every traffic light.
            self.waypoints = self.red_waypoints

    def traffic_cb(self, msg):
        # The waypoint updater is conservative. The car stops at every
        # traffic light, unless a green light is detected.
        self.update_waypoints(msg.data)

    def sim_traffic_cb(self, msg):
        # Callback for /vehicle/traffic_lights message.
        self.update_waypoints(msg.lights[0].state)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def distance_2p(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    def decelerate(self, waypoints, stop_index, max_decel = 1):
        last = waypoints[stop_index]
        last.twist.twist.linear.x = 0.
        for wp in waypoints[:stop_index][::-1]:
            dist = self.distance_2p(wp.pose.pose.position, last.pose.pose.position)
            #decelerate at a constant rate (cuadratic decline in velocity).
            vel = math.sqrt(2 * max_decel * dist)
            if vel < 2:
                vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints

    def distances_to_pose(self, waypoints, current_position):
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

    def get_closest_waypoint_to_point(self, waypoints, pose):
        #calculate distances from current position to each waypoint
        distances = self.distances_to_pose(waypoints, pose)

        # find index of closest waypoint, which has the shortest distance
        closest_index, _ = min(enumerate(distances), key=itemgetter(1))
        return closest_index

    def make_lane_msg(self, waypoints):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        return lane

    ###
    ### Publishing the next LOOKAHEAD_WPS waypoints to final_waypoints
    ###
    def updateWaypoints(self):
        # Make sure the current position is not the deafult initialization
        if self.current_position is not None and len(self.waypoints) > 0:

            # find index of closest waypoint, which has the shortest distance
            closest_index = self.get_closest_waypoint_to_point(self.waypoints, self.current_position)

            # Use the waypoints following the current waypoint as the next
            # short term goals.
            next_waypoint = min(closest_index + 1, len(self.waypoints))
            last_waypoint = min(next_waypoint + LOOKAHEAD_WPS, len(self.waypoints))
            next_waypoints = self.waypoints[next_waypoint:last_waypoint]
            get_vel = lambda wp: wp.twist.twist.linear.x

            #if closest_index > self.prev_index:
                #print closest_index, list(map(get_vel,next_waypoints))[:5]
                #self.prev_index = closest_index

            #Create a message of type Lane and publish it on the final_waypoints topic
            final_waypoints_message = self.make_lane_msg(next_waypoints)
            self.final_waypoints_pub.publish(final_waypoints_message)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

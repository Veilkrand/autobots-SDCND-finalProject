#!/usr/bin/env python
#

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math
import time

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')
        rospy.loginfo("dbw_node-initmode = %s",time.time())


        # Below are the parameters used for creating `TwistController` object
        #---------------------------------------------------------------------
        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        min_speed = rospy.get_param('~min_speed', 0.0)  # added for yaw_controller taking default 0.0 for now

        #Parameters under consideraion
        #----------------------------------------------------------------------

        # Created  TwistController Object
        #---------------------------------------------------------------------
        self.controller = Controller(
                              wheel_base,
                              vehicle_mass,
                              steer_ratio,
                              min_speed,
                              max_lat_accel,
                              max_steer_angle)

        rospy.loginfo("Controller-created = %s",time.time())


        # Below three are the output to be published from the dbw_node
        #--------------------------------------------------------------------
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)




        #Initialised Variables
        #----------------------------------------------------------------------
        self.dbw_enabled = False
        rospy.loginfo("self.dbw_enabled = %s",self.dbw_enabled)
        self.twist = None
        rospy.loginfo("self.twist = %s",self.twist)
        self.current_velocity = None
        rospy.loginfo("self.current_velocity = %s",self.current_velocity)


        # Topics to be subscribed
        #---------------------------------------------------------------------
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)


        self.loop()
    ######################################################################
    def loop(self):

        rate = rospy.Rate(30) # 50Hz

        while not rospy.is_shutdown():

            if self.twist is None or self.current_velocity is None:
                rospy.loginfo("self.twist is None = %s",self.twist)
                rospy.loginfo( "self.twist is None-time = %s",time.time())
                pass
            else:
                desired_linear_velocity = self.twist.linear.x
                rospy.loginfo("desired_linear_velocity = %s",desired_linear_velocity)

                desired_angular_velocity = self.twist.angular.z
                rospy.loginfo("desired_angular_velocity = %s",desired_angular_velocity)
                current_linear_velocity = self.current_velocity.linear.x
                rospy.loginfo("current_linear_velocity = %s",current_linear_velocity)
                current_angular_velocity = self.current_velocity.angular.z
                rospy.loginfo("current_angular_velocity = %s",current_angular_velocity)

        # Get predicted throttle, brake, and steering using control method of controller object
        #----------------------------------------------------------------------
                throttle, brake, steer = self.controller.control(
                                             desired_linear_velocity,
                                             desired_angular_velocity,
                                             current_linear_velocity,
                                             current_angular_velocity)


        # You should only publish the control commands if dbw is enabled
        #----------------------------------------------------------------------
                if self.dbw_enabled:
                    rospy.loginfo("tbs-publishing time = %s", time.time())
                    self.publish(throttle, brake, steer)
                else:
                    rospy.loginfo("its manual-time = %s", time.time())


            rate.sleep()




    #------------------------------------------------------------------------------
    def current_velocity_cb(self, msg):
        rospy.loginfo("current_velocity_cb-time = %s",time.time())
        self.current_velocity = msg.twist

    def twist_cmd_cb(self, msg):
        rospy.loginfo("twist_cmd_cb-time = %s", time.time())
        self.twist = msg.twist

    def dbw_enabled_cb(self, msg):
        rospy.loginfo("dbw_enabled_cb-time = %s", time.time())
        self.dbw_enabled = msg.data




    ###########  No Need to update following #######################################
    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()

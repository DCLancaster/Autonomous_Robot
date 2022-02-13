#!/usr/bin/env python

import rospy

from math import sin, cos  # , pi

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from autonomous_bot.msg import stepper_angle


class DiffTf:
    def __init__(self):
        rospy.init_node("diff_tf")
        self.nodename = rospy.get_name()

        '''----------------------------------------CALLING ALL PARAMETERS--------------------------------------------'''
        self.rate = rospy.get_param('~rate', 5.0)  # the rate at which to publish the transform, 30 matches other TFs
        self.ticks_meter = float(rospy.get_param('Ticks_Per_Meter'))  # Equations are in ticks per meter,
        #                                                               could convert to distance per tick...bite me
        self.base_width = float(rospy.get_param('Robot_Width'))  # The wheel base width in meters

        self.Robot_Name = rospy.get_param('Robot_Name')
        self.base_frame_id = '/base_footprint'  # the name of the base frame of the robot
        self.odom_frame_id = '/odom'  # the name of the odometry reference frame

        self.encoder_min = rospy.get_param('encoder_min')
        self.encoder_max = rospy.get_param('encoder_max')
        self.encoder_low_wrap = self.encoder_min
        self.encoder_high_wrap = self.encoder_max

        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = rospy.Time.now() + self.t_delta

        '''----------------------------------------INTERNAL PARAMETERS----------------------------------------------'''
        self.enc_left = None  # raw step input
        self.enc_right = None
        self.left = 0  # actual distance
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_l_enc = 0
        self.prev_r_enc = 0
        self.x = 0  # position in xy plane
        self.y = 0
        self.th = 0
        self.dx = 0  # speeds in x/rotation
        self.dr = 0
        self.then = rospy.Time.now()

        # subscriptions
        rospy.Subscriber('L/Wheel_Angle', stepper_angle, self.lwheelCallback)
        rospy.Subscriber('R/Wheel_Angle', stepper_angle, self.rwheelCallback)
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()

    #############################################################################
    def spin(self):
        #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    #############################################################################
    def update(self):
        #############################################################################
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            # calculate odometry
            if self.enc_left is None:
                d_left = 0
                d_right = 0
            else:
                d_left = (self.left - self.enc_left) / self.ticks_meter
                d_right = (self.right - self.enc_right) / self.ticks_meter
            self.enc_left = self.left
            self.enc_right = self.right

            # distance traveled is the average of the two wheels
            d = (d_left + d_right) / 2
            # this approximation works (in radians) for small angles
            th = -(d_right - d_left) / self.base_width
            # calculate velocities
            self.dx = d / elapsed
            self.dr = th / elapsed

            if d != 0:
                # calculate distance traveled in x and y
                x = cos(th) * d
                y = -sin(th) * d
                # calculate the final position of the robot
                self.x = self.x + (cos(self.th) * x - sin(self.th) * y)
                self.y = self.y + (sin(self.th) * x + cos(self.th) * y)
            if th != 0:
                self.th = self.th + th

            # publish the odom information
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2)
            quaternion.w = cos(self.th / 2)
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
            )

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.dr
            self.odomPub.publish(odom)

    def lwheelCallback(self, msg):
        self.left = msg.stepper_angle

    def rwheelCallback(self, msg):
        self.right = msg.stepper_angle


if __name__ == '__main__':
    """ main """
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass

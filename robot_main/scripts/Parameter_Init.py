#!/usr/bin/env python2
import rospy

# base parameters
rospy.init_node("Parameter_Init")

Wheel_Radius_m = rospy.get_param("Wheel_Radius_m", 0.025)
rospy.set_param("Wheel_Radius_m", Wheel_Radius_m)

Robot_Width = rospy.get_param("Robot_Width", 0.4)
rospy.set_param("Robot_Width", Robot_Width)

Max_Wheel_Speed = rospy.get_param("Max_Wheel_Speed", 0.05)
rospy.set_param("Max_Wheel_Speed", Max_Wheel_Speed)

Min_Wheel_Speed = rospy.get_param("Min_Wheel_Speed", 0.005)
rospy.set_param("Min_Wheel_Speed", Min_Wheel_Speed)

Wheel_Acceleration = rospy.get_param("Wheel_Acceleration", 0.0125)
rospy.set_param("Wheel_Acceleration", Wheel_Acceleration)

Number_Poles_Stepper = rospy.get_param("Number_Poles_Stepper", 200)
rospy.set_param("Number_Poles_Stepper", Number_Poles_Stepper)

Monitoring_Delay = rospy.get_param("Monitoring_Delay", 1)
rospy.set_param("Monitoring_Delay", Monitoring_Delay)

# derived parameters
n_t = rospy.get_param('Number_Poles_Stepper')
r = rospy.get_param('Wheel_Radius_m')
t_p_m = n_t / (2 * 3.14 * r)
rospy.set_param('Ticks_Per_Meter', t_p_m)
rospy.set_param('encoder_min', 0)
rospy.set_param('encoder_max', rospy.get_param('Number_Poles_Stepper'))
print "Parameters initialised"

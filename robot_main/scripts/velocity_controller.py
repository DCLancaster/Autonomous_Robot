#!/usr/bin/env python2
import rospy
import sys
from autonomous_bot.msg import stepper_freq
from geometry_msgs.msg import Twist

'''Controller for left or right motor?'''
side_of_bot = sys.argv[1]  # "Right" OR "Left" ONLY

'''------------------------------------------------------------------------------------------------------------------'''
'''-------------------------------------------CALL GLOBAL PARAMETERS-------------------------------------------------'''
'''------------------------------------------------------------------------------------------------------------------'''
rr = rospy.get_param(rospy.search_param('Wheel_Radius_m'))
maxws_int = rospy.get_param(rospy.search_param('Max_Wheel_Speed'))
n_s = rospy.get_param(rospy.search_param('Number_Poles_Stepper'))
acc_c = rospy.get_param(rospy.search_param('Wheel_Acceleration'))
CVS = rospy.get_param(rospy.search_param('cmd_vel_scale'))


def call_global_parameters():  # makes global parameters callable later for live editing
    global maxws, acc_c, CVS
    maxws = rospy.get_param(rospy.search_param('Max_Wheel_Speed'))
    acc_c = rospy.get_param(rospy.search_param('Wheel_Acceleration'))
    CVS = rospy.get_param(rospy.search_param('cmd_vel_scale'))


'''local variables'''
d_ws = 0  # delta wheel speed to increment
cws = 0  # current_wheel_speed
iws = 0  # internal wheel speed for calculations
tws = 0  # initial target_wheel_speed
tws_int = 0  # internal target wheel speed for rounding
fz = 1  # initial stepper frequency (for each step)
f_acc = 15  # frequency of vel calc (doesn't effect acceleration)
maxws = (acc_c/f_acc) * round(maxws_int/(acc_c/f_acc))  # ensures max speed is a value that is calculable
# setting variables based on left or right side
if side_of_bot == "Left":
    dir_m = 1
elif side_of_bot == "Right":
    dir_m = -1
else:
    print('Naming error, see motor controller:', side_of_bot)
    PUB_NAME = "Velocity_Control_LR_node_ERROR"
    dir_m = 0  # you best hope this don't happen

'''------------------------------------------------------------------------------------------------------------------'''
'''----------------------------------------------------FUNCTIONS-----------------------------------------------------'''
'''------------------------------------------------------------------------------------------------------------------'''


def callback(msg):  # Function is called for every received cmd_vel command. Just sets the variable for use later.
    global tws, tws_int, acc_c
    forward_velocity = clamp(msg.linear.x/1, -0.5, 0.5)
    angular_velocity = clamp(msg.angular.z/3, -1, 1)
    tws_int = (forward_velocity + (dir_m * angular_velocity))/CVS  # uses direction of motor to change LR orientation
    tws = (acc_c/f_acc) * round(tws_int/(acc_c/f_acc))
    rospy.logdebug("%s New L Target speed:      %s", side_of_bot, tws)
    rospy.logdebug("%s New L exact TWS:         %s", side_of_bot, tws_int)
    rospy.logdebug("%s New L angular_velocity:  %s", side_of_bot, angular_velocity)
    rospy.logdebug("%s New L forward_velocity:  %s", side_of_bot, forward_velocity)


def clamp(n, minn, maxn):  # little solution to min max a value
    return max(min(maxn, n), minn)


def abs_clamp(n, minn, maxn):  # modified clamp to maintain respect to sign of input (proud of this little guy <3)
    if n > 0:
        return max(min(maxn, abs(n)), minn)
    if n < 0:
        return -(max(min(maxn, abs(n)), minn))
    if n == 0:
        return minn


def incr_wheel_speed(target_ws, acceleration, current_ws):  # increments speed with acceleration parameter
    if current_ws < target_ws:
        current_ws += acceleration
    elif current_ws > target_ws:
        current_ws -= acceleration
    return clamp(current_ws, -maxws, maxws)


def calc_freq(wheel_radius, number_steps, wheel_speed):  # calculate new frequency based on internal wheel speed
    return (number_steps * wheel_speed) / (wheel_radius * 2 * 3.14)


'''------------------------------------------------------------------------------------------------------------------'''
'''----------------------------------------------------ROS-----------------------------------------------------------'''
'''------------------------------------------------------------------------------------------------------------------'''
PUB_NAME = "Stepper_Freq"
rospy.init_node("Velocity_Controller")  # make the node with name set earlier from LR controller setting
sub = rospy.Subscriber('/cmd_vel', Twist, callback)  # setup subscriber with Twist message type
# setup publisher with custom message stepper freq
pub = rospy.Publisher(PUB_NAME, stepper_freq, queue_size=1)
output_frequency = stepper_freq()  # create object of type stepper angle (the custom message type)
'''------------------------------------------------------------------------------------------------------------------'''
'''---------------------------------------------------MAIN-----------------------------------------------------------'''
'''------------------------------------------------------------------------------------------------------------------'''
r = rospy.Rate(f_acc)
while not rospy.is_shutdown():
    call_global_parameters()
    iws = round(incr_wheel_speed(tws, acc_c/f_acc, cws), 5)
    cws = (acc_c/f_acc) * round(iws/(acc_c/f_acc))  # rounding to keep values a multiple off acceleration constant
    fz = calc_freq(rr, n_s, cws)
    output_frequency.stepper_freq = fz
    pub.publish(output_frequency)
    if rospy.is_shutdown():
        rospy.logdebug("velocity_controller breaking loop")
        break
    rospy.logdebug("acc: %s", acc_c/f_acc)
    rospy.logdebug("cws: %s", cws)
    rospy.logdebug("fz:  %s", fz)
    r.sleep()
rospy.logdebug("End of velocity controller")

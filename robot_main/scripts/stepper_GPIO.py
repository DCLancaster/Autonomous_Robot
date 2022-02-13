#!/usr/bin/env python2
import rospy
import sys
from autonomous_bot.msg import stepper_angle
from autonomous_bot.msg import stepper_freq

rospy.init_node("LR_GPIO_Node")  # , log_level=rospy.DEBUG
side_of_bot = sys.argv[1]  # "Right" OR "Left" ONLY
Robot_Name = rospy.get_param("~/Robot_Name")
GPIO_Enable = rospy.get_param("~/Platform")

'''Initial variables'''
global Stepper_motor_order
fz = 1
step = 0
Dir = 0
debug_pin = ""
debug_coil = ""
debug_state = ""
Stepper_Motor_pins = [1, 1, 1, 1]

'''The holy grail... THE FULL STEP PIN LOGIC'''  # half step was too power consuming :(
full_step_pin_logic = [
    [0, 0, 1, 0],  # -A
    [1, 0, 0, 0],  # +A
    [0, 1, 0, 0],  # +B
    [0, 0, 0, 1]]  # -B

if GPIO_Enable == "Pi":
    import RPi.GPIO as GPIO
# the following uses board pin numbering (BCM), not GPIO numbering
# internal pin ordering is set to follow ordering of physical patch leads
# ordering for yellow bot reversed because lidar is mounted backwards compared to clear robot
if side_of_bot == "Left" and Robot_Name == "C":
    Stepper_Motor_pins = [31, 33, 35, 37]  # in order for "-A", "+A", "+B", "-B"
    rospy.logdebug("L W pins: %s", Stepper_Motor_pins)
elif side_of_bot == "Right" and Robot_Name == "C":
    Stepper_Motor_pins = [18, 16, 12, 10]  # in order for "+A", "+B", "-A", "-B"
    rospy.logdebug("R W pins: %s", Stepper_Motor_pins)
elif side_of_bot == "Left" and Robot_Name == "Y":
    Stepper_Motor_pins = [37, 35, 33, 31]  # in order for "+A", "+B", "-A", "-B"
    rospy.logdebug("L Y pins: %s", Stepper_Motor_pins)
elif side_of_bot == "Right" and Robot_Name == "Y":
    Stepper_Motor_pins = [10, 12, 16, 18]  # in order for "+A", "+B", "-A", "-B"
    rospy.logdebug("R Y pins: %s", Stepper_Motor_pins)
else:
    rospy.logerr("LR WY NAMING ISSUE - GPIO NODE")
    quit()


'''
For swapping in and out:
[31, 33, 35, 37]
[18, 16, 12, 10]
[37, 35, 33, 31]
[10, 12, 16, 18]
'''


def clamp(n, minn, maxn):  # little solution to min max a value
    return max(min(maxn, n), minn)


def callback(msg):
    global fz, Dir  # fz = output freq, Dir = direction or off
    fz_int = msg.stepper_freq  # internal fz for logic purposes
    fz = clamp(abs(fz_int), 20, 500)  # 5<1000
    # Directions here are reversed from the logical values
    # Likely due to stepper motor wiring/pole orders
    # Changing the pole values in the pin assignment could work but would be confusing
    # if not in order of physical pin outs in my opinion. NOT TESTED WITH YELLOW BOT
    if fz_int > 0:
        Dir = 1
    elif fz_int < 0:
        Dir = -1
    elif fz_int == 0:
        Dir = 0


def setup_GPIO():
    global Stepper_Motor_pins, Stepper_motor_order
    Stepper_motor_order = ["-A", "+A", "+B", "-B"]  # really just used for debugging
    if GPIO_Enable == "Pi":
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(Stepper_Motor_pins, GPIO.OUT)
        GPIO.output(Stepper_Motor_pins, 0)
        rospy.logdebug("Stepper_Motor_pins: %s", Stepper_Motor_pins)


def inc_step(step_int):
    step_int += Dir
    if step_int == 4:
        step_int = 0
    elif step_int == -1:
        step_int = 3
    return step_int


SUB_NAME = "Stepper_Freq"
PUB_NAME = "Wheel_Angle"
sub = rospy.Subscriber(SUB_NAME, stepper_freq, callback)
pub = rospy.Publisher(PUB_NAME, stepper_angle, queue_size=0)
Wheel_Angle = stepper_angle()
Wheel_Angle.stepper_angle = 0
pub.publish(Wheel_Angle)
setup_GPIO()
r = rospy.Rate(fz)  # initial spin freq

'''When running on the RPi this code needed optimising to reduce CPU hog and over 200Hz real output'''
if GPIO_Enable == "Pi":
    while not rospy.is_shutdown():
        for coil in range(0, 4):
            GPIO.output(Stepper_Motor_pins[coil], full_step_pin_logic[coil][step])
        step = inc_step(step)
        Wheel_Angle.stepper_angle += Dir
        pub.publish(Wheel_Angle)
        r = rospy.Rate(fz)
        if rospy.is_shutdown():
            break
        r.sleep()

if GPIO_Enable == "Ub":
    while not rospy.is_shutdown():
        GPIO_Debug_str = ""
        for coil in range(0, 4):
            if GPIO_Enable == "Pi":
                GPIO.output(Stepper_Motor_pins[coil], full_step_pin_logic[coil][step])
            debug_pin += str(Stepper_Motor_pins[coil]) + " "
            debug_coil += str(Stepper_motor_order[coil]) + " "
            debug_state += str(full_step_pin_logic[coil][step]) + "  "
        step = inc_step(step)
        Wheel_Angle.stepper_angle += Dir
        pub.publish(Wheel_Angle)
        r = rospy.Rate(fz)
        if rospy.is_shutdown():
            break
        rospy.logdebug("fz:   %s", fz)
        rospy.logdebug("step: %s", step)
        rospy.logdebug("Dir:  %s", Dir)
        rospy.logdebug("Odom out: %s", Wheel_Angle.stepper_angle)
        rospy.logdebug("%s", debug_pin)
        rospy.logdebug("%s", debug_coil)
        rospy.logdebug("%s", debug_state)
        rospy.logdebug(" ")
        debug_pin = ""
        debug_coil = ""
        debug_state = ""
        r.sleep()
if GPIO_Enable == "Pi":
    GPIO.output(Stepper_Motor_pins, 0)
    GPIO.cleanup()
rospy.loginfo('GPIO Cleaned')

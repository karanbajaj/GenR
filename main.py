#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Driving Base Program
-----------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Button, Color, ImageFile, SoundFile
from pybricks.robotics import DriveBase
from ucollections import namedtuple
from pybricks.tools import wait, StopWatch, DataLog



def jason_run():
    jason_start_slide()


def jason_start_slide():
    #slide
    #3rd line angle-
    #drive  backwards for 43 in
    robot.settings(straight_speed=400, straight_acceleration=100, turn_rate=10, turn_acceleration=10)
    robot.straight(1092.2)
    ev3.speaker.beep()
    robot.stop()


    #turn 30 left
    robot.turn(-30)
    robot.stop()

    #straight 10 in
    #drive straight
    robot.settings(straight_speed=150, straight_acceleration=50, turn_rate=10, turn_acceleration=10)
    robot.straight(254)
    robot.stop()
    #turn left 30
    robot.turn(30)
    robot.stop()
    #drive back 32 in
    robot.settings(straight_speed=150, straight_acceleration=50, turn_rate=10, turn_acceleration=10)
    robot.straight(-812.8)
    robot.stop()


def sophie_run():
    sophie_run_basket_ball()


def sophie_run_basket_ball():
    # basketball- 27 in. forward - <3 90 degree turn left - 34 inches forward - (-30 mm) turn - raise bar() -lower bar() - turn 30 degrees- lift bar(baccia) -   
    #drive straight
    #robot.settings(straight_speed=450, straight_acceleration=150, turn_rate=10, turn_acceleration=10)
    robot.straight(-685.8)
    robot.stop()
    #robot.settings(straight_speed=150, straight_acceleration=50, turn_rate=100, turn_acceleration=150)
    
    #turn 90 to face the 
    robot.turn(90)
    robot.stop()
    angle = robot.angle()
    robot.turn(90)
    robot.stop()
    
    robot.turn(90)
    robot.stop()
    



def jolene_run():
    #go forwrd fast
    robot.settings(straight_speed=400, straight_acceleration=100, turn_rate=10, turn_acceleration=10)
    robot.straight(-1016)
    ev3.speaker.beep()
    robot.stop()

    # push step counter - super slow to blue 
    robot.settings(straight_speed=30, straight_acceleration=20, turn_rate=10, turn_acceleration=10)
    robot.straight(-370.4)
    robot.stop()

    #step back from the step counter 
    robot.settings(straight_speed=150, straight_acceleration=50, turn_rate=10, turn_acceleration=10)
    robot.straight(306.4)
    robot.stop()

    #turn left 
    robot.turn(30)
    robot.stop()
    #drive straight
    robot.settings(straight_speed=150, straight_acceleration=50, turn_rate=10, turn_acceleration=10)
    robot.straight(-595)
    robot.stop()
    #turn right
    robot.turn(-30)
    robot.stop()
    #drive upto treadmill
    robot.straight(-593.4)
    ev3.speaker.beep()
    robot.stop()

    #right motor turn
    frnt_right_motor.run_target(500, 4000)

def jolene_test():
    frnt_right_motor.run_target(500, 3,468)



def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.
    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


#start ############################################################ 



# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.C)
right_motor = Motor(Port.D)

frnt_left_motor = Motor(Port.B) 
frnt_right_motor = Motor(Port.A) 

gyro_sensor = GyroSensor(Port.S3)
color_sensor1 = ColorSensor(Port.S4)
color_sensor2 = ColorSensor(Port.S2)
color_sensor3 = ColorSensor(Port.S1)




GYRO_CALIBRATION_LOOP_COUNT = 200
GYRO_OFFSET_FACTOR = 0.0005

# Actions will be used to change which way the robot drives.
Action = namedtuple('Action ', ['drive_speed', 'steering'])

# These are the pre-defined actions
STOP = Action(drive_speed=0, steering=0)
FORWARD_FAST = Action(drive_speed=150, steering=0)
FORWARD_SLOW = Action(drive_speed=40, steering=0)
BACKWARD_FAST = Action(drive_speed=-75, steering=0)
BACKWARD_SLOW = Action(drive_speed=-10, steering=0)
TURN_RIGHT = Action(drive_speed=0, steering=70)
TURN_LEFT = Action(drive_speed=0, steering=-70)


left_motor.reset_angle(0)
right_motor.reset_angle(0)


# Calibrate the gyro offset. This makes sure that the robot is perfectly
# still by making sure that the measured rate does not fluctuate more than
# 2 deg/s. Gyro drift can cause the rate to be non-zero even when the robot
# is not moving, so we save that value for use later.
# while True:
#     gyro_minimum_rate, gyro_maximum_rate = 440, -440
#     gyro_sum = 0
#     for _ in range(GYRO_CALIBRATION_LOOP_COUNT):
#         gyro_sensor_value = gyro_sensor.speed()
#         gyro_sum += gyro_sensor_value
#         if gyro_sensor_value > gyro_maximum_rate:
#             gyro_maximum_rate = gyro_sensor_value
#         if gyro_sensor_value < gyro_minimum_rate:
#             gyro_minimum_rate = gyro_sensor_value
#         wait(5)
#     if gyro_maximum_rate - gyro_minimum_rate < 2:
#         break
# gyro_offset = gyro_sum / GYRO_CALIBRATION_LOOP_COUNT


# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=53, axle_track=120)

#do the runs

#data = DataLog('color')
print('Hello Gen R !')


color1 = color_sensor1.color()
print(color1)

color2 = color_sensor2.color()
print(color2)

color = color_sensor3.color()
print(color)


#data.log(color)
#jolene_run()

if(color==Color.WHITE):
    print("Jolene Run")
    jolene_run()
if(color==Color.BLUE):
    print("Jason Run")
    jason_run()
#
#jolene_test()
if(color==Color.RED):
    print("Sophie Run")
    sophie_run()


# Turn clockwise by 360 degrees and back again.
# robot.turn(360)
# ev3.speaker.beep()

# robot.turn(-360)
# ev3.speaker.beep()

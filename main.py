#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Driving Base Program
-----------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""
import logging
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Button, Color, ImageFile, SoundFile, Stop
from pybricks.robotics import DriveBase
from ucollections import namedtuple
from pybricks.tools import wait, StopWatch, DataLog

# Logging
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(levelname)5s: %(message)s')
log = logging.getLogger(__name__)
log.info("Starting Program")

data = DataLog('time', 'angle', name='my_file', timestamp=False, extension='txt')
start_gyro_value =0

################# robot functions
def turn_robot_left_in_place(angle):
    turn_robot_in_place("LEFT",angle)

def turn_robot_right_in_place(angle):
    turn_robot_in_place("RIGHT",angle)

def turn_robot_in_place(direction,angle):
    current_gyro_angle=gyro_sensor.angle()
    data.log("turn_robot_in_place:: start (current) gyro value:",current_gyro_angle)
    print("turn_robot_in_place:: start (current) gyro value: ", current_gyro_angle)
    #gyro_sensor.reset_angle(0)
    #print("gyro angle reset: ", gyro_sensor.angle())
    #current_gyro_angle=abs(gyro_sensor.angle())
    print("turn_robot_in_place:: calculating direction multipliers")

    left_direction = 1
    right_direction = -1
    angle_direction = 1

    if(direction=="LEFT"):
        right_direction = -1
        left_direction = 1
        angle_direction = -1
    if(direction=="RIGHT"):
        right_direction = 1
        left_direction = -1
        angle_direction = 1
    
    print("turn_robot_in_place:: turning robot in direction: ",direction)

    angle_we_want = current_gyro_angle + angle * angle_direction
    
   
    print("turn_robot_in_place::angle reguested: ",angle) 
    print("turn_robot_in_place::angle_we_want: ",angle_we_want)
 
    data.log("turn_robot_in_place::angle reguested: ",angle) 
    data.log("turn_robot_in_place::angle_we_want: ",angle_we_want)
    
    

   
    while( current_gyro_angle <angle_we_want*0.85):
        #
        left_motor.run(200*left_direction)
        right_motor.run(200*right_direction)
        wait(10)
        current_gyro_angle= abs(gyro_sensor.angle())
        print("turn_robot_in_place::gyro angle 85%: ", current_gyro_angle)
        data.log("0.85:turn_robot_in_place:current_gyro_angle:",gyro_sensor.angle())
        data.log("0.85:turn_robot_in_place:angle_we_want:",angle_we_want)
    
    while( current_gyro_angle <angle_we_want*0.99):
        #
        left_motor.run(100*left_direction)
        right_motor.run(100*right_direction)
        wait(10)    
        current_gyro_angle= abs(gyro_sensor.angle())
        print("turn_robot_in_place::gyro angle 99%: ", current_gyro_angle)
        data.log("0.99:turn_robot_in_place:current_gyro_angle:",gyro_sensor.angle())
        data.log("0.99:turn_robot_in_place:angle_we_want:",angle_we_want)

    left_motor.stop()
    right_motor.stop()

def turn_robot(direction,angle):
    current_gyro_angle=abs(gyro_sensor.angle())

    print("gyro angle start: ", gyro_sensor.angle())
    #gyro_sensor.reset_angle(0)
    #print("gyro angle reset: ", gyro_sensor.angle())
    print("turning motor: ")

    motor = left_motor

    if(direction=="RIGHT_BACKWARD"):
        motor = left_motor
        direction = 1
    if(direction=="RIGHT_FORWARD"):
        motor = left_motor
        direction=-1
    if(direction=="LEFT_BACKWARD"):
        motor = right_motor
        direction = 1
    if(direction=="LEFT_FORWARD"):
        motor = right_motor 
        direction=-1
    
    angle_we_want=angle+current_gyro_angle
    #robot.turn(45)
    while( current_gyro_angle <angle_we_want*0.90):
        #
        motor.run(direction*200)
        current_gyro_angle= abs(gyro_sensor.angle())
        print("gyro angle end1 : ", current_gyro_angle)
        wait(10)
    
    #motor.hold()
    wait(10)
    current_gyro_angle= abs(gyro_sensor.angle())
        
    while( current_gyro_angle <angle_we_want*0.99):
        #
        motor.run(direction*200)
        current_gyro_angle= abs(gyro_sensor.angle())
        print("gyro angle end2 : ", current_gyro_angle)
        wait(10)
    
    motor.stop()
        
def follow_line(line_color_sensor,distance,direction):
    PROPORTIONAL_GAIN = 1.2
    BLACK = 9
    WHITE = 85
    threshold = (BLACK + WHITE) / 2
    global start_gyro_value

    distance_travelled = 0
    direction_multiplier = -1
    if(direction=="BACKWARDS"):
        direction_multiplier = 1

    DRIVE_SPEED = direction_multiplier*200

    data.log("Follow black line")
    while distance_travelled<distance:
        # Calculate the deviation from the threshold.
        deviation = line_color_sensor.reflection() - threshold
        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation
        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)
        # You can wait for a short time or do other things in this loop.
        distance_travelled  = abs(robot.distance())
        print("distance travelled:",distance_travelled)
        data.log("distance travelled:",distance_travelled)
        gyro_angle = gyro_sensor.angle()
        
        if(start_gyro_value==0 and gyro_angle!=0):
            start_gyro_value = gyro_angle
            data.log("start gyro value:",gyro_angle)

        data.log("gyro value:",gyro_angle)
        wait(10)
    # while distance_travelled<distance:
    #     # Calculate the deviation from the threshold.
    #     deviation = line_color_sensor.reflection() - threshold
    #     # Calculate the turn rate.
    #     turn_rate = PROPORTIONAL_GAIN * deviation
    #     # Set the drive base speed and turn rate.
    #     robot.drive(DRIVE_SPEED, turn_rate)
    #     # You can wait for a short time or do other things in this loop.
    #     distance_travelled  = abs(robot.distance())
    #     print("distance travelled:",distance_travelled)
    #     wait(10)

def drive_straight_with_gyro:
    # drive for 10 ms
    # check gyro 
    #if angle has changed
    # drive with adjested angle

    while distance_travelled<distance:
        # Calculate the deviation from the threshold.
        deviation = line_color_sensor.reflection() - threshold
        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation
        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)
        # You can wait for a short time or do other things in this loop.
        distance_travelled  = abs(robot.distance())
        print("distance travelled:",distance_travelled)
        data.log("distance travelled:",distance_travelled)
        gyro_angle = gyro_sensor.angle()
        
        if(start_gyro_value==0 and gyro_angle!=0):
            start_gyro_value = gyro_angle
            data.log("start gyro value:",gyro_angle)

        data.log("gyro value:",gyro_angle)
        wait(10)
    return
#################

#################runs

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

def jason_step_counter():
    #turn_robot_in_place("RIGHT_BACKWARD",90)
    #go forwrd fast
    robot.settings(straight_speed=400, straight_acceleration=70, turn_rate=10, turn_acceleration=10)
    robot.straight(-1016)
    ev3.speaker.beep()
    robot.stop()

    # # push step counter - super slow to blue 
    robot.settings(straight_speed=30, straight_acceleration=20, turn_rate=10, turn_acceleration=10)
    robot.straight(-280.4)
    robot.stop()

    # #step back from the step counter 
    robot.settings(straight_speed=150, straight_acceleration=50, turn_rate=10, turn_acceleration=10)
    robot.straight(50)
    robot.stop()

    # #turn left 
    turn_robot_in_place("RIGHT_BACKWARD",90)
    #drive forward
    robot.straight(300)

    robot.straight(-800)

    robot.stop()

def sophie_run():
    sophie_run_basket_ball()

def sophie_run_basket_ball():
    # basketball- 27 in. forward - <3 90 degree turn left - inches forward - (-30 mm) turn - raise bar() -lower bar() - turn 30 degrees- lift bar(baccia) -   
    #drive straight
    #robot.settings(straight_speed=450, straight_acceleration=150, turn_rate=10, turn_acceleration=10)
    #robot.straight(-685.8)
    #robot.stop()

    #robot.settings(straight_speed=150, straight_acceleration=50, turn_rate=100, turn_acceleration=150)


    #turn right forwards 

    #turn 90 to face the 
    turn_robot_in_place("LEFT_FORWARD",90)    
        
        
        
    wait(10)
    robot.stop()

def jolene_run():
    #go forwrd fast
    # robot.settings(straight_speed=400, straight_acceleration=100, turn_rate=10, turn_acceleration=10)
    # robot.straight(-1016)
    # ev3.speaker.beep()
    # robot.stop()

    # # push step counter - super slow to blue 
    # robot.settings(straight_speed=30, straight_acceleration=20, turn_rate=10, turn_acceleration=10)
    # robot.straight(-370.4)
    # robot.stop()

    # #step back from the step counter 
    # robot.settings(straight_speed=150, straight_acceleration=50, turn_rate=10, turn_acceleration=10)
    # robot.straight(306.4)
    # robot.stop()

    # #turn left 
    # robot.turn(30)
    # robot.stop()
    #drive straight
    # robot.settings(straight_speed=150, straight_acceleration=50, turn_rate=10, turn_acceleration=10)
    # robot.straight(-595)
    # robot.stop()
    # #turn right
    # robot.turn(-30)
    # robot.stop()
    #drive upto treadmill
    #go staright till wtge black linwe
    
    #get gyro value
    global start_gyro_value

    robot.settings(straight_speed=400, straight_acceleration=50, turn_rate=10, turn_acceleration=10)
    robot.straight(-150)
    robot.stop()
    gyro_sensor.reset_angle(0)
    start_gyro_value = gyro_sensor.angle()
    print("start gyro value: ",start_gyro_value)
    data.log("start_gyro_value",start_gyro_value)


    follow_line(line_sensor_right,1775,"FORWARD")
    
    #get the gyro value to see where we are
    end_gyro_value =  gyro_sensor.angle()
    
    robot.stop()
    data.log("start gyro_alue",start_gyro_value)
    data.log("end gyro value",end_gyro_value)

    ev3.speaker.beep()
   
    
    #strIGHTEN THE ROBOT
    gyro_angle_diff = end_gyro_value - start_gyro_value
    direction = "LEFT_FORWARD"
    if(gyro_angle_diff<0):
        direction="RIGHT_FORWARD"

    data.log("gyro angle diff ",gyro_angle_diff)
    data.log("direction",direction)

    turn_robot_in_place(direction,abs(gyro_angle_diff))
    robot.stop()
    #ROBOT GO FORWARD
    robot.straight(-160)
    robot.stop()
    #right motor turn
    frnt_right_motor.run_target(5000, 3500)
    robot.stop()
    #going backward 
    robot.straight(150)
    robot.stop()
    #turn obot 35 degrees
    turn_robot_left_in_place(28)
    robot.stop()
    #go torwards row machine
    robot.straight(-260)
    robot.stop()
    #bring down the hook
    frnt_left_motor.run_target(500, -1100)
    robot.straight(150)
    robot.stop()
    turn_robot_left_in_place(30)
    robot.straight(60)

def jolene_test():
    #follow_line(line_sensor_right,1450,"FORWARD")
    #frnt_right_motor.run_target(500, 3,468)
    frnt_left_motor.run_target(500, -900)
###############


def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.
    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)

def gyro_test():
    #calibrate_gyro_offset()

    #while True:
    #    gyro_value = gyro_sensor.angle()
    #    print("gyro value: ",gyro_value)
    robot.settings(straight_speed=400, straight_acceleration=50, turn_rate=10, turn_acceleration=10)
    robot.straight(-150)
    robot.stop()
    start_gyro_value = gyro_sensor.angle()
    print("start gyro value: ",start_gyro_value)
    turn_robot_in_place("LEFT_FORWARD",90)
    gyro_value = gyro_sensor.angle()
    print("gyro value: ",gyro_value)



#start ############################################################ 
GYRO_CALIBRATION_LOOP_COUNT = 20
GYRO_OFFSET_FACTOR = 0.0005

def calibrate_gyro_offset():
    global gyro_offset

    gyro_sensor.reset_angle(0)
    # Calibrate the gyro offset. This makes sure that the robot is perfectly
    # still by making sure that the measured rate does not fluctuate more than
    # 2 deg/s. Gyro drift can cause the rate to be non-zero even when the robot
    # is not moving, so we save that value for use later.
    while True:
        gyro_minimum_rate, gyro_maximum_rate = 440, -440
        gyro_sum = 0
        for iteration in range(GYRO_CALIBRATION_LOOP_COUNT):
            gyro_sensor_value = gyro_sensor.speed()
            gyro_sum += gyro_sensor_value
            if gyro_sensor_value > gyro_maximum_rate:
                gyro_maximum_rate = gyro_sensor_value
            if gyro_sensor_value < gyro_minimum_rate:
                gyro_minimum_rate = gyro_sensor_value
            log.debug("Calibrating gyro iteration: {:2d}, gyro_sum: {:4.2f}, gyro_minimum_rate: {:4.2f}, gyro_maximum_rate: {:4.2f}".format(
            iteration, gyro_sum, gyro_minimum_rate, gyro_maximum_rate))
            wait(5)
        if gyro_maximum_rate - gyro_minimum_rate < 2:
            break
    gyro_offset = gyro_sum / GYRO_CALIBRATION_LOOP_COUNT
    print("gyro_offset:",gyro_offset)
   


######## MAIN PROGRAM ##########

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.C)
right_motor = Motor(Port.D)

frnt_left_motor = Motor(Port.B) 
frnt_right_motor = Motor(Port.A) 

gyro_sensor = GyroSensor(Port.S3)
color_sensor = ColorSensor(Port.S1)
line_sensor_left = ColorSensor(Port.S4)
line_sensor_right = ColorSensor(Port.S2)



gyro_sensor.reset_angle(0)

#calibrate_gyro_offset()

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


# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=53, axle_track=120)

#do the runs

#data = DataLog('color')
print('Hello Gen R !')


color = color_sensor.color()
print(color)


#data.log(color)
#jolene_run()

if(color==Color.WHITE):
    print("Jolene Run")
    jolene_run()
    #jolene_test()
elif(color==Color.BLUE):
    print("Jason Run")
    jason_run()
#
#jolene_test()
elif(color==Color.RED):
    print("Sophie Run")
    sophie_run()

else:
    gyro_test()
    #jolene_run()

# Turn clockwise by 360 degrees and back again.
# robot.turn(360)
# ev3.speaker.beep()

# robot.turn(-360)
# ev3.speaker.beep()

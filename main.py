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
from threading import Thread
import sys

# Logging
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(levelname)5s: %(message)s')
log = logging.getLogger(__name__)
log.info("Starting Program")

data = DataLog('time', 'angle', name='my_file', timestamp=False, extension='txt')
start_gyro_value =0

#region robot functions
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
    start_angle = current_gyro_angle
    end_angle = current_gyro_angle + angle * angle_direction

    
    
   
    print("turn_robot_in_place::start_angle: ",start_angle) 
    print("turn_robot_in_place::end_angle: ",end_angle)
 
    data.log("turn_robot_in_place::start_angle: ",start_angle) 
    data.log("turn_robot_in_place::end_angle: ",end_angle)
    
    

    current_abs_angle=0

    while( current_abs_angle <angle*0.85):
        #
        left_motor.run(200*left_direction)
        right_motor.run(200*right_direction)
        wait(10)
        current_gyro_angle= gyro_sensor.angle()
        current_abs_angle = abs(start_angle - current_gyro_angle)
        print("turn_robot_in_place::gyro angle 85%: ", current_gyro_angle)
        data.log("0.85:turn_robot_in_place:current_gyro_angle:",gyro_sensor.angle())
        data.log("0.85:turn_robot_in_place:current_abs_angle:",current_abs_angle)
    
    while( current_abs_angle <angle*0.99):
        #
        left_motor.run(100*left_direction)
        right_motor.run(100*right_direction)
        wait(10)    
        current_gyro_angle= gyro_sensor.angle()
        current_abs_angle = abs(start_angle - current_gyro_angle)
        print("turn_robot_in_place::gyro angle 99%: ", current_gyro_angle)
        data.log("0.99:turn_robot_in_place:current_gyro_angle:",gyro_sensor.angle())
        data.log("0.99:turn_robot_in_place:current_abs_angle:",current_abs_angle)

    data.log("turn_robot_in_place:done",current_abs_angle)

    left_motor.stop()
    right_motor.stop()

def gyro_straighten_robot(start_gyro_value):
    #strIGHTEN THE ROBOT
    end_gyro_value =  gyro_sensor.angle()
    gyro_angle_diff = end_gyro_value - start_gyro_value
    direction = "LEFT"
    if(gyro_angle_diff<0):
        direction="RIGHT"

    data.log("gyro_straighten_robot::gyro angle diff ",gyro_angle_diff)
    data.log("gyro_straighten_robot::direction",direction)

    turn_robot_in_place(direction,abs(gyro_angle_diff))

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
    current_gyro_angle= abs(gyro_sesor.angle())
        
    while( current_gyro_angle <angle_we_want*0.99):
        #n
        motor.run(direction*200)
        current_gyro_angle= abs(gyro_sensor.angle())
        print("gyro angle end2 : ", current_gyro_angle)
        wait(10)
    
    motor.stop()
        
def follow_line(line_color_sensor,distance,direction,speed):
    PROPORTIONAL_GAIN = 1.2
    BLACK = 9
    WHITE = 85
    threshold = (BLACK + WHITE) / 2
    global start_gyro_value

    distance_travelled = 0
    direction_multiplier = -1

    
    if(direction=="BACKWARD"):
        direction_multiplier = 1
     
    drive_speed= 200   
    if(speed == "SUPERDUPERFAST"):
        drive_speed=400
    elif(speed == "SUPERFAST"):
        drive_speed =300
    elif(speed == "MEDIUMFAST"):
        drive_speed =250
    elif(speed=="FAST"):
        drive_speed =200
    elif(speed=="NORMAL"):
        drive_speed =150
    elif(speed=="SLOW"):
        drive_speed =100
        PROPORTIONAL_GAIN =1.1
    elif(speed=="SUPERSLOW"):
        drive_speed =50
        PROPORTIONAL_GAIN =1.05


    DRIVE_SPEED = direction_multiplier*drive_speed

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
    return
    
def drive_straight_with_gyro(distance,direction="FORWARD"):
    # drive for 10 ms
    # check gyro 
    #if angle has changed
    # drive with adjested angle
    PROPORTIONAL_GAIN = 1.2
    distance_travelled = 0

    direction_multiplier = -1
    if(direction=="BACKWARDS"):
        direction_multiplier = 1

    gyro_angle = gyro_sensor.angle()
    threshold = gyro_angle
    DRIVE_SPEED = direction_multiplier*200

    while distance_travelled<distance:
        # Calculate the deviation from the threshold.
        deviation = gyro_sensor.angle() - threshold
        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation
        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)
        # You can wait for a short time or do other things in this loop.
        distance_travelled  = abs(robot.distance())
        print("distance travelled:",distance_travelled)
        data.log("distance travelled:",distance_travelled)

        data.log("gyro value:",gyro_angle)
        print("gyro value:",gyro_angle)
        wait(10)
    return
#endregion

#region runs
# region jason's runs


def jason_start_slide():
    #turn 30 left
    turn_robot_left_in_place(25)
    #go straight for slide
    robot.settings(straight_speed=800, straight_acceleration=100, turn_rate=10, turn_acceleration=10)
    #ev3.speaker.beep()
    robot.straight(-1002)
    robot.stop()
    #ev3.speaker.beep()    #robot.settings(straight_speed=400, straight_acceleration=100, turn_rate=10, turn_acceleration=10)
    robot.settings(straight_speed=1400, straight_acceleration=100, turn_rate=10, turn_acceleration=10)
    robot.straight(800)
    robot.stop()
    turn_robot_right_in_place(25)
    #turn_robot_in_place("LEFT",45)
    robot.stop()
    robot.settings(straight_speed=2400, straight_acceleration=100, turn_rate=10, turn_acceleration=10)
    
    robot.straight(500)
    robot.stop()



def jason_step_counter():
    #turn_robot_in_place("RIGHT_BACKWARD",90)
    #go forwrd fast
    robot.settings(straight_speed=1400, straight_acceleration=350, turn_rate=10, turn_acceleration=10)
    robot.straight(-966)
    #ev3.speaker.beep()
    robot.stop()

    # # push step counter - super slow to blue 
    robot.settings(straight_speed=40, straight_acceleration=20, turn_rate=10, turn_acceleration=10)
    robot.straight(-280.4)
    robot.stop()

    # #step back from the step counter 
    robot.settings(straight_speed=1500, straight_acceleration=350, turn_rate=10, turn_acceleration=10)
    robot.straight(45)
    robot.stop()

    # #turn left 
    turn_robot_in_place("LEFT",90)
    robot.stop() 
    #drive backwards to square up
    robot.straight(300)
    #go under the pull-up bar
    robot.straight(-800)
    robot.stop()
    #turn towards the dance floor
    turn_robot_in_place("LEFT",40)
    #go onto the dance floor
    robot.straight(-500)
    robot.stop()
    #dance
    current_try=0
    number_tries=5
    while(current_try<number_tries):
        turn_robot_in_place("LEFT",45)
        robot.stop()
        turn_robot_in_place("RIGHT",45)
        current_try=current_try+1

#endregion
#region sophies runs
def sophie_run():
    sophie_run_basket_ball()

def beep():
    ev3.speaker.beep()
    
def front_motor_down(speed,position):
    frnt_left_motor.run_target(speed,position)
def sophie_run_basket_ball():
    # basketball- 27 in. forward - <3 90 degree turn left - inches forward - (-30 mm) turn - raise bar() -lower bar() - turn 30 degrees- lift bar(baccia) -   
    #drive straight
    #robot.settings(straight_speed=450, straight_acceleration=150, turn_rate=10, turn_acceleration=10)
    #robot.straight(-685.8)
    #robot.stop()

    #robot.settings(straight_speed=150, straight_acceleration=50, turn_rate=100, turn_acceleration=150)
    print("Sophie Basket ball Run")  
    robot.settings(straight_speed=1400, straight_acceleration=150, turn_rate=10, turn_acceleration=10)
    
    #drive fwd
    #drive_straight_with_gyro(-450)
    robot.straight(-375)
    robot.stop()
    print("Follow line")  
    
    data.log("Following line at fast speed")  
    follow_line(line_sensor_right,200,"FORWARD","FAST")
    robot.stop()
    t=Thread(target=beep).start()
   
    #turn_robot_in_place("LEFT",30)
    #follow_line(line_sensor_right,975,"FORWARD","SUPERSLOW")
    data.log("Following line at slow speed")  
    follow_line(line_sensor_right,575,"FORWARD","SUPERSLOW")
    t=Thread(target=beep).start()
    
    data.log("Following line at normal speed")  
    follow_line(line_sensor_right,875,"FORWARD","SUPERFAST")
    robot.stop()
    follow_line(line_sensor_right,975,"FORWARD","SLOW")
    robot.stop()

    #turn robot to face the basketball mission
    data.log("turning robot to 20")  
    turn_robot_in_place("LEFT",20)
    robot.stop()
    robot.straight(-245)
    robot.stop()
    #go up
    frnt_left_motor.run_target(1000, 5800)
    robot.stop()
    #go down
    
    Thread(target=front_motor_down,args=(1500,2800)).start()
    
    wait(300)
    
    #go back a little
    robot.straight(115)
    robot.stop()
    turn_robot_in_place("RIGHT",78)
    # go forward a little
    # robot.straight(-55)
    # robot.stop()

    # frnt_left_motor.run_target(1000, 3000)
    # robot.stop()

    # turn_robot_in_place("RIGHT",70)
    robot.straight(-415)
    robot.stop()

    frnt_right_motor.run_target(1500, -200)
    robot.stop()

    robot.settings(straight_speed=1400, straight_acceleration=350, turn_rate=10, turn_acceleration=10)
    robot.straight(415)
    robot.stop()
    turn_robot_in_place("LEFT",15)
    robot.straight(1175)
    robot.stop()
    
def sophie_bench():
    #drive straight
     #go forwrd fast
    robot.settings(straight_speed=1400, straight_acceleration=350, turn_rate=10, turn_acceleration=10)
    robot.straight(-400)
    robot.stop()
    robot.settings(straight_speed=400, straight_acceleration=350, turn_rate=10, turn_acceleration=10)
    robot.straight(-450)
    #ev3.speaker.beep()
    robot.stop()
    robot.settings(straight_speed=1400, straight_acceleration=350, turn_rate=10, turn_acceleration=10)
    robot.straight(600)
    robot.stop()

    
    
    
#endregion
#region  jolene's run
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

    robot.settings(straight_speed=500, straight_acceleration=500, turn_rate=10, turn_acceleration=10)
    robot.straight(-150)
    robot.stop()
    gyro_sensor.reset_angle(0)
    start_gyro_value = gyro_sensor.angle()
    print("start gyro value: ",start_gyro_value)
    data.log("start_gyro_value",start_gyro_value)

    follow_line(line_sensor_right,475,"FORWARD","SUPERFAST")    # follow_line(line_sensor_right,1575,"FORWARD","SUPERFAST")
    ev3.speaker.beep()
    # wait(2)
    # robot.stop()
    # follow_line(line_sensor_right,1775,"FORWARD","NORMAL")
    # robot.settings(straight_speed=600, straight_acceleration=50, turn_rate=10, turn_acceleration=10)
    # robot.straight(-1975)
    # wait(2)
    # robot.stop()
    robot.stop()
    follow_line(line_sensor_right,700,"FORWARD","FAST")
    ev3.speaker.beep()
    robot.stop()
    follow_line(line_sensor_right,1275,"FORWARD","SUPERFAST")
    ev3.speaker.beep()
    robot.stop()
    follow_line(line_sensor_right,1500,"FORWARD","FAST") 
    ev3.speaker.beep()
    robot.stop()
    follow_line(line_sensor_right,1750,"FORWARD","MEDIUMFAST") 
    robot.stop()
    ev3.speaker.beep()
    follow_line(line_sensor_right,1950,"FORWARD","SUPERSLOW") 
    robot.stop()
    ev3.speaker.beep()
    #get the gyro value to see where we are
    end_gyro_value =  gyro_sensor.angle()
    
    robot.stop()
    data.log("start gyro_alue",start_gyro_value)
    data.log("end gyro value",end_gyro_value)

   
    
    
    #straighten THE ROBOT
    gyro_angle_diff = end_gyro_value - start_gyro_value
    direction = "RIGHT"
    if(gyro_angle_diff<0):
        direction="LEFT"

    data.log("gyro angle diff ",gyro_angle_diff)
    data.log("direction",direction)

    #turn_robot_in_place("direction",abs(gyro_angle_diff))
    #turn_robot_in_place("LEFT",5)
    #ev3.speaker.beep()
    
    robot.settings(straight_speed=350, straight_acceleration=50, turn_rate=10, turn_acceleration=10)
    #ROBOT GO FORWARD
    #robot.straight(-50)
    #robot.stop()?
    #right motor turn
    frnt_right_motor.run_target(5000, 1600)
    robot.stop()
    robot.settings(straight_speed=500, straight_acceleration=100, turn_rate=10, turn_acceleration=10)
    #going backward 
    robot.straight(80)
    robot.stop()
    #turn obot 35 degrees
    turn_robot_left_in_place(24)
    robot.stop()
    #go torwards row machine
    robot.straight(-220)
    robot.stop()
    #bring down the hook
    frnt_left_motor.run_target(1500, -1000)
    robot.straight(140)
    robot.stop()
    # turn_robot_left_in_place(30)
    turn_robot_in_place("LEFT",20)
    robot.straight(-60)
    robot.stop()
    #bring up the hook
    Thread(target=front_motor_down,args=(1500,100)).start()


    #frnt_left_motor.run_target(1500, 100)
    # start going back
    robot.straight(55)
    robot.stop()
   
    turn_robot_in_place("RIGHT",27)
    
    robot.settings(straight_speed=5000, straight_acceleration=1000, turn_rate=10, turn_acceleration=10)
    
    robot.straight(200)
    robot.stop()

    turn_robot_in_place("RIGHT",19)
    robot.settings(straight_speed=5000, straight_acceleration=1000, turn_rate=10, turn_acceleration=10)
    robot.straight(2200)
    robot.stop()
#endregion
#endregion
#region tests 

def jolene_test():
    #follow_line(line_sensor_right,1450,"FORWARD")
    #frnt_right_motor.run_target(500, 3,468)
    frnt_left_motor.run_target(500, -900)

def front_motor_test():
    frnt_right_motor.run_target(5000, 2000)
  


def gyro_test():
    #calibrate_gyro_offset()
    turn_robot_in_place("LEFT_FORWARD",90)
    #drive_straight_with_gyro(1300)

    #while True:
    #    gyro_value = gyro_sensor.angle()
    #    print("gyro value: ",gyro_value)
    # robot.settings(straight_speed=400, straight_acceleration=50, turn_rate=10, turn_acceleration=10)
    # robot.straight(-150)
    # robot.stop()
    # start_gyro_value = gyro_sensor.angle()
    # print("start gyro value: ",start_gyro_value)
    # turn_robot_in_place("LEFT_FORWARD",90)
    # gyro_value = gyro_sensor.angle()
    # print("gyro value: ",gyro_value)

#endregion

#region calibration

def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.
    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)




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
   
#endregion

######## MAIN PROGRAM ##########

#region  main

def reset_robot_motors_and_gyro():
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    gyro_sensor.reset_angle(0)  
    frnt_left_motor.reset_angle(0)
    frnt_right_motor.reset_angle(0)
    robot.reset()

def stop_motors():
    robot.stop()
   
    
    frnt_left_motor.stop()
    frnt_right_motor.stop()
   

def run_on_color():
    color_sensor = ColorSensor(Port.S1)
    color = color_sensor.color()
    print(color)
    data.log('Color detected',color)

    rgb_value = color_sensor.rgb()
    print('Color detected RGB',rgb_value)
    data.log('Color detected RGB',rgb_value)

    ambient = color_sensor.ambient()
    print('Color detected ambient',ambient)
    data.log('Color detected ambient',ambient)

    reflection = color_sensor.reflection()
    print('Color detected reflection',reflection)
    data.log('Color detected reflection',reflection)
    
    #reset gyro before start of program
    gyro_sensor.reset_angle(0)

    if(color==Color.WHITE):
        ev3.speaker.say('white')
        print("Jolene Run")
        jolene_run()
        #jolene_test()
    elif(color==Color.BLUE):
        #ev3.speaker.say('blue')
        print("Jason Run")
        jason_start_slide()
    #
    #jolene_test()
    elif(color==Color.RED):
        #ev3.speaker.say('red')
        print("Sophie Run")  
        sophie_run()

    elif(color==Color.BLACK):
        jason_step_counter()

    elif(color==Color.GREEN):
        sophie_bench()

    else:
        rgb_value = color_sensor.rgb()
        print(rgb_value)
        #gyro_test()
        jolene_run()
        #jason_run()


#region initialize motors an brick
# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.C)
right_motor = Motor(Port.D)

frnt_left_motor = Motor(Port.B) 
frnt_right_motor = Motor(Port.A) 

gyro_sensor = GyroSensor(Port.S3)

line_sensor_left = ColorSensor(Port.S4)
line_sensor_right = ColorSensor(Port.S2)

left_motor.reset_angle(0)
right_motor.reset_angle(0)
gyro_sensor.reset_angle(0)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=53, axle_track=120)


#endregion





# # Actions will be used to change which way the robot drives.
# Action = namedtuple('Action ', ['drive_speed', 'steering'])

# # These are the pre-defined actions
# STOP = Action(drive_speed=0, steering=0)
# FORWARD_FAST = Action(drive_speed=150, steering=0)
# FORWARD_SLOW = Action(drive_speed=40, steering=0)
# BACKWARD_FAST = Action(drive_speed=-75, steering=0)
# BACKWARD_SLOW = Action(drive_speed=-10, steering=0)
# TURN_RIGHT = Action(drive_speed=0, steering=70)
# TURN_LEFT = Action(drive_speed=0, steering=-70)




#do the runs

#data = DataLog('color')
print('Hello Gen R !')
ev3.light.on(Color.RED)

# cancel_button = false
no_of_times_up_pressed = 0
no_of_times_down_pressed = 0
t=Thread(target=sophie_run_basket_ball)

while True:
    center_pressed = Button.CENTER in ev3.buttons.pressed()
    up_pressed =  Button.UP in ev3.buttons.pressed()
    down_pressed =  Button.DOWN in ev3.buttons.pressed()
    left_pressed =  Button.LEFT in ev3.buttons.pressed()

    if center_pressed:
        print("center button pressed",center_pressed)
        center_pressed=False
        #front_motor_test()
        #stop_motors()
        reset_robot_motors_and_gyro()
        Thread(target=front_motor_down,args=(1500,200)).start()
        #t=Thread(target=front_motor_test)
        
        wait(1000)
    elif up_pressed:
        print("up button pressed",up_pressed)
        up_pressed=False
        no_of_times_up_pressed = no_of_times_up_pressed +1
        reset_robot_motors_and_gyro()

        if(no_of_times_up_pressed==1):
            print("sophie_bsktball",up_pressed)
            t=Thread(target=sophie_run_basket_ball)
            t.start()
        else:
            print("sophie_bench",up_pressed)
            sophie_bench()
        wait(1000)

    elif down_pressed:
        print("down button pressed",down_pressed)
        down_pressed=False
        no_of_times_down_pressed = no_of_times_down_pressed +1
        reset_robot_motors_and_gyro()
        
        if(no_of_times_down_pressed==1):
            print("jason_start_slide",up_pressed)
            jason_start_slide()
        else:
            print("jason_step_counter",up_pressed)
            jason_step_counter()
        wait(1000)

    elif left_pressed:
        print("left button pressed",left_pressed)
        left_pressed=False
        reset_robot_motors_and_gyro()
        print("jolene run",up_pressed)
        #front_motor_test()
        t=Thread(target=jolene_run)
        t.start()
        
        wait(1000)


#endregion

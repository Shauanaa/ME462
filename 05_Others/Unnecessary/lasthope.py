import time
import sys
import select
import stepper
from machine import Pin, Timer
from usys import stdin
from uselect import poll


def print_air_hockey_table():
    table = """
    +-----------------------------------------+
    |                   .                     |
    |                                         |
    |                   |                     |
    |                   |                     |
    |                   |                     |
    |                   |                     |
    |                   |                     |
    |                   |                     |
    |                   |                     |
    |                   |                     |
    |                   |                     |
    |                   |                     |
    |                   |                     |
    |                   .                     |
    +-----------------------------------------+
    """
    print(table)
    
def print_instructions():
    instructions = """
The numbers are:
1 for puck disturber, which has an end effector to catch the puck.
2 for the wall motor, which has a wall with a soft material and becomes on and off.
3 for bringing the flipper into the game.
4 for turning flipper on.
9 for stopping the current process after it finishes.
Ctrl + C for interrupting the current process.
"""
    print(instructions)
    
def repl():
    print("*** WELCOME TO AIR HOCKEY ***")
    print_air_hockey_table()
    instructions()
    try:
        while True:
            command = input(">>> ")
            if command == 'exit':
                break
            else:
                sys.stdin = StringIO(command)
                controller.run()
    except KeyboardInterrupt:
        print("\nExiting REPL.")
    finally:
        sys.stdin = sys.__stdin__
        
# Define the stepper motor pins
SM_IN1 = 10
SM_IN2 = 11
SM_IN3 = 12
SM_IN4 = 13

SP_IN1 = 6
SP_IN2 = 7
SP_IN3 = 8
SP_IN4 = 9

PM_IN1 = 2
PM_IN2 = 3
PM_IN3 = 4
PM_IN4 = 5

SPIO_IN1 = 18
SPIO_IN2 = 19
SPIO_IN3 = 20
SPIO_IN4 = 21

# Initialize the stepper motors
puck_motor_stepper_motor = stepper.HalfStepMotor.frompins(PM_IN1, PM_IN2, PM_IN3, PM_IN4)
puck_motor_stepper_motor.reset()
puck_motor_working = False

sponge_motor_stepper_motor = stepper.HalfStepMotor.frompins(SM_IN1, SM_IN2, SM_IN3, SM_IN4)
sponge_motor_stepper_motor.reset()
sponge_motor_working = False

spio_motor_stepper_motor = stepper.HalfStepMotor.frompins(SPIO_IN1, SPIO_IN2, SPIO_IN3, SPIO_IN4)
spio_motor_stepper_motor.reset()
spio_motor_working = False

spin_motor_stepper_motor = stepper.HalfStepMotor.frompins(SP_IN1, SP_IN2, SP_IN3, SP_IN4)
spin_motor_stepper_motor.reset()
spin_motor_working = False

# Create global poll_obj
poll_obj = select.poll()
poll_obj.register(sys.stdin, select.POLLIN)

try:
    while True:
        res = poll_obj.poll()
        ch = res[0][0].read(1)
        print("in first while")
        if ch == '1':
            print("in if")
            puck_motor_working = True
            while puck_motor_working:
                print("in while")
                puck_motor_stepper_motor.step_until_angle(20)
                puck_motor_stepper_motor.step_until_angle(0)
                print("after ch1")
                if ch1 == '9':
                    print("in ch1")
                    puck_motor_working = False
                    break
                
        elif ch == '2':
            sponge_motor_working = True
            while sponge_motor_working:
                sponge_motor_stepper_motor.step_until_angle(30)
                sponge_motor_stepper_motor.step_until_angle(0)
                res2 = poll_obj.poll()
                ch2 = res2[0][0].read(1)
                if ch2 == '9':
                    sponge_motor_working = False
                    break
                
        elif ch == '3':
            spin_motor_working = True
            while spin_motor_working:
                spin_motor_stepper_motor.step_until_angle(30)
                spin_motor_stepper_motor.step_until_angle(0)
                res3 = poll_obj.poll()
                ch3 = res3[0][0].read(1)
                if ch3 == '9':
                    spin_motor_working = False
                    break
                
        elif ch == '4':
            spio_motor_working = True
            while spio_motor_working:
                spio_motor_stepper_motor.step_until_angle(30)
                spio_motor_stepper_motor.step_until_angle(0)
                res4 = poll_obj.poll()
                ch4 = res4[0][0].read(1)
                if ch4 == '9':
                    spio_motor_working = False
                    break
                
        elif ch == '12':
            puck_motor_working = True
            sponge_motor_working = True
            while puck_motor_working and sponge_motor_working:
                puck_motor_stepper_motor.step_until_angle(30)
                puck_motor_stepper_motor.step_until_angle(0)
                
                sponge_motor_stepper_motor.step_until_angle(30)
                sponge_motor_stepper_motor.step_until_angle(0)
                
                res5 = poll_obj.poll()
                ch5 = res5[0][0].read(1)
                if ch5 == '9':
                    puck_motor_working = False
                    sponge_motor_working = False
                    break

        elif ch == '13':
            puck_motor_working = True
            spin_motor_working = True
            while puck_motor_working and spin_motor_working:
                puck_motor_stepper_motor.step_until_angle(30)
                puck_motor_stepper_motor.step_until_angle(0)
                
                spin_motor_stepper_motor.step_until_angle(30)
                spin_motor_stepper_motor.step_until_angle(0)
                
                res6 = poll_obj.poll()
                ch6 = res6[0][0].read(1)
                if ch6 == '9':
                    puck_motor_working = False
                    spin_motor_working = False
                    break

        elif ch == '14':
            puck_motor_working = True
            spio_motor_working = True
            while puck_motor_working and spio_motor_working:
                puck_motor_stepper_motor.step_until_angle(30)
                puck_motor_stepper_motor.step_until_angle(0)
                
                spio_motor_stepper_motor.step_until_angle(30)
                spio_motor_stepper_motor.step_until_angle(0)
                
                res7 = poll_obj.poll()
                ch7 = res7[0][0].read(1)
                if ch7 == '9':
                    puck_motor_working = False
                    spio_motor_working = False
                    break

        elif ch == '23':
            sponge_motor_working = True
            spin_motor_working = True
            while sponge_motor_working and spin_motor_working:
                sponge_motor_stepper_motor.step_until_angle(30)
                sponge_motor_stepper_motor.step_until_angle(0)
                
                spin_motor_stepper_motor.step_until_angle(30)
                spin_motor_stepper_motor.step_until_angle(0)
                
                res8 = poll_obj.poll()
                ch8 = res8[0][0].read(1)
                if ch8 == '9':
                    sponge_motor_working = False
                    spin_motor_working = False
                    break

        elif ch == '34':
            spin_motor_working = True
            spio_motor_working = True
            while spin_motor_working and spio_motor_working:
                spin_motor_stepper_motor.step_until_angle(30)
                spin_motor_stepper_motor.step_until_angle(0)
                
                spio_motor_stepper_motor.step_until_angle(30)
                spio_motor_stepper_motor.step_until_angle(0)
                
                res9 = poll_obj.poll()
                ch9 = res9[0][0].read(1)
                if ch9 == '9':
                    spin_motor_working = False
                    spio_motor_working = False
                    break

        elif ch == '9':
            break
                
except KeyboardInterrupt:
    print("except works")


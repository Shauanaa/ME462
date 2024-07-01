import time
import board
import digitalio
import stepper

# Define the stepper motor pins
IN1 = board.GP2
IN2 = board.GP3
IN3 = board.GP4
IN4 = board.GP5

# Initialize the stepper motor
stepper_motor = stepper.HalfStepMotor.frompins(IN1, IN2, IN3, IN4)

# Set the current position as position 0
stepper_motor.reset()

try:
    while True:
        stepper_motor.step_until_angle(90)
        time.sleep(0.5)
        stepper_motor.step_until_angle(0)
        time.sleep(0.5)
    
except KeyboardInterrupt:
    print('Keyboard interrupt')

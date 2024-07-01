import time
import sys
import select
import stepper
from machine import Pin, Timer
from usys import stdin
from uselect import poll

class StepperMotorControl:
    def __init__(self):
        self.puck_motor = self.initialize_motor(10, 11, 12, 13)
        self.wall_motor = self.initialize_motor(2, 3, 4, 5)
        self.spin_motor = self.initialize_motor(6, 7, 8, 9)
        self.spio_motor = self.initialize_motor(18, 19, 20, 21)
        
        self.poll_obj = select.poll()
        self.poll_obj.register(sys.stdin, select.POLLIN)
        
    def initialize_motor(self, in1, in2, in3, in4):
        motor = stepper.HalfStepMotor.frompins(in1, in2, in3, in4)
        motor.reset()
        return motor

    def step_motor(self, motor, angle):
        motor.step_until_angle(angle)
        motor.step_until_angle(0)

    def stop_motor(self, motor):
        motor.reset()
    
    def control_motor(self, motor, working_flag):
        setattr(self, working_flag, True)
        while getattr(self, working_flag):
            self.step_motor(motor, 30)
            res = self.poll_obj.poll()
            ch = res[0][0].read(1)
            if ch == '9':
                setattr(self, working_flag, False)
                self.stop_motor(motor)
                break

    def control_dual_motors(self, motor1, motor2, working_flag1, working_flag2):
        setattr(self, working_flag1, True)
        setattr(self, working_flag2, True)
        while getattr(self, working_flag1) and getattr(self, working_flag2):
            self.step_motor(motor1, 30)
            self.step_motor(motor2, 30)
            res = self.poll_obj.poll()
            ch = res[0][0].read(1)
            if ch == '9':
                setattr(self, working_flag1, False)
                setattr(self, working_flag2, False)
                self.stop_motor(motor1)
                self.stop_motor(motor2)
                break

    def run(self):
        try:
            while True:
                res = self.poll_obj.poll()
                ch = res[0][0].read(1)
                if ch == '1':
                    self.control_motor(self.puck_motor, 'puck_motor_working')
                elif ch == '2':
                    self.control_motor(self.wall_motor, 'wall_motor_working')
                elif ch == '3':
                    self.control_motor(self.spin_motor, 'spin_motor_working')
                elif ch == '4':
                    self.control_motor(self.spio_motor, 'spio_motor_working')
                elif ch == '12':
                    self.control_dual_motors(self.puck_motor, self.wall_motor, 'puck_motor_working', 'wall_motor_working')
                elif ch == '13':
                    self.control_dual_motors(self.puck_motor, self.spin_motor, 'puck_motor_working', 'spin_motor_working')
                elif ch == '14':
                    self.control_dual_motors(self.puck_motor, self.spio_motor, 'puck_motor_working', 'spio_motor_working')
                elif ch == '23':
                    self.control_dual_motors(self.wall_motor, self.spin_motor, 'wall_motor_working', 'spin_motor_working')
                elif ch == '34':
                    self.control_dual_motors(self.spin_motor, self.spio_motor, 'spin_motor_working', 'spio_motor_working')
                elif ch == '9':
                    break
        except KeyboardInterrupt:
            print("Control interrupted by user.")
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
    
def repl():
    controller = StepperMotorControl()
    print("*** WELCOME TO AIR HOCKEY ***")
    print_air_hockey_table()
    print("The controlling mechanism of the disturbance mechanisms are as follows":)
    print("To power up the motors, you should enter the corresponding number for each motor.")

    print("Enter commands to control the motors. Type 'exit' to quit.")
    
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

if __name__ == "__main__":
    repl()



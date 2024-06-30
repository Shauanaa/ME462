import serial

class Commander:
    def __init__(self, serial_port:str="/dev/ttyUSB0", baudrate:int=115200):
        self._port = serial.Serial(serial_port, baudrate, timeout= 10)
        if not self._port.is_open:
            print(f"ERROR: Couldn't open port {serial_port}.")
            exit()

    def _constrain(self, val, min_val = -255, max_val = 255):
        return min(max_val, max(min_val, val))

    def send(self, agent_id:int, left_motor_power:int, right_motor_power:int, fan_power:int=0):
        send_str = f"{chr(ord('a') + agent_id)}{self._constrain(left_motor_power):+04}{self._constrain(right_motor_power):+04}{self._constrain(fan_power, 0):+04}\n"
        self._port.write(send_str.encode())

    def __del__(self):
        self._port.close()

def main():
    """
    Desk Fan Example using Python API
    """
    import time,math
    commander = Commander()
    theta = 0
    while True:
        mag = 80*math.sin(theta)
        commander.send(1, int(mag), int(-mag), 200)
        theta = theta + 0.15
        time.sleep(0.1)

if __name__ == "__main__":
    main()
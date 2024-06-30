import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int16MultiArray
import serial

class CommanderNode(Node):
    SERIAL_PORT = "/dev/ttyUSB0"
    subs = []
    def __init__(self):
        super().__init__("Commander")
        self.port = serial.Serial(self.SERIAL_PORT, 115200, timeout= 10)
        if not self.port.is_open:
            self.get_logger().error(f"Couldn't open port {self.SERIAL_PORT}.")
            rclpy.shutdown()
            exit()
        self.subs.append(self.create_subscription(Int16MultiArray, f"/agent_0/cmd", self.cmd_callback0, 1))
        self.subs.append(self.create_subscription(Int16MultiArray, f"/agent_1/cmd", self.cmd_callback1, 1))
        self.get_logger().info("Initialized!")

    def constrain(self, val:int, min_val = -255, max_val = 255):
        return min(max_val, max(min_val, val))

    def cmd_callback0(self, msg):
        if len(msg.data) != 3:
            self.get_logger().warning(f'Command should only have 3 elements, got {len(msg.data)}. Not sending the command.', throttle_duration_sec=1)
            return
        send_str = f"{chr(ord('a') + 1)}{self.constrain(msg.data[0]):+04}{self.constrain(msg.data[1]):+04}{self.constrain(msg.data[2], 0):+04}\n"
        self.port.write(send_str.encode())
        print(send_str)

    def cmd_callback1(self, msg):
        if len(msg.data) != 3:
            self.get_logger().warning(f'Command should only have 3 elements, got {len(msg.data)}. Not sending the command.', throttle_duration_sec=1)
            return
        send_str = f"{chr(ord('a') + 1)}{self.constrain(msg.data[0]):+04}{self.constrain(msg.data[1]):+04}{self.constrain(msg.data[2], 0):+04}\n"
        self.port.write(send_str.encode())
        print(send_str)

    def __del__(self):
        self.port.close()

def main(args=None):
    rclpy.init(args=args)
    node = CommanderNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
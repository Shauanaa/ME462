import time
import math
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int16MultiArray

class CommandExample(Node):
    """
    Desk Fan Example using ROS2
    """
    def __init__(self):
        super().__init__("Command_Example")
        self.pub = self.create_publisher(Int16MultiArray, "/agent_1/cmd", 1)
        self.get_logger().info("Initialized!")
        timer_period = 0.1 #seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.theta = 0
        self.fan_speed = 200

    def timer_callback(self):
        mag = 80*math.sin(self.theta)
        self.pub.publish(Int16MultiArray(data=[int(mag), int(-mag), self.fan_speed]))
        self.theta = self.theta + 0.15
    

def main(args=None):
    rclpy.init(args=args)
    node = CommandExample()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
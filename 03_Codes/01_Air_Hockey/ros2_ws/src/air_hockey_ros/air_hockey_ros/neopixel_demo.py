#!/home/peter/venv/bin/python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import UInt8MultiArray
import numpy as np
import time

class NeoPixelNode(Node):
    default_color = (0, 0, 180)
    
    def __init__(self):
        super().__init__("NeoPixel_Demo")
        # Grid size of used leds
        self.width = 28 # 27 unused, 55 total in a row
        self.height = 13 # number of rows all used
        
        self.pixel_pub = self.create_publisher(UInt8MultiArray, "/NeoPixel/array", 1)
        self.get_logger().info("Starting!")
        pos = 0
        while True:
            for thickness in range(1,7):
                grid = np.zeros((self.height,self.width,3),dtype=np.uint8)
                for i in range(0, self.height - thickness + 1, thickness):
                    for j in range(0, self.width - thickness + 1, thickness):
                        grid[i:i+thickness,j:j+thickness,:] = np.tile(np.array(self.wheel(pos), dtype=np.uint8),(thickness,thickness,1))
                        time.sleep(0.02*(thickness**2))
                        msg = UInt8MultiArray(data=list(grid.flatten()))
                        self.pixel_pub.publish(msg)
                        pos = pos + 1

    def wheel(self, pos):
        pos = pos % 255
        if pos < 85:
            r = int(pos * 3)
            g = int(255 - pos * 3)
            b = 0
        elif pos < 170:
            pos -= 85
            r = int(255 - pos * 3)
            g = 0
            b = int(pos * 3)
        else:
            pos -= 170
            r = 0
            g = int(pos * 3)
            b = int(255 - pos * 3)
        return (r, g, b)

def main(args=None):
    rclpy.init(args=args)
    node = NeoPixelNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
#!/home/peter/venv/bin/python3
import rclpy
from rclpy.node import Node
import board
import neopixel
from std_msgs.msg import ColorRGBA
from example_interfaces.msg import UInt8MultiArray

class NeoPixelNode(Node):
    default_color = (0, 0, 180)
    
    def __init__(self):
        super().__init__("NeoPixel")
        # NeoPixels must be connected to D10, D12, D18 or D21 to work.
        pixel_pin = board.D10
        # Grid size of used leds
        self.width = 28 # 27 unused, 55 total in a row
        self.height = 13 # number of rows all used
        # The number of NeoPixels (even the unused ones)
        self.num_pixels = (self.width*2 - 1)*self.height
        # The order of the pixel colors - RGB or GRB. Some NeoPixels have red and green reversed!
        # For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.
        ORDER = neopixel.GRB
        self.pixels = neopixel.NeoPixel(pixel_pin, self.num_pixels, brightness=0.2, auto_write=False, pixel_order=ORDER)
        self.pixels.fill((0, 0, 0))
        for i in range(self.height):
            for j in range(self.width):
                    self.pixels[j*2+i*(self.width*2 - 1)] = self.default_color
        self.pixels.show()
        self.pixel_array_sub = self.create_subscription(UInt8MultiArray, "/NeoPixel/array", self.pixel_array_callback, 1)
        self.pixel_fill_sub = self.create_subscription(ColorRGBA, "/NeoPixel/fill", self.pixel_fill_callback, 1)
        self.get_logger().info("Initialized!")

    def pixel_array_callback(self, msg):
        for i in range(self.height):
            for j in range(self.width):
                    if i%2 == 1:
                        k = self.width - j - 1
                    else:
                        k = j
                    if (j+i*self.width)*3+2 < len(msg.data):
                        self.pixels[k*2+i*(self.width*2 - 1)] = tuple(msg.data[(j+i*self.width)*3:(1+j+i*self.width)*3])
        self.pixels.show()
    
    def pixel_fill_callback(self, msg):
        self.pixels.fill((0, 0, 0))
        for i in range(self.height):
            for j in range(self.width):
                    self.pixels[j*2+i*(self.width*2 - 1)] = (int(255*msg.r), int(255*msg.g), int(255*msg.b))
        self.pixels.show()

def main(args=None):
    rclpy.init(args=args)
    node = NeoPixelNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
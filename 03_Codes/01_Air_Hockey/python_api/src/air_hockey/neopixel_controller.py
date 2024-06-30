try:
    import numpy as np
except:
    print("Error: Couldn't import numpy!")
    exit()
import board
import neopixel

class NeoPixel_Controller:
    def __init__(self, width:int=28, height:int=13, order=neopixel.GRB, brightness:float = 0.3, default_color:tuple[int,int,int]=(0, 0, 180), dont_show_default:bool=False):
        pixel_pin = board.D10
        # Grid size of used leds
        self.width = width # 27 unused, 55 total in a row
        self.height = height # number of rows all used
        # The number of NeoPixels (even the unused ones)
        self.num_pixels = (self.width*2 - 1)*self.height

        self._pixels = neopixel.NeoPixel(pixel_pin, self.num_pixels, brightness=brightness, auto_write=False, pixel_order=order)
        if not dont_show_default:
            self._pixels.fill((0, 0, 0))
            for i in range(self.height):
                for j in range(self.width):
                        self._pixels[j*2+i*(self.width*2 - 1)] = default_color
            self._pixels.show()

    def setFromFlatArray(self, array):
        for i in range(self.height):
            for j in range(self.width):
                    if i%2 == 1:
                        k = self.width - j - 1
                    else:
                        k = j
                    if (j+i*self.width)*3+2 < len(array):
                        self._pixels[k*2+i*(self.width*2 - 1)] = tuple(array[(j+i*self.width)*3:(1+j+i*self.width)*3])
        self._pixels.show()
    
    def fill(self, colorRGB:tuple[int,int,int]):
        self._pixels.fill((0, 0, 0))
        for i in range(self.height):
            for j in range(self.width):
                    self._pixels[j*2+i*(self.width*2 - 1)] = (int(255*colorRGB[0]), int(255*colorRGB[1]), int(255*colorRGB[2]))
        self._pixels.show()

def main():
    import time
    def wheel(pos):
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
    
    controller = NeoPixel_Controller()
    print("Starting")
    pos = 0
    while True:
        for thickness in range(1,7):
            grid = np.zeros((controller.height, controller.width, 3),dtype=np.uint8)
            for i in range(0, controller.height - thickness + 1, thickness):
                for j in range(0, controller.width - thickness + 1, thickness):
                    grid[i:i+thickness,j:j+thickness,:] = np.tile(np.array(wheel(pos), dtype=np.uint8),(thickness,thickness,1))
                    time.sleep(0.02*(thickness**2))
                    controller.setFromFlatArray(list(grid.flatten()))
                    pos = pos + 1

if __name__ == "__main__":
    main()
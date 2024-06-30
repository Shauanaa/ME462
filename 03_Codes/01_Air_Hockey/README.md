# Air Hockey
The codes that run on the Raspberry Pi of the air hockey table. The Raspberry Pi has Ubuntu 24.04 Server running.

### Python API
#### NeoPixel Controller
Python library to control NeoPixels of the air hockey table. Example usage:
```
from air_hockey.neopixel_controller import NeoPixel_Controller

controller = NeoPixel_Controller()

controller.fill((100, 0, 255)) # Set every pixel the specified rgb color.

controller.setFromFlatArray([(100, 0, 255),(80, 20, 255),(60, 40, 255),(40, 60, 255),(20, 80, 255),(0, 100, 255)]) # Sets pixels from array in order.
```
#### Commander
Python library to control micro robots wirelessly. Desk fan example:
```
from air_hockey.microrobot_commander import Commander
import time,math

commander = Commander()
theta = 0
while True:
    mag = 80*math.sin(theta)
    commander.send(1, int(mag), int(-mag), 200)
    theta = theta + 0.15
    time.sleep(0.1)
```
#### Localization
Python library to obtain pose data of micro robots. Example usage:
```
from air_hockey.localize_microrobot import Locator
import time

locator = Locator()
while True:
    print(locator.getPoses())
    time.sleep(0.5)
```
### ROS2 Workspace
This workspace contains the `air_hockey` package that includes the codes to be able to control LEDs of the table, control micro robots and obtain the pose information of these robots.
#### NeoPixels
The server code is run by the command `ros2 run air_hockey neopixel`. This code listens to 2 topics called `/NeoPixel/array` and `/NeoPixel/fill`. They show the given rgb array or all LEDs display a RGB color, respectively. Array elements are 8-bit unsigned integers and mimic OpenCV Numpy Array structure. Fill topic communicates with `std_msgs/msg/ColorRGBA` and expects float values that range from 0.0 to 1.0.
#### Commander
The commander code is run by the command `ros2 run air_hockey commander`. This code listens to `/agent_0/cmd` and `/agent_1/cmd` topics. These topics expect 3 integer values: left motor power, right motor power, and fan motor power, respectively. These values are in ranges {[-255,255],[-255,255],[0,255]}. This code then communicates with the commander ESP that controls the micro robots.
#### Localization
The localization code is run by the command `ros2 run air_hockey localization`. This code uses the camera to detect the April Tags of the micro robots and calculate their poses. Once detected, the pose data are published to `/agent_x/pose` topic, where x changes with the detected tag id.
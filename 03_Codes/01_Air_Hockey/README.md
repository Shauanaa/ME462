# Air Hockey
The codes that run on the Raspberry Pi of the air hockey table. The Raspberry Pi has Ubuntu 24 Server running.
### Ros2 Workspace
This workspace contains the `air_hockey` package that includes the codes to be able to control LEDs of the table, control micro robots and obtain the pose information of these robots.
#### NeoPixels
The server code is run by the command `ros2 run air_hockey neopixel`. This code listens to 2 topics called `/NeoPixel/array` and `/NeoPixel/fill`. They show the given rgb array or all LEDs display a RGB color, respectively. Array elements are 8-bit unsigned integers and mimic OpenCV Numpy Array structure. Fill topic communicates with `std_msgs/msg/ColorRGBA` and expects float values that range from 0.0 to 1.0.
#### Commander
The commander code is run by the command `ros2 run air_hockey commander`. This code listens to `/agent_0/cmd` and `/agent_1/cmd` topics. These topics expect 3 integer values: left motor power, right motor power, and fan motor power, respectively. These values are in ranges {[-255,255],[-255,255],[0,255]}. This code then communicates with the commander ESP that controls the micro robots.
#### Localization
The localization code is run by the command `ros2 run air_hockey localization`. This code uses the camera to detect the April Tags of the micro robots and calculate their poses. Once detected, the pose data are published to `/agent_x/pose` topic, where x changes with the detected tag id.
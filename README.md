# 🏒 AirHockey and Micro Robot Project
![istockphoto-1455938819-612x612 (1)](https://github.com/Shauanaa/ME462/assets/44114999/967f291a-d41e-4c4b-b271-bce5b575ca1f)


## 📚 Table of Contents
- [Project Abstract](#project-abstract)
- [AirHockey](#airhockey)
  - [Mechanical](#mechanical)
  - [Software](#software)
  - [Electronic](#electronic-airhockey)
- [Micro Robot](#micro-robot)
  - [Mechanical](#mechanical-1)
  - [Software](#software-1)
  - [Electronic](#electronic-microrobot)
- [How to Use the Code](#how-to-use-the-code)
- [Cautions](#cautions)
- [Contribution](#contribution)
- [License](#license)



## 🎯 Project Abstract
This project integrates an IoT-enabled air hockey table with a remotely-controllable micro robot, creating diverse game scenarios. It showcases the fusion of mechanical engineering and software development to produce an interactive, entertaining system that reimagines classic gameplay through modern technology.

## 🏒 AirHockey

### Mechanical
The skeleton of the hockey table is constructed using a robust framework of aluminum extrusions, providing both strength and versatility. The main structure utilizes 20x20 mm sigma profile extrusions, offering an excellent balance of lightweight design and structural integrity. For enhanced stability and support, particularly along the edges, 20x40 mm extrusions are employed for the top bars, ensuring a rigid and durable frame.

The playing surface of the table is crafted from high-quality plexy glass, chosen for its smooth texture and durability. This surface is meticulously perforated with 1 mm diameter holes arranged in a precise grid pattern. These holes serve a crucial function in the air hockey system, allowing for even air distribution across the entire playing field, which is essential for smooth puck movement.

To contain the gameplay and provide a clean aesthetic, the sides of the table are enclosed using medium-density fiberboard (MDF). This material was selected for its cost-effectiveness, ease of fabrication, and ability to be finished smoothly. The MDF side panels not only define the boundaries of the playing area but also contribute to the overall structural integrity of the table.

The entire assembly is held together using a combination of M5 and M6 screws and nuts. This hardware selection ensures a secure connection between all components while allowing for potential disassembly or modifications in the future. The use of standardized fasteners also simplifies maintenance and repairs.
This thoughtful design and material selection results in a hockey table that is not only sturdy and functional but also aesthetically pleasing and adaptable to various gameplay modifications or technological integrations.

There is a 'balcony' under the fans which is a housing for electronic components.


Our air hockey table is equipped with:

- High-powered air blowers
- High energy power supply for leds 
- Camera setup for object detection/tracking 
- LED lighting system for enhanced gameplay experience
- Remotely controllable side obstacles 

### Software
The software interfaces with 13 neopixel LED lines, strategically positioned and serially connected around the table. These LEDs are carefully wired and soldered to ensure stable connections between the power supply and each LED, enabling dynamic lighting effects during gameplay.

Complementing the LED system are mini electronic side obstacles, each equipped with its own controller. These obstacles are connected using jumper wires and breadboards, allowing for independent operation and easy modifications. This modular approach facilitates maintenance and potential future upgrades.

The combination of serially connected LED lines and independently controlled obstacles creates a flexible and responsive gaming platform, capable of supporting various gameplay scenarios and interactive features.

The air hockey system is powered by:

- Computer vision algorithms for real-time puck and leds
- Pip installable python code
  
### Electronics
- Please see the readme.md documents under the[Link to README](https://github.com/Shauanaa/ME462/blob/main/03_Codes/01_Air_Hockey/README.md)
  

## 🤖 Micro Robot

### Mechanical

The micro robot's chassis is additively manufactured using Creality 3D printers, offering a lightweight yet durable design. The chassis is engineered to house and protect all electronic components from external damage. Two strategically positioned motors control the robot's movement, enabling precise maneuvering and agile performance on the air hockey table. This combination of 3D-printed structure and dual-motor system results in a compact, resilient robot capable of swift and accurate movements during gameplay.

The micro robot features:

- Compact, agile design for quick movements
- Remotely Controllable motors
- Durable chassis to withstand impacts
- Modular design for easy maintenance and upgrades

### Software
The micro robot features two motors controlled by an ESP8266 microcontroller. An ESP-NOW controller enables remote control of the micro-robots, allowing for precise maneuvering. Additionally, the robot is equipped with a controllable propeller, enhancing its movement capabilities. This combination of dual-motor drive, wireless control, and propeller system creates a versatile and responsive robot, ideal for dynamic air hockey gameplay.

The micro robot is controlled by:

- You by publishing a message to the relevant topic, Raspberry Pi communicating with a controller ESP8266 via serial communication to send commands, controller ESP communicating with the micro robot ESP8266 using ESP-NOW protocol, and finally, the micro robot ESP applying the command.
### Electronics
- The schematics of the microrobot are provided as following:
  ![MicroRobotCircuit_bb (1)](https://github.com/Shauanaa/ME462/assets/44114999/ae0b8cf6-30b0-49f3-9239-375acf00825d)
- More technical drawing is as follows:
   ![MicroRobotCircuit_schem (1)](https://github.com/Shauanaa/ME462/assets/44114999/1906de21-5c7a-4281-8005-6e14d36fd1b1)



## 🖥️ How to Use the Code

Each entity in this project has its own readme file in details. HEre's the quick introduction version.
The folder structure is descripted as below:
```
project_root/
│
├── /03_Codes
│   └── 01_Air_Hockey/
│       ├── python_api/
│           └── microrobot_commander.py
│           ├── neopixel_controller.py
│       ├── ros2_ws/
│           └── src/air_hockey/
│               └── air_hockey/
│                   └── __init__.py
│                   ├── commander.py
│                   ├── commander_demo.py
│                   ├── localize.py
│                   ├── neopix.py
│                   ├── neopixel_demo.py
│               ├── config/
│                   └── cam_params.yaml
│               ├── resource/
│                   └── air_hockey
│               ├── test/
│                   └── test_copyright.py
│                   ├── test_flake8.py
│                   ├── test_pep257.py
│               ├── LICENSE
│               ├── package.xml
│               ├── setup.cfg
│               ├── setup.py
│           ├── 99-gpio.rules
│           ├── neopixel_test.py
│           ├── requirements.txt
│           ├── README.md
|       ├── qr_code_label_and_centroid_detection.py
│       └── camera_controller.py
│       └── Readme.md
│   └── 02_Micro_Robot/
|       ├── esp8266_espnow/
│           └── esp8266_espnow.ino
│       └── esp8266_espnow_agents/
│           └── esp8266_espnow_agents.ino
│           └── motor.ino
│       └── Readme.md

```


To get started with the AirHockey and Mini Robot project, follow these steps:

#### 1. Clone the Repository
```
git clone https://github.com/Shauanaa/ME462.git
cd ME462
```

#### 2. Install Dependencies
Ensure you have Python installed, then run:
```
pip install -e .
```

#### 3. Configure the System
##### For Python API:
  1) Install the requirements : `pip install neopixels serial numpy`
##### For ROS2:
  1) Create a venv: `python3 -m venv ~/ros2_venv`
  2) activate the venv `source ~/ros2_venv/bin/activate`
  3) install the required packages `pip install rclpy neopixels board serial`

#### 4. Run the Air Hockey System
##### For Python API:
  1) Install the requirements : `pip install neopixels serial numpy`
  2) Then simply run the codes: `python microrobot_commander.py` and `python neopixel_controller.py
##### For ROS2:
  1) navigate to the venv `cd ~/ros2_ws`
  2) Build your workspace: `colcon build`
  3) activate the venv `source ~/ros2_venv/bin/activate`
  
#### 5. Control the Micro Robot
Run the desired codew as folloing:`ros2 run air_hockey <package_name>`

#### 6. Additional Commands
- To segment the leds from the camera image : `python led_finder.py`
- To detect the QR code labels of the robots and positions of them: `python qr_code_label_and_centroid_detection.py`
- To test neopixels in ROS2: `python 03_Codes/01_Air_Hockey/ros2_ws/src/air_hockey/air_hockey/neopixel_demo.py`
- To test commander tools in ROS2: `python 03_Codes/01_Air_Hockey/ros2_ws/src/air_hockey/air_hockey/commander_demo.py`

For more detailed instructions and troubleshooting, please reach out the contributors and open an issue.


## ⚠️ Cautions

When using this system, please be aware of the following:
- 🛑PLEASE UNPLUG ALL OF THE CABLES WHILE OPENING THE ELECTRONIC DEVICES OF THE AIRHOCKEY TABLE🛑

- The IP address of the Raspberry Pi changes when connecting to different networks. Always ensure you have the correct IP address before attempting to connect to the Raspberry Pi.
- Do not forget to change the shebang of the ros2 codes


## 🤝 Contribution

We welcome contributions from the community! If you'd like to contribute, please follow these steps:

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

---

<p align="center">
  Made with ❤️ by the AirHockey and Mini Robot Team:
  Berk Ünlü, 
  Sait Burak Yücekaya,
  Yunus Çınar Kızıltepe,
  Tan Çağatay Acar,
  Doğukan Özoğul,
  Candaş Uysal
</p>

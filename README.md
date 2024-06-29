# 🏒 AirHockey and Mini Robot Project
![istockphoto-1455938819-612x612 (1)](https://github.com/Shauanaa/ME462/assets/44114999/967f291a-d41e-4c4b-b271-bce5b575ca1f)


## 📚 Table of Contents
- [Project Abstract](#project-abstract)
- [AirHockey](#airhockey)
  - [Mechanical](#mechanical)
  - [Software](#software)
- [Mini Robot](#mini-robot)
  - [Mechanical](#mechanical-1)
  - [Software](#software-1)
- [How to Use the Code](#how-to-use-the-code)
- [Cautions](#cautions)
- [Contribution](#contribution)
- [License](#license)



## 🎯 Project Abstract
This project integrates an IoT-enabled air hockey table with a remotely-controllable microrobot, creating diverse game scenarios. It showcases the fusion of mechanical engineering and software development to produce an interactive, entertaining system that reimagines classic gameplay through modern technology.

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
  

## 🤖 Mini Robot

### Mechanical

The mini robot features:

- Compact, agile design for quick movements
- Remotely Controllable motors
- Durable chassis to withstand impacts
- Modular design for easy maintenance and upgrades

### Software

The mini robot is controlled by:

- Wireless communication with the main computer using micro ROS connection

## 🖥️ How to Use the Code

The folder structure is descripted as below:
'''bash
project_root/
│
├── AirHockey/
│   └── Software/
│       ├── neopixel_led.py
│       └── camera_controller.py
│
└── MiniRobot/
    └── Software/
        └── minirobot_controller.py
'''


To get started with the AirHockey and Mini Robot project, follow these steps:

#### 1. Clone the Repository
#### 2. Install Dependencies
Ensure you have Python installed, then run:

#### 3. Configure the System
- Update the `config.py` file with your specific settings, including the IP address of the mini robot.

#### 4. Run the Air Hockey System

#### 5. Control the Mini Robot

#### 6. Additional Commands
- To calibrate the camera: `python calibrate_camera.py`
- To test the LED system: `python test_leds.py`

For more detailed instructions and troubleshooting, please refer to the `docs` folder.


## ⚠️ Cautions

When using this system, please be aware of the following:

- The IP address of the microrobot changes when connecting to different networks. Always ensure you have the correct IP address before attempting to connect or control the microrobot.


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
  Berk Ünlü 
  Sait Burak Yücekaya
  Yunus Çınar Kızıltepe
  Tan Çağatay Acar
  Doğukan Özoğul
  Candaş Uysal
</p>

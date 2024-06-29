# Air Hockey ROS2 Workspace
This workspace contains the `air_hockey` package. All details are explained in [here](/03_Codes/01_Air_Hockey/README.md).
To build the package, come to this folder and type `colcon build`. Then source the environment with `source ./install/setup.bash`.

## Permissions for NeoPixel GPIO Access
In order to be able to control the NeoPixels, run the following commands:
```
groupadd gpio
usermod -a -G gpio <username>
sudo cp ./99-gpio.rules /etc/udev/rules.d/99-gpio.rules
```
Now you should be set and able to use the NeoPixel codes.
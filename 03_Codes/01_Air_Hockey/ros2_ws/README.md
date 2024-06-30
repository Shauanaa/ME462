# Air Hockey ROS2 Workspace
This workspace contains the `air_hockey_ros` package. All details are explained in [here](/03_Codes/01_Air_Hockey/README.md).

## Setting Up the Environment
You need to set up a virtual env in a specific place. To do so, run the following commands:
```
sudo apt install python3-venv -y
cd ~
python3 -m venv venv
source venv/bin/activate
pip3 install -r path/to/ws/requirements.txt
```
Also you need to update the shebang in each python file such that "peter" is replaced with your username.

To build the package, come to this folder, make sure you activated the virtual environment, and type `colcon build`. Then source the environment with `source ./install/setup.bash`.

## Permissions for NeoPixel GPIO Access
In order to be able to control the NeoPixels, run the following commands:
```
groupadd gpio
usermod -a -G gpio <username>
sudo cp ./99-gpio.rules /etc/udev/rules.d/99-gpio.rules
```
Now you should be set and able to use the NeoPixel codes.
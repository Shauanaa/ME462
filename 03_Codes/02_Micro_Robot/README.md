# Micro Robot
This robot is a small, remote controllable, heavy robot that is also equiped with a fan. Controlling these robots remotely is not explained here and instead discussed in [Air Hockey Section](/03_Codes/01_Air_Hockey).

### Mechanics
The mechanical design of the robots are completed with Autodesk Fusion. Therefore, Fusion Archive Files and stls are included in [this folder](/04_CAD_Files/02_Micro_Robot).

### Electronics
The electonics are fitted in such a small compartment that we need the smallest components and modules to construct this robot. The list of the components used to build one robot is provided here:

* [L9110 Fan Module](https://www.robolinkmarket.com/l9110-fan-modulu)
* [2x70 RPM 12mm Micro Motor](https://www.robotistan.com/6v-12mm-70-rpm-reduktorlu-dc-motor)
* [TB6612FNG Motor Controller](https://www.robotistan.com/tb6612fng-step-motor-surucu-dc-motor-sucuru?language=tr&h=288b471b&_sgm_campaign=product&_sgm_source=7370&_sgm_action=search&_sgm_term=tb6&_sgm_pinned=false)
* [Lolin Mini Clone ESP8266 Card](https://www.robolinkmarket.com/esp8266-nodemcu-mini-gelistirme-karti)
* [2A Mini Buck Converter](https://www.robolinkmarket.com/2a-mini-ayarlanabilir-voltaj-dusurucu-regulator-karti)
* [2x18650 Li-ion Battery](https://www.robolinkmarket.com/power-xtra-px18650-25p-37v-2500-mah-li-ion-sarjli-pil-12c?_sgm_campaign=fcs_9ec70268bc000&_sgm_source=8776%7Cproduct&_sgm_action=search&_sgm_term=18650&_sgm_pinned=true&_sgm_pin_position=1)
* [2S 18650 Battery Bed](https://www.robolinkmarket.com/2li-18650-pil-yuvasi-lityum?_sgm_campaign=product&_sgm_source=6158%7Cbrand&_sgm_action=search&_sgm_term=pil+yata%C4%9F%C4%B1+18650&_sgm_pinned=false)
* On/Off Switch
* Female Headers and Wires

The pin assignments are present in the codes provided.

### Software
The ESP8266 microcontroller is coded with Arduino IDE. 2 codes are present, `esp8266_espnow_agents` code is for the robots. The `AGENT_ID` should be unique for each robot that will operate simultaneously. `esp8266_espnow` code is for the controller ESP8266 that is used to communicate with the robots. This controller should be connected to your PC or the Raspberry Pi via a USB cable to be able to control the robots. `MAX_AGENTS` parameter dictates the maximum number of robots to be controlled simultaneously. Also, the robots should have ids lower than this value!
# VersaIMU -- (versatile) IMU interface

_VersaIMU_ is a stripped down version of [VersaVIS](https://github.com/ethz-asl/versavis) that allows receiving data from an IMU and publishing the sensor data in ROS. 
Currently, only ADIS16448 (both AMLZ and BMLZ versions) is supported. Data is published using a slightly modified version of [rosserial](https://github.com/ethz-asl/rosserial/). 


## Install

### Clone and build

```
cd ~/catkin_ws/src/
git clone git@github.com:mantelt/versaIMU.git --recursive
catkin build versavis_adis16448_receiver
cd versaIMU/firmware
./setup.sh
```

### Setup udev rule
Add yourself to `dialout` group
```
sudo adduser <username> dialout
```

Copy udev rule file to your system:
```
sudo cp firmware/98-versa-vis.rules /etc/udev/rules.d/98-versa-vis.rules
```
Afterwards, use the following commands to reload the rules
```
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo ldconfig
```
Note: You might have to reboot your computer for this to take effect. You can check by see whether a `/dev/versavis` is available and pointing to the correct device.

### Configure
Adapt the [configuration file](https://github.com/mantelt/versaIMU/blob/master/firmware/libraries/versavis/src/versavis_configuration.h) to your setup needs. 

### Flash firmware on the VersaVIS board
* Install the arduino IDE from [here](https://www.arduino.cc/en/Main/OldSoftwareReleases#previous). Use version 1.8.2!
    - Note that a small modification of the install script (`install.sh`) might be required. In particular you may need to change the line `RESOURCE_NAME=cc.arduino.arduinoide` to `RESOURCE_NAME=arduino-arduinoide` as per the issue [here](https://github.com/arduino/Arduino/issues/6116#issuecomment-290012812).
* Open `firmware/versavis/versavis.ino` in the IDE
* Go to `File -> Preferences`
* Change Sketchbook location to `versavis/firmware/`
* Install board support:
    - For VersaVIS 1.0 (the black one): [Check here](https://github.com/ethz-asl/versavis_hw/)
* Set `Tools -> Port -> tty/ACM0 (Arduino Zero)`, and `Tools -> Board -> VersaVIS`.
* Compile using the *Verify* menu option
* Flash using the *Upload* menu option

## Usage
* Adapt `versavis/launch/run_versavis_adis16448.launch` to your needs.
* Run with
```
roslaunch versavis run_versavis_adis16448.launch
```
* Wait for successfull initialization.

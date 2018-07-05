# malish  
Malish is a small industrial robot on omniwheels, which can use advanced mapping via RGBD camera.
It has many options and can work collaboratively with humans.
Malish is a testing prototype which will further be replaced by Tolstyak.

It consists such parts as:
*Jetson - Core, Mapping
*Odroid - Move_Base, Joy_Control
*Adruino - Motor control, encoders
*Adruino(Extended) - LED, lifters, buzzer, sonar, IMU
*Wi-fi router
*Accumulator
*Power routing
*Breadboard Power 1-3
*Breadboard lifter
*Breadboard hot power replace
*Breadboard sonar/imu rooting
*Breadboard buzzer, LED

## System setup:

### ROS setup.

```bash
sudo apt-get update
sudo apt-get upgrade
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc


sudo apt-get install ros-kinetic-joy ros-kinetic-rosserial ros-kinetic-pcl-ros ros-kinetic-tf2-geometry-msgs ros-kinetic-rtabmap ros-kinetic-rtabmap-ros ros-kinetic-urg-node ros-kinetic-image-view ros-kinetic-robot-localization ros-kinetic-move-base ros-kinetic-teb-local-planner ros-kinetic-global-planner ros-kinetic-teb-local-planner ros-kinetic-range-sensor-layer ros-kinetic-eband-local-planner ros-kinetic-find-object-2d ros-kinetic-aruco-detect ros-kinetic-fiducial-msgs ros-kinetic-fiducial-slam ros-kinetic-fiducials ros-kinetic-usb-cam ros-kinetic-amcl ros-kinetic-map-server

```
Then you should look which IP does have your pc, using *ifconfig* and add it to .bashrc for ros_ip (do it after you connect to ROBOT NETWORK):
```bash
echo "export ROS_IP=$(hostname -I)" >> ~/.bashrc
export AVIS_ROOT=~/catkin_ws
```

After that you can add host machine ip here as well.
* User interface:

```bash
sudo apt-get install terminator xboxdrv ssh screen zsh
```

* Installing oh-my-zsh:
```bash
sh -c "$(wget https://raw.githubusercontent.com/robbyrussell/oh-my-zsh/master/tools/install.sh -O -)"
```


* GCC/G++ 6

```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-6 g++-6
sudo update-alternatives --remove-all gcc
sudo update-alternatives --remove-all g++
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 10
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-6 10
```

### Git & Building of source code

```bash
ssh-keygen
cat ~/.ssh/id_rsa.pub
git clone git+ssh://git@github.com/arobosys/malish.git catkin_ws
git clone --recurse-submodules git+ssh://git@github.com/arobosys/malish.git catkin_ws #With submodules (like ZED-wrapper)
cd ~/catkin_ws
git checkout devel
catkin_make
catkin_make -DCATKIN_BLACKLIST_PACKAGES="zed-ros-wrapper" #For pc without cuda
catkin_make -pkg malish #To make only one package
```

### ZSH/ssh recommendations
* Copying the settings from .bashrc-->.zshrc:
```bash
echo "source /opt/ros/kinetic/setup.zsh" >> ~/.zshrc
echo "source ~/catkin_ws/devel/setup.zsh" >> ~/.zshrc
echo "export ROS_IP=$(hostname -I)" >> ~/.zshrc
```
* Camera calibration:
```bash
roslaunch usb_cam usb_cam-test.launch
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.035 image:=/image_view/output camera:=/usb_cam
```


* For ssh use: 
```bash
ssh -X -C username@address
```
and you will have access to X-server as well. 

#Joystick/ Arduino setup:
Add code:
```bash
sudo xboxdrv -s
```
to /etc/rc.local, and do:

```bash
sudo usermod -a -G dialout $USER
sudo chmod a+rw /dev/ttyACM0 /dev/ttyUSB0
```
for arduino.

###Building Arduino libraries
To build arduino libraries you do:
```bash
cd ~/catkin_ws
catkin_make
cd ~/Arduino/libraries
rm -r ros_lib
rosrun rosserial_arduino make_libraries.py . |grep malish
```

Then you can check in the ros_lib folder if everything is right.

## Start malish system
To start the malish complicated system you will use at least 3 ubuntu PC's (host machine, jetson and odroid); and 2 arduino via rosserial.
* Start host machine
* Do ssh from host machine to odroid and ubuntu
For this you better have an alias(in Gosha PC just type ubod and 2 terminals will come up)
Another good improvement could be naming IP's in */etc/hosts* and getting their ssh key's *ssh-copy-id ubuntu@rtx2*

```bash
ssh -X -C ubuntu@rtx2 terminator
```
Then on ubuntu pc you separate screen by Ctrl+E.
On first sid you execute
```
screen -S core
roscore
```
On the second one
```
screen -S map
roslaunch buggy_2dnav mapping.launch
```

Then you open the odroid and do same thing with: 
```
screen -S joy
roslaunch malish joy_control.launch
```
and for movebase:
```
screen -S move
roslaunch buggy_2dnav move_base.launch
``` 

After the camera recorded the map you can run:
```
rosservice call /rtabmap/localization_mode_on
``` 

To launch rviz on host PC you should do:
```bash 
export ROS_MASTER_URI=http://IP_OF_YOUR_MASTER!!!:11311
rviz
``` 


### Arduino devices  
#### IMU connection  
'V' - 3.3V; 
'G' - ground; 
Arduino UNO: 
'D' - A3; 
'C' - A4; 
Arduino MEGA 2560 
'D' - SDA 20; 
'C' - SCL 21; 
Depends upon: 
TroykaIMU.h https://github.com/amperka/Troyka-IMU 

#### Sonars connection 
            left, right, front, rear   
trig pins =   50,    46,    38,   42; 
echo pins =   48,    44,    36,   40; 

Depends upon:
Ultrasonic.h https://github.com/JRodrigoTech/Ultrasonic-HC-SR04
In Ultrasonic.cpp change line21: Time_out=6000;

#### LED/buzz connection  
red | green  | blue | buzz
--- | ------ | ---- | ----
6   |   5    |  4   | 7


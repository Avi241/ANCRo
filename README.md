# ancro
 Raspberry Pi 4 Setup Steps


Insert Class 10 32/64 GB MicroSD Card with SD card Adapter/Card reader to your Laptop

Download Image file from https://cdimage.ubuntu.com/releases/18.04/release/ubuntu-18.04.5-preinstalled-server-arm64+raspi3.img.xz
Flash memory card with the file “ubuntu-18.04.5-preinstalled-server-arm64+raspi3.img.xz”using Balena Etcher

After Flash is Complete. Goto System boot Partition of Flashed SD Card 
Right-Click and click “open in terminal”
Open the File network-config by Command
sudo gedit network-config
Uncomment the wifis: and wlan0 section in the network-config file 
Replace your SSID and Password instead of homessid and password and Save the File
Example- SSID is robonet

Eject the System-boot and Remove MicroSD Card
Insert the MicroSD card into Raspberry Pi and start the Raspberry Pi by Powering it up with 5V 3A power supply 
(Here I have used Headless Setup for Raspberry Pi 4 You can choose Desktop Setup also with Ubuntu 18.04 LTS)

Find the IP address of Pi with Angry IP Scanner or any scanner
Connect with SSH (Using Putty SSH Client or any Other) 
user name-ubuntu  password-ubuntu
Change the password at the first login and after changing connect again with a new password.


To Update the System and Packages
sudo apt-get update && sudo apt-get upgrade -y

Microphone and Speaker setup with USB Sound Card(We are using USb Sound Card adapter to connect Mic and Speaker)
sudo apt-get install espeak
sudo apt-get install alsa-utils
sudo apt-get install pulseaudion

How to use USB Pendrive
sudo fdisk -l
sudo  mkdir /media/usb
sudo mount /dev/sdb1 /media/usb
now you can copy files/folder by cp command, Example : cp test.py /home/ubuntu/
sudo umount /media/usb

Vosk Speech Recognizer
sudo apt-get install python3-pip -y  
sudo -H pip3 install --upgrade pip 
python3 -m pip install https://github.com/alphacep/vosk-api/releases/download/0.3.21/vosk-0.3.21-py3-none-linux_aarch64.whl

Get your preferred model from here : https://alphacephei.com/vosk/models

Port audio and Py audio

First we need to install portaudio modules: 
sudo apt-get install libasound-dev
Download the portaudio archive from: http://portaudio.com/download.html (OR)
wget http://files.portaudio.com/archives/pa_stable_v190700_20210406.tgz 
Unzip the archive: 
tar -zxvf [portaudio.tgz]
Enter the directory, then run: 
./configure && make
Install: 
sudo make install
And Then: 
sudo pip install pyaudio
Update & Upgrade the system to fix all broken libraries:
sudo apt update && sudo apt upgrade

Wakeup Word detection
sudo pip3 install pvporcupinedemo
porcupine_demo_mic --keywords blueberry  // to run the example of detection of word blueberry

ROS Melodic Installation
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-melodic-desktop-full
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools -y
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
mkdir catkin_ws
cd catkin_ws
mkdir -p src
cd src
catkin_init_workspace
pip3 install rospkg
echo “source ~/catkin_ws/devel/setup.bash” >> ~/.bashrc
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
sudo apt-get install ros-melodic-rosserial*
sudo apt-get install ros-melodic-openni*
sudo apt-get install ros-melodic-freenect* 

ROS INSTALLATION UBUNTU 18 # Detail Installation Instruction

YDLidar ROS

Clone this project to your catkin's workspace src folder
cd ~/catkin_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros
Running catkin_make to build ydlidar_node and ydlidar_client
cd ~/catkin_ws
catkin_make
Create the name "/dev/ydlidar" for YDLIDAR
roscd ydlidar_ros/startup
sudo chmod 777 ./*
sudo sh initenv.sh
Nvidia Jetson nano Setup Steps


Insert Class 10 32/64 GB MicroSD Card with SD card Adapter/Card reader to your Laptop

Download Image file from https://developer.nvidia.com/jetson-nano-2gb-sd-card-image
Flash memory card with the file “jetson-nano-2gb-jp451-sd-card-image.zip”using Balena Etcher
After the Flash is Complete Remove MicroSd card from Laptop
Insert the MicroSD card into Nvidia Jetson Nano and start the Nvidia Jetson Nano by Powering it up with 5V 3A power supply and Connecting HDMI,Mouse,Keyboard.

Do the initial Setup of Ubuntu It will open the Desktop in front of You
Open terminal

Object detection Setup:
sudo apt-get update
sudo apt-get install v4l-utils
sudo apt-get update
sudo apt-get install dialog
sudo apt-get install git cmake libpython3-dev python3-numpy
git clone --recursive https://github.com/dusty-nv/jetson-inference
cd jetson-inference
mkdir build
cd build
cmake ../

Make choice for the models. If the models are not downloading, make sure you have installed the dialog. Still see problems, please use link https://github.com/dusty-nv/jetson-inference/releases
Just Press OK enter

make 
sudo make install
sudo ldconfig

Running Jetson Object detection demo
cd /jetson-inference/build/aarch64/bin
./detectnet-camera.py --network=ssd-mobilenet-v2 --camera=/dev/video0




Communication Setup Installing Tensorflow
sudo apt-get update
sudo apt-get install libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev zip libjpeg8-dev liblapack-dev libblas-dev gfortran
sudo apt-get install python3-pip
sudo pip3 install -U pip testresources setuptools==49.6.0
sudo pip3 install -U numpy==1.19.4 future==0.18.2 mock==3.0.5 h5py==2.10.0 keras_preprocessing==1.1.1 keras_applications==1.0.8 gast==0.2.2 futures protobuf pybind11
sudo pip3 install --pre --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v45 tensorflow


Sharing Wifi Internet over ethernet
https://askubuntu.com/questions/1104506/share-wireless-internet-connection-through-ethernet-on-18-04-lts


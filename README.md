# ANCRO (Autonomous Navigation and Communication Robot): Raspberry Pi 4 and Nvidia Jetson Nano Setup Guide

## Raspberry Pi 4 Setup

### Requirements
- Class 10 32/64 GB MicroSD Card with SD card Adapter/Card reader
- Laptop with internet connection
- Raspberry Pi 4
- 5V 3A power supply
- SSH Client (e.g., PuTTY)
- Angry IP Scanner or any IP scanner

### Steps

1. **Prepare the MicroSD Card**:
    - Insert the MicroSD card into your laptop.
    - Download the Ubuntu 18.04 image file from [Ubuntu Releases](https://cdimage.ubuntu.com/releases/18.04/release/ubuntu-18.04.5-preinstalled-server-arm64+raspi3.img.xz).
    - Flash the image to the MicroSD card using [Balena Etcher](https://www.balena.io/etcher/).

2. **Configure Network Settings**:
    - Open the System-boot partition of the flashed SD card.
    - Open a terminal in this partition.
    - Edit the `network-config` file:
      ```sh
      sudo gedit network-config
      ```
    - Uncomment the `wifis:` and `wlan0` sections.
    - Replace `homessid` and `password` with your WiFi SSID and password.
    - Save and close the file.

3. **Boot Raspberry Pi**:
    - Eject the MicroSD card and insert it into the Raspberry Pi.
    - Power on the Raspberry Pi with a 5V 3A power supply.
    - Find the IP address of the Raspberry Pi using Angry IP Scanner.
    - Connect via SSH:
      ```sh
      ssh ubuntu@<pi-ip-address>
      ```
    - Use `ubuntu` as both the username and password. Change the password upon first login.

4. **Update System and Packages**:
    ```sh
    sudo apt-get update && sudo apt-get upgrade -y
    ```

5. **Microphone and Speaker Setup**:
    - Install required packages:
      ```sh
      sudo apt-get install espeak alsa-utils pulseaudio
      ```

6. **Using USB Pendrive**:
    - Mount USB pendrive:
      ```sh
      sudo fdisk -l
      sudo mkdir /media/usb
      sudo mount /dev/sdb1 /media/usb
      ```
    - Copy files:
      ```sh
      cp test.py /home/ubuntu/
      ```
    - Unmount USB pendrive:
      ```sh
      sudo umount /media/usb
      ```

7. **Vosk Speech Recognizer**:
    - Install Vosk:
      ```sh
      sudo apt-get install python3-pip -y
      sudo -H pip3 install --upgrade pip
      python3 -m pip install https://github.com/alphacep/vosk-api/releases/download/0.3.21/vosk-0.3.21-py3-none-linux_aarch64.whl
      ```
    - Download model from [Vosk Models](https://alphacephei.com/vosk/models).

8. **PortAudio and PyAudio**:
    - Install PortAudio:
      ```sh
      sudo apt-get install libasound-dev
      wget http://files.portaudio.com/archives/pa_stable_v190700_20210406.tgz
      tar -zxvf pa_stable_v190700_20210406.tgz
      cd portaudio
      ./configure && make
      sudo make install
      ```
    - Install PyAudio:
      ```sh
      sudo pip install pyaudio
      ```

9. **Wakeup Word Detection**:
    ```sh
    sudo pip3 install pvporcupinedemo
    porcupine_demo_mic --keywords blueberry
    ```

10. **ROS Melodic Installation**:
    - Add ROS repository and key:
      ```sh
      sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
      sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
      ```
    - Install ROS:
      ```sh
      sudo apt-get update
      sudo apt-get install ros-melodic-desktop-full
      sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools -y
      sudo rosdep init
      rosdep update
      ```
    - Configure ROS environment:
      ```sh
      echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
      source ~/.bashrc
      mkdir -p ~/catkin_ws/src
      cd ~/catkin_ws/src
      catkin_init_workspace
      cd ~/catkin_ws
      catkin_make
      echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
      source ~/.bashrc
      ```
    - Install additional ROS packages:
      ```sh
      sudo apt-get install ros-melodic-rosserial* ros-melodic-openni* ros-melodic-freenect*
      ```

11. **YDLidar ROS**:
    - Clone and build YDLidar ROS package:
      ```sh
      cd ~/catkin_ws/src
      git clone https://github.com/YDLIDAR/ydlidar_ros
      cd ~/catkin_ws
      catkin_make
      ```
    - Set up YDLidar:
      ```sh
      roscd ydlidar_ros/startup
      sudo chmod 777 ./*
      sudo sh initenv.sh
      ```

## Nvidia Jetson Nano Setup

### Requirements
- Class 10 32/64 GB MicroSD Card with SD card Adapter/Card reader
- Laptop with internet connection
- Nvidia Jetson Nano
- 5V 3A power supply
- HDMI monitor, mouse, and keyboard

### Steps

1. **Prepare the MicroSD Card**:
    - Insert the MicroSD card into your laptop.
    - Download the Jetson Nano image file from [Nvidia Developer](https://developer.nvidia.com/jetson-nano-2gb-sd-card-image).
    - Flash the image to the MicroSD card using [Balena Etcher](https://www.balena.io/etcher/).

2. **Initial Setup**:
    - Insert the MicroSD card into the Jetson Nano.
    - Connect HDMI, mouse, and keyboard.
    - Power on the Jetson Nano and complete the Ubuntu setup.

3. **Object Detection Setup**:
    - Install required packages:
      ```sh
      sudo apt-get update
      sudo apt-get install v4l-utils dialog git cmake libpython3-dev python3-numpy
      ```
    - Clone and build Jetson Inference:
      ```sh
      git clone --recursive https://github.com/dusty-nv/jetson-inference
      cd jetson-inference
      mkdir build
      cd build
      cmake ../
      make
      sudo make install
      sudo ldconfig
      ```
    - Run Object Detection Demo:
      ```sh
      cd /jetson-inference/build/aarch64/bin
      ./detectnet-camera.py --network=ssd-mobilenet-v2 --camera=/dev/video0
      ```

4. **TensorFlow Installation**:
    - Install TensorFlow dependencies:
      ```sh
      sudo apt-get update
      sudo apt-get install libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev zip libjpeg8-dev liblapack-dev libblas-dev gfortran
      sudo apt-get install python3-pip
      sudo pip3 install -U pip testresources setuptools==49.6.0
      sudo pip3 install -U numpy==1.19.4 future==0.18.2 mock==3.0.5 h5py==2.10.0 keras_preprocessing==1.1.1 keras_applications==1.0.8 gast==0.2.2 futures protobuf pybind11
      sudo pip3 install --pre --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v45 tensorflow
      ```

5. **Sharing WiFi Internet Over Ethernet**:
    - Follow instructions from [Ask Ubuntu](https://askubuntu.com/questions/1104506/share-wireless-internet-connection-through-ethernet-on-18-04-lts).

---

This guide provides a comprehensive setup for both Raspberry Pi 4 and Nvidia Jetson Nano, including network configuration, system updates, peripheral setup, ROS installation, and additional software installations. For any further assistance or troubleshooting, please refer to the respective official documentation.

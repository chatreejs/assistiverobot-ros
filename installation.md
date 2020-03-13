# Installation Instruction
Instruction for setup environment to using this package
## ROS Installation
### 1. Setup your sources.list
Setup your computer to accept software from packages.ros.org.
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### 2. Set up your keys
```bash
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
If you experience issues connecting to the keyserver, you can try substituting `hkp://pgp.mit.edu:80` or `hkp://keyserver.ubuntu.com:80` in the previous command.

Alternatively, you can use curl instead of the apt-key command, which can be helpful if you are behind a proxy server:
```bash
$ curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
```

### 3. Installation
First, make sure your Debian package index is up-to-date:
```bash
$ sudo apt update
```
There are many different libraries and tools in ROS. We provided four default configurations to get you started. You can also install ROS packages individually.

In case of problems with the next step, you can use following repositories instead of the ones mentioned above [ros-shadow-fixed](http://wiki.ros.org/ShadowRepository)
* **Desktop-Full Install: (Recommended)** : ROS, [rqt](http://wiki.ros.org/rqt), [rviz](http://wiki.ros.org/rviz), robot-generic libraries, 2D/3D simulators and 2D/3D perception
    ```bash
    $ sudo apt install ros-melodic-desktop-full
    ```
    or [click here](apt:ros-melodic-desktop-full?refresh=yes)
* **Desktop Install**: ROS, [rqt](http://wiki.ros.org/rqt), [rviz](http://wiki.ros.org/rviz), and robot-generic libraries
    ```bash
    $ sudo apt install ros-melodic-desktop
    ```
    or [click here](apt:ros-melodic-desktop?refresh=yes)
* **ROS-Base**: (Bare Bones) ROS package, build, and communication libraries. No GUI tools.
    ```bash
    $ sudo apt install ros-melodic-ros-base
    ```
    or [click here](apt:ros-melodic-ros-base?refresh=yes)
* **Individual Package**: You can also install a specific ROS package (replace underscores with dashes of the package name):
    ```bash
    $ sudo apt install ros-melodic-PACKAGE
    ```
    e.g.
    ```bash
    $ sudo apt install ros-melodic-slam-gmapping
    ```
To find available packages, use:
```bash
$ apt search ros-melodic
```

### 4. Initialize rosdep
Before you can use ROS, you will need to initialize `rosdep`. `rosdep` enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS.

```bash
$ sudo rosdep init
$ rosdep update
```

### 5. Environment setup
It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched:

```bash
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```
*If you have more than one ROS distribution installed, `~/.bashrc` must only source the `setup.bash` for the version you are currently using.*

If you just want to change the environment of your current shell, instead of the above you can type:

```bash
$ source /opt/ros/melodic/setup.bash
```
If you use zsh instead of bash you need to run the following commands to set up your shell:

```bash
$ echo "source /opt/ros/melodic/setup.zsh" >> ~/.zshrc
$ source ~/.zshrc
```

### 6. Dependencies for building packages
Up to now you have installed what you need to run the core ROS packages. To create and manage your own ROS workspaces, there are various tools and requirements that are distributed separately. For example, [rosinstall](http://wiki.ros.org/rosinstall) is a frequently used command-line tool that enables you to easily download many source trees for ROS packages with one command.

To install this tool and other dependencies for building ROS packages, run:

```bash
$ sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

## Kobuki Package Installation
### 1. Create catkin workspace
Create kobuki workspace for build a kobuki package:
```bash
$ mkdir -p ~/kobuki_ws/src
```

### 2. Clone Kobuki Package
Clone kobuki package to `kobuki_ws`:
```bash
$ cd ~/kobuki_ws/src
$ git clone https://github.com/yujinrobot/kobuki.git
```
Checkout to branch melodic to use kobuki package in ros-melodic:
```bash
$ git checkout melodic
```

### 3. Install Dependencies
Install kobuki package dependencies for melodic distro:
```bash
$ cd ~/kobuki_ws
$ rosdep install —from-paths src -i -y -r
```

### 4. Build package
Build a kobuki package:
```bash
$ catkin_make
```

### 5. Environment Setup
It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched:

```bash
$ echo "source ~/kobuki_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

### 6. Dialout Group
If not already in the dialout group:

```bash
$ sudo usermod -a -G dialout $USER
```
and then log out, log back in again.

### 7. Set Udev Rule
```
$ rosrun kobuki_ftdi create_udev_rules
```
Reinsert the Kobuki's USB cable into your laptop/pc/... You should now find it show up at `/dev/kobuki.`

## Navigation Stack Package Installation
### 1. Install Navigation Stack Package
```bash
$ sudo apt-get install ros-melodic-navigation-tutorials 
```

### 2. Install DWA Local Planner
```bash
$ sudo apt-get install ros-melodic-dwa-local-planner
```

## OpenSLAM Package Installation
```bash
$ sudo apt-get install ros-melodic-openslam-gmapping ros-melodic-gmapping
```

## RPLIDAR Package Installation
### 1. Create catkin workspace
Create RPLIDAR workspace for build a RPLIDAR package:
```bash
$ mkdir -p ~/rplidar_ws/src
```

### 2. Clone RPLIDAR Package
Clone RPLIDAR package to `rplidar_ws`:
```bash
$ cd ~/rplidar_ws/src
$ git clone https://github.com/robopeak/rplidar_ros.git
```

### 4. Build package
Build a kobuki package:
```bash
$ cd ~/rplidar_ws
$ catkin_make
```

### 5. Environment Setup
It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched:

```bash
$ echo "source ~/rplidar_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

### 6. Device Settings
Check the authority of rplidar's serial-port :
```bash
$ ls -l /dev | grep ttyUSB
```
Add the authority of write: (such as /dev/ttyUSB0)
```bash
$ sudo chmod 666 /dev/ttyUSB0
```

For fixed rplidar port, you can using the script file to remap the USB port name:
```bash
$ roscd ~/rplidar_ros/scripts
$ ./create_udev_rules.sh
```
Once you have change the USB port remap, you can change the launch file about the serial_port value.
```xml
<param name="serial_port" type="string" value="/dev/rplidar"/>
```

# ROS Installation for Pioneer 3 Robot

**OS:** Ubuntu 20.04.2

## Recommendations

### Virtual machine (for mac and windows users)

If using a Mac or Windows OS its recommended that you install a virtual machine using Linux. One option is parallels ( https://www.parallels.com/au/ ). Just follow the install instructions and it will ask what OS you want to install (choose Ubuntu). 

### Terminal management

ROS requires a several terminal windows to be open at a time. A terminal window manager like tmux is recommend (`sudo apt install tmux`) as it makes it easier to switch between windows.

---

## Installation

There are a few things that need to be installed (instructions below):
1. ROS - Using Noetic for this project
2. Catkin - Workspace / build system for ROS
3. ARIA - A ROS package for interfacing with the Pioneer 3 robot
4. AMRISim - A virtual simulator for the Pioneer 3 robot

### ROS (Noetic)

Instructions taken from: [noetic/Installation/Ubuntu - ROS Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu)

1. Accept packages (apt install ...) from packages.ros.org

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

2. Install cURL `sudo apt install curl`

3. Setup apt keys

```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

4. Update `sudo apt update`

5. Install full version of ROS noetic

```bash
sudo apt install ros-noetic-desktop-full
```

6. Add the setup script to .bashrc to run every time a new terminal is opened

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

7. Install dependencies to ROS noetic

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

8. Install mode dependencies (rosdep)

```bash
sudo apt install python3-rosdep
```

9. Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

### Catkin (workspace for ROS)

Instructions taken from: [ROS/Tutorials/InstallingandConfiguringROSEnvironment - ROS Wiki](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

1. Create and build catkin workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

2. Source catkin workspace setup.bash script

```bash
source ~/catkin_ws/devel/setup.bash
# or (untested but hopefully this works)
echo "source ~/catkin/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### ARIA (ROS internal package for Pioneer 3)

Instructions from this repository: [reedhedges/AriaCoda](https://github.com/reedhedges/AriaCoda)

1. Install requirements (documentation) `sudo apt install doxygen`

2. Install AriaCoda from source

```bash
cd ~/Downloads
git clone https://github.com/reedhedges/AriaCoda
cd AriaCoda
make
sudo make install
```

### AMRISim (Pioneer 3 Simulator)

Instructions from: [GitHub - reedhedges/AMRISim)](https://github.com/reedhedges/AMRISim)

1. Install dependencies

```bash
sudo apt install libroscpp-dev libtf-dev libsensor-msgs-dev libgeometry-msgs-dev libstd-msgs-dev libstd-srvs-dev libnav-msgs-dev libnodelet-dev rviz
```

2.  Install mode dependencies

```bash
sudo apt install python3-roslaunch python3-rostopic ros-geometry-msgs ros-nav-msgs ros-std-msgs ros-sensor-msgs ros-std-srvs
```

3. Install AMRISim from source

```bash
cd ~/Downloads
git clone https://github.com/reedhedges/AMRISim
cd AMRISim
# Define path to AriaCoda before running make
export ARIA=/home/<user>/Downloads/AriaCoda
make
sudo make install
```

## Running a simulator (or real) instance

1. Start ROS (in one terminal window)

```bash
cd ~/catkin_ws
source devel/setup.bash # Always required
roscore # Starts a ROS instance (once per machine)
```

2. Start the Simulator (in another terminal window)

```bash
cd ~/Downloads/AMRISim # or where you downloaded AMRISim
./AMRISIM # Start AMRISim
# Choose a map within the AMRISim folder (there should be a file workspace.map)
```

3. Start ROSAria (in another terminal window)

```bash
cd ~/catkin_ws
rosrun rosaria RosAria
```

4. Send a command to see it things are working (in another terminal window)

```bash
# This should move the robot forwards and rotate
rostopic pub -1 /RosAria/cmd_vel geometry_msgs/Twist '[1.0, 0.0, 0.0]' '[0.0, 0.0, 1.0]'
```

**END OF INSTALLATION**
---

## Setting up a ROS project (package)

### Making a new project

Information sourced from: http://wiki.ros.org/ROS/Tutorials/CreatingPackage

1. Enter the source directory of catkin

```bash
cd ~/catkin_ws/src
```

2. Create a new project

```bash
catkin_create_pkg <package_name> [dependency 1] [dependency 2]
# Example below:
catkin_create_pkg my_package roscpp std_msgs
```

3.  Make the project in the catkin workspace

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

This will create the following file structure:

```dir
catkin_ws/
    src/
        CMakeLists.txt         # Top level Cmake file
        <package_name>/
            CMakeLists.txt     # CMake for package
            package.xml        # Package manifest
```

### Checking project dependencies

```bash
rospack depends1 <package_name> # First order dependencies
rospack depends <package_name> # All dependencies (dependencies of dependencies)
```

---

## Setting up remote robot over TCP

Instructions from: [http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA](http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA)
Ideally the robot should just listen to commands from a central machine. This process involves:

1. On the **robot machine**:
```bash
export ROS_MASTER_URI=http://<ip_address_of_robot>:11311
export ROS_IP=<ip_address_of_robot>

# Add ip address of host to /etc/hosts
sudo vim /etc/hosts
# Add "<ip_address_of_host>		my_workstation" to /etc/hosts

# In a new terminal window
cd ~/catkin_ws
catkin_make
roscore

# In another terminal window
cd ~/catkin_ws
catkin_make
rosrun rosaria RosAria _port:=/dev/ttyS0
```

2. On the **host machine** (do not run roscore here):
```bash
export ROS_MASTER_URI=http://<ip_address_of_robot>:11311
export ROS_HOSTNAME=<ip_address_of_host>
export ROS_IP=<ip_address_of_host>

cd ~/catkin_ws/
catkin_make
rostopic echo /RosAria/pose # Get position from robot
```


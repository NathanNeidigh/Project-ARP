# Project ARP

An Autonomous Racing Prototype built on ROS2 to be run in Microsoft AirSIm

# Development Process

The following instructions describe the setup process for ARP development. The following code is meant to be run in WSL2.

## WSL Setup

In Windows Powershell run WSL and install a new Linux distro and set it as default.

```
wsl --install Ubuntu-20.04
wsl --set-default Ubuntu-20.04
```

You then will be prompted for a username and a password.

After creating a user, navigate to the new user's directory and update Ubuntu.

```
sudo apt update
sudo apt upgrade -y
```

Then install ROS and its needed dependencies by following this [guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html). For understanding of ROS concepts read the [documentation guide](https://docs.ros.org/en/foxy/index.html).

Install colcon.
```
sudo apt install python3-colcon-common-extensions
```

Setup the ros workspace.

```
cd ~
mkdir -p dev_ws/src
cd dev_ws/src
git clone git@github.com:NathanNeidigh/Project-ARP.git
mv Project-ARP/ project-arp
```

### Install Airsim Packages

Navigate back to your home directory and clone AirSim, build, and copy the ros packages into our workspace.

```
git clone https://github.com/Microsoft/AirSim.git
cd Airsim
./setup.sh
./build.sh
cp -r ros2/src/airsim_interfaces/ ~/dev_ws/src/
cp -r ros2/src/airsim_ros_pkgs/ ~/dev_ws/src/
```

Navigate back to your ROS workspace (dev_ws/) and manually install some dependecies

```
sudo apt-get install ros-foxy-geographic-msgs
sudo apt-get install ros-foxy-mavros-msgs
sudo apt-get install libyaml-cpp-dev
```

Then change line 26 of dev_ws/src/airsim_ros_pkgs/CMakeLists.txt to set(AIRSIM_ROOT "$ENV{HOME}/AirSim").

```
colcon build --symlink-install
```

Also install Eigen from their official website.

Changed airsim_ros_wrapper.cpp
auto transformStampedENU = tf_buffer_->lookupTransform(AIRSIM_FRAME_ID, vehicle_name, rclcpp::Time(0), rclcpp::Duration::from_nanoseconds(1)); 
to 
auto transformStampedENU = tf_buffer_->lookupTransform(AIRSIM_FRAME_ID, vehicle_name, rclcpp::Time(0), rclcpp::Duration(std::chrono::nanoseconds(1)));
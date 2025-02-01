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
mkdir dev_ws/src
cd dev_ws/src
git clone git@github.com:NathanNeidigh/SCAR-System.git
cd ..
colcon build --symlink-install
```
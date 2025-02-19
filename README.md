# Project ARP

An Autonomous Racing Prototype built on ROS2 to be run in Microsoft AirSIm

## Development Process

The following instructions describe the setup process for ARP development. The following code is meant to be run in WSL2. A lot of inspiration for this project comes from the FSAE team Technion and there work with AirSim found [here](https://github.com/microsoft/AirSim/wiki/technion).

## AirSim Setup

Following the AirSim build [guide](https://microsoft.github.io/AirSim/build_windows/), however, make sure that you install UE 4.27.

### Import Custom Environment and Car Assets

Follow this [guide](https://github.com/Microsoft/AirSim/wiki/build_FSTDriverless_windows) to import the custom assets into Unreal Engine

### Configure AirSim

Navigate to your documents folder and you should see an AirSim subdirectory. Open the settings.json file in a text editor and change it so that if looks like the following.

```json
{
    "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
    "SettingsVersion": 1.2,
    "SimMode": "Car",
    "Vehicles": {
        "RaceCar": {
            "VehicleType": "PhysXCar",
            "UseSerial": false,
            "UseTcp": true,
            "QgcHostIp": "",
            "TcpPort": 4560,
            "ControlIp": "172.21.80.87",
            "ControlPort": 14580,
            "LocalHostIp": "172.21.80.1",
            "Sensors": {
                "Imu": {
                    "SensorType": 2,
                    "Enabled": true
                }
            }
        }
    }
}
```

`ControlIp` should be set to the IP address of your local machine, and `LocalHostIp` should be set to the IP address of WSL2.

## WSL Setup

In Windows Powershell run WSL and install a new Linux distro and set it as default.

```bash
wsl --install Ubuntu-20.04
wsl --set-default Ubuntu-20.04
```

You then will be prompted for a username and a password.

After creating a user, navigate to the new user's directory and update Ubuntu.

```bash
sudo apt update
sudo apt upgrade -y
```

Then install ROS and its needed dependencies by following this [guide](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html). For understanding of ROS concepts read the [documentation guide](https://docs.ros.org/en/galactic/index.html).

Install colcon.

```bash
sudo apt install python3-colcon-common-extensions
```

Setup the ros workspace.

```bash
cd ~
mkdir -p dev_ws/src
cd dev_ws/src
git clone git@github.com:NathanNeidigh/Project-ARP.git
mv Project-ARP/ project-arp
```

### Install Airsim Packages

Navigate back to your home directory and clone AirSim, build, and copy the ros packages into our workspace.

```bash
git clone https://github.com/Microsoft/AirSim.git
cd Airsim
./setup.sh
./build.sh
source tools/install_ros2_deps.sh
cp -r ros2/src/airsim_interfaces/ ~/dev_ws/src/
cp -r ros2/src/airsim_ros_pkgs/ ~/dev_ws/src/
```

Install Eigen

```bash
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz\
tar -zxvf eigen-3.4.0.tar.gz
cd eigen-3.4.0
mkdir build
cd build
cmake ..
sudo make install
cd ~
rm -r eigen-3.4.0
rm eigen-3.4.0.tar.gz
```

Then open up dev_ws/src/airsim_ros_pkgs/CMakeLists.txt in your text editor and make the following changes:

```CMake
set(AIRSIM_ROOT "$ENV{HOME}/AirSim") # Line 27: routes CMake to where we installed AirSim.
```

### Configure .bashrc script

Open up your .bashrc file in a text editor and append the following lines. Change the `WSL_HOST_IP` to whatever your WSL IP address is.

```bash
source ~/dev_ws/install/setup.bash
cd ~/dev_ws
export WSL_HOST_IP=172.21.80.1 # Set this to whatever your windows system thinks the ip address of WSL is.
```

### Build and run the current deployment

Build the ROS2 workspace

```bash
colcon build --symlink-install
```

Open up the project in Unreal Engine and hit play. Then in a WSL terminal launch the following code to setup the connection between ROS and AirSim.

```bash
ros2 launch airsim_ros_pkgs airsim_node_launch.py output:=screen host:=$WSL_HOST_IP
```

# ABV_Controller

The ABV_Controller module runs on board the air-bearing vehicles and controls the vehicle via PID feedback control

## Dependencies

Install the required dependencies using the command below

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    libyaml-cpp-dev \
    ros-humble-rclcpp \
    ros-humble-ament-cmake
```

## Workspace

Setup a workspace to clone each of the abv software modules in.

```bash
mkdir -p ~/abv_ws/src
```

## Install

Clone this repo into the src directory of the workspace made above.

```bash
cd ~/abv_ws/src && git clone https://github.com/samlovelace/abv_controller.git
```

Then run the clone.sh script to clone the other abv modules.

```bash
cd ~abv_ws/src/abv_controller
chmod +x clone.sh
./clone.sh
```

Confirm that the required repositories were installed alongside the abv_controller repository in the src directory of the workspace.

## Compile

Navigate to the root of the workspace and build the abv modules

```bash
cd ~/abv_ws && colcon build
```

## Run

To run the abv_controller, from the root of the workspace

```bash
source install/setup.bash
ros2 run abv_controller abv_controller
```

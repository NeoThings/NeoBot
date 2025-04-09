### NeoBot

- ROS based differential drive robot

#### Hardware

- X86 Mini PC w/ N5105 CPU
- STM32F407
- IMU
- Lidar
- Depth camera
- Direct Drive Motor

#### Requirements

- Ubuntu 20.04
- ROS Noetic

#### Installation

```bash
# Install dependence
sudo apt install ros-noetic-map-server ros-noetic-serial
```

```bash
cd ${HOME}
mkdir -p neobot_ws/src
cd neobot_ws/src
git clone https://github.com/NeoThings/NeoBot.git
cd NeoBot/neobot_scripts
. install_cartographer.bash
. install_dependencies.bash
. import_gazebo_path.bash
```

```bash
cd ${HOME}/neobot_ws
catkin_make
```

```bash
echo "source ${HOME}/neobot_ws/devel/setup.bash --extend" >> ~/.bashrc
source ~/.bashrc
```


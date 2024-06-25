### StaggerLite

- ROS project of a self made differential drive robot

#### Requirements

- Ubuntu 20.04
- ROS Noetic

#### Installation

```bash
cd ${HOME}
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/NeoThings/StaggerLite.git
cd StaggerLite/staggerlite_scripts
. install_cartographer.bash
. install_navigation.bash
. install_sensors_driver.bash
```

```bash
cd ${HOME}/catkin_ws
catkin_make
```

```bash
echo "source ${HOME}/staggerlite_ws/devel/setup.bash --extend" >> ~/.bashrc
source ~/.bashrc
```

#### Run simulation

```bash
###
```

#### Run real robot

```bash
###
```


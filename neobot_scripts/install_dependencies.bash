#! /bin/bash
cd ..
sudo apt install ros-noetic-tf2-sensor-msgs ros-noetic-move-base-msgs
git clone https://github.com/NeoThings/navigation.git
git clone https://github.com/NeoThings/neobot_sensors.git
git clone https://github.com/NeoThings/neobot_manager.git
git clone https://github.com/NeoThings/neobot_comm.git
#git clone https://github.com/NeoThings/neobot_models.git

mkdir neobot_kits
cd neobot_kits
git clone https://github.com/NeoThings/teleop_tools.git
git clone https://github.com/NeoThings/teleop_twist_keyboard.git
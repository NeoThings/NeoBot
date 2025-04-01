#! /bin/bash
# echo "export GAZEBO_MODEL_PATH=${HOME}/.gazebo/models/bookstore/:
#                                ${HOME}/.gazebo/models/hospital:
#                                ${HOME}/.gazebo/models/small_house:${GAZEBO_MODEL_PATH}" >> ~/.bashrc

mkdir -p ${HOME}/.gazebo/models/bookstore/models
mkdir -p ${HOME}/.gazebo/models/hospital/models
mkdir -p ${HOME}/.gazebo/models/small_house/models

cp -r ../neobot_models/worlds/bookstore/models/* ${HOME}/.gazebo/models/bookstore/models
cp -r ../neobot_models/worlds/hospital/models/* ${HOME}/.gazebo/models/hospital/models
cp -r ../neobot_models/worlds/small_house/models/* ${HOME}/.gazebo/models/small_house/models

echo "export GAZEBO_MODEL_PATH=${HOME}/.gazebo/models/bookstore/models/:\
${HOME}/.gazebo/models/hospital/:\
${HOME}/.gazebo/models/small_house/models/" >> ~/.bashrc

echo "export GAZEBO_RESOURCE_PATH=${HOME}/.gazebo/models/bookstore/:\
${HOME}/.gazebo/models/small_house/" >> ~/.bashrc
#! /bin/bash

echo "╔══╣ Install: SIGVERSE ROS BRIDGE (STARTING) ╠══╗"

# Keep track of the current directory
DIR=`pwd`

# Install mongo_c
mkdir ~/mongo_c
cd ~/mongo_c/
wget https://github.com/mongodb/mongo-c-driver/releases/download/1.4.2/mongo-c-driver-1.4.2.tar.gz
tar zxvf mongo-c-driver-1.4.2.tar.gz
cd mongo-c-driver-1.4.2/
./configure
make
sudo make install

# Install mongo_cpp
mkdir ~/mongo_cpp
cd ~/mongo_cpp/
wget https://github.com/mongodb/mongo-cxx-driver/archive/r3.0.3.tar.gz
tar zxvf r3.0.3.tar.gz
cd mongo-cxx-driver-r3.0.3/build/
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DLIBMONGOC_DIR=/usr/local -DLIBBSON_DIR=/usr/local ..
sudo make EP_mnmlstc_core
make
sudo make install

# Install rosbridge_suite
cd $DIR/../..
git clone -b humble https://github.com/RobotWebTools/rosbridge_suite.git

# Install ROS dependecies
rosdep update
rosdep install --from-paths src --ignore-src

# Install Python dependecies
pip3 install --upgrade pip
pip3 install pymongo
pip3 install tornado

cd $DIR

echo "╚══╣ Install: SIGVERSE ROS BRIDGE (FINISHED) ╠══╝"
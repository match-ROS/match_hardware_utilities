#!/usr/bin/env bash
# Check if ROS version was passed
if [ $# -ne 1 ]; then
    echo "Error: Incorrect number of arguments."
    echo "Usage: $0 <ROS1|ROS2|humble>"
    exit 1
fi

# Check if the argument is valid
if [ "$1" != "ROS1" ] && [ "$1" != "ROS2" ] && [ "$1" != "humble" ]; then
    echo "Error: Invalid argument."
    echo "Usage: $0 <ROS1|ROS2|humble>"
    exit 1
fi

cd `dirname $0`
CURRENT_DIR=$(pwd)
echo "Running script from path: "$CURRENT_DIR

#build Livox-SDK2 at root folder (See https://github.com/Livox-SDK/Livox-SDK2)
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
echo 'Finished building Livox-SDK2' 

#build Livox ROS driver 2 (Modified build.sh, see https://github.com/Livox-SDK/livox_ros_driver2)
cd $CURRENT_DIR
cd ..
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2

source /opt/ros/noetic/setup.sh

readonly VERSION_ROS1="ROS1"
readonly VERSION_ROS2="ROS2"
readonly VERSION_HUMBLE="humble"

pushd `pwd` > /dev/null

ROS_VERSION=""
ROS_HUMBLE=""

# Set working ROS version
if [ "$1" = "ROS2" ]; then
    ROS_VERSION=${VERSION_ROS2}
elif [ "$1" = "humble" ]; then
    ROS_VERSION=${VERSION_ROS2}
    ROS_HUMBLE=${VERSION_HUMBLE}
elif [ "$1" = "ROS1" ]; then
    ROS_VERSION=${VERSION_ROS1}
else
    echo "Invalid Argument"
    exit
fi
echo "ROS version is: "$ROS_VERSION

# clear `build/` folder.
# TODO: Do not clear these folders, if the last build is based on the same ROS version.
rm -rf ../../build/
rm -rf ../../devel/
rm -rf ../../install/
# clear src/CMakeLists.txt if it exists.
if [ -f ../CMakeLists.txt ]; then
    rm -f ../CMakeLists.txt
fi

# exit

# substitute the files/folders: CMakeList.txt, package.xml(s)
if [ ${ROS_VERSION} = ${VERSION_ROS1} ]; then
    if [ -f package.xml ]; then
        rm package.xml
    fi
    cp -f package_ROS1.xml package.xml
elif [ ${ROS_VERSION} = ${VERSION_ROS2} ]; then
    if [ -f package.xml ]; then
        rm package.xml
    fi
    cp -f package_ROS2.xml package.xml
    cp -rf launch_ROS2/ launch/
fi

# build
pushd `pwd` > /dev/null
if [ $ROS_VERSION = ${VERSION_ROS1} ]; then
    cd ../../
    catkin_make -DROS_EDITION=${VERSION_ROS1}
elif [ $ROS_VERSION = ${VERSION_ROS2} ]; then
    cd ../../
    colcon build --cmake-args -DROS_EDITION=${VERSION_ROS2} -DHUMBLE_ROS=${ROS_HUMBLE}
fi
popd > /dev/null

# remove the substituted folders/files
if [ $ROS_VERSION = ${VERSION_ROS2} ]; then
    rm -rf launch/
fi

popd > /dev/null

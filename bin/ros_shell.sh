DATA_DIR=$(pwd)/data
echo "mounting data in "$DATA_DIR

docker run -it -v $DATA_DIR:/data ubuntu_ros:latest /bin/bash


# 1. check if envs are set
# printenv | grep ROS
#
# 2. Check out what's in the rosbag
# rosbag info /data/dataset.bag

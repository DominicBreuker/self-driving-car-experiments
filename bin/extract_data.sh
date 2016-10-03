DATA_DIR=$(pwd)/data
echo "mounting data in "$DATA_DIR

docker run -v $DATA_DIR:/data ubuntu_ros:latest bin/bash -c "source /opt/ros/jade/setup.bash ; python /extract_data.py"

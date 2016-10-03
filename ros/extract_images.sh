# prepare the shell
source /opt/ros/jade/setup.bash

# start roscore
roscore &

# create folders for pictures and start image extractors
LEFT_DIR=/data/left_camera
CENTER_DIR=/data/center_camera
RIGHT_DIR=/data/right_camera

mkdir -p $LEFT_DIR && cd $LEFT_DIR
rosrun image_view extract_images image:=/left_camera/image_color &

mkdir -p $CENTER_DIR && cd $CENTER_DIR
rosrun image_view extract_images image:=/center_camera/image_color &

mkdir -p $RIGHT_DIR && cd $RIGHT_DIR
rosrun image_view extract_images image:=/right_camera/image_color &

# play rosbag file - images should appear now in TARGET_DIR
rosbag play /data/dataset.bag

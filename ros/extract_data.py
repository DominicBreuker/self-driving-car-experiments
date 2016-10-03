import os
import rosbag
import cv2
import csv
from cv_bridge import CvBridge, CvBridgeError
from bisect import bisect

data_dir = "data"
rosbag_file = os.path.join(data_dir, "dataset.bag")


def get_image_dir(image_type):
    images_dir = os.path.join(data_dir, image_type)
    if not os.path.exists(images_dir):
        os.makedirs(images_dir)
    return images_dir

left_images_dir = get_image_dir("left")
center_images_dir = get_image_dir("center")
right_images_dir = get_image_dir("right")

steering_angle_file = os.path.join(data_dir, "steering_angles.csv")

steering_report_topic = "/vehicle/steering_report"
left_camera_topic = "/left_camera/image_color"
center_camera_topic = "/center_camera/image_color"
right_camera_topic = "/right_camera/image_color"
topics = [steering_report_topic, left_camera_topic,
          center_camera_topic, right_camera_topic]

angle_timestamps = []
angle_values = []

left_image_timestamps = []
center_image_timestamps = []
right_image_timestamps = []

counter = 0

bridge = CvBridge()


def try_write_image(dir, msg):
    image_name = os.path.join(dir, str(msg.header.stamp.to_nsec()) + ".pgm")
    try:
        cv2.imwrite(image_name, bridge.imgmsg_to_cv2(msg))
    except CvBridgeError as e:
        print(e)

with rosbag.Bag(rosbag_file, "r") as bag:
    for topic, msg, t in bag.read_messages(topics=topics):
        if topic == steering_report_topic:
            angle_timestamps.append(msg.header.stamp.to_nsec())
            angle_values.append(msg.steering_wheel_angle)
        elif topic == left_camera_topic:
            left_image_timestamps.append(msg.header.stamp.to_nsec())
            try_write_image(left_images_dir, msg)
        elif topic == center_camera_topic:
            center_image_timestamps.append(msg.header.stamp.to_nsec())
            try_write_image(center_images_dir, msg)
        elif topic == right_camera_topic:
            right_image_timestamps.append(msg.header.stamp.to_nsec())
            try_write_image(right_images_dir, msg)
        counter += 1
        if counter >= 100:
            break


# make sure timestamps are in ascending chronological order
def assert_sorted(l):
    assert all(l[i] <= l[i+1] for i in xrange(len(l)-1))

assert_sorted(angle_timestamps)
assert_sorted(left_image_timestamps)
assert_sorted(center_image_timestamps)
assert_sorted(right_image_timestamps)

# make sure we have an angle value for each initial image
assert (angle_timestamps[0] < left_image_timestamps[0])
assert (angle_timestamps[0] < center_image_timestamps[0])
assert (angle_timestamps[0] < right_image_timestamps[0])

angles_at_timestamps = {}


def get_angles_at_timestamps(image_timestamps):
    angle_idx = 0
    for image_idx in xrange(len(image_timestamps)-1):
        # go through angle values until we reach current image time
        while angle_timestamps[angle_idx] <= image_timestamps[image_idx]:
            angle_idx += 1
        # at any image timestamp, use the last known steering angle
        angles_at_timestamps[image_timestamps[image_idx]] = \
            angle_values[angle_idx - 1]

get_angles_at_timestamps(left_image_timestamps)
get_angles_at_timestamps(center_image_timestamps)
get_angles_at_timestamps(right_image_timestamps)


with open(steering_angle_file, "w") as angles_file:
    fieldnames = ["timestamp", "angle"]
    writer = csv.DictWriter(angles_file, fieldnames=fieldnames)
    writer.writeheader()
    for timestamp in sorted(angles_at_timestamps.keys()):
        writer.writerow({"timestamp": timestamp,
                         "angle": angles_at_timestamps[timestamp]})

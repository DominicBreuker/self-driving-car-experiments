# Self-driving car experiments

Udacity is hosting a [self-driving car challenge](https://medium.com/udacity/challenge-2-using-deep-learning-to-predict-steering-angles-f42004a36ff3#.avqvc84n3) in which your goal is to predict the steering wheel angles from camera images.
This repo contains code for some experiments with the data.

## Getting data

The data is available [here](http://bit.ly/udacity-dataset-2-1).
Use `bin/get_data.sh` to download and extract the data.
You will get a [rosbag file](http://wiki.ros.org/Bags) which you can use together with [ROS](http://www.ros.org/) to explore the data.
To create a dataset for prediction, you can use the Ubuntu-based ROS docker image found in `ros`.

### Quick and easy - use image from Docker hub

To get up and running quickly you only need docker.
Get it with `curl -fsSL https://get.docker.com/ | sh`, then do:

```bash
cd <some folder>
mkdir data
mv path/to/rosbag/dataset.bag data/dataset.bag # put dataset.bag into data folder
docker run -v $(pwd)/data:/data dominicbreuker/ubuntu_ros:latest bin/bash -c "source /opt/ros/jade/setup.bash ; python /extract_data.py"
```

This will create three folders `data/left`, `data/center`, and `data/right` with all the images.
Image filenames will be `<timestamp>.pgm`.
You also get a file `data/steering_angles.csv` containing the angles for each timestep (= target variables)

### DIY - build the docker image locally

Run `bin/build_docker_image.sh` to build the docker image locally.
After that, use `bin/extract_data.sh` to get the data.

## Exploring data

To be done... :)

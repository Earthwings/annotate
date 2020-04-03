# annotate
The evaluation of perception algorithms requires labeled ground truth data. Supervised machine learning algorithms need labeled training data. Annotate is a tool to create 3D labeled bounding boxes in ROS/RViz. Its labeled data is useful both as training data for machine learning algorithms as well as ground truth for evaluation.

## Requirements
Annotate is useful in the following contexts:
* You are using ROS. Annotate was tested with ROS Kinetic (Ubuntu 16.04) and ROS Melodic (Ubuntu 18.04).
* Your data is available in (or can easily be converted to) [3D point clouds](http://wiki.ros.org/pcl).

## Overview
Create labeled data using RViz. Annotate provides RViz extensions for labeling. You are responsible to provide labeling data using a ROS topic with [sensor_msgs/PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html) data.
* Use ```rosbag play``` or similar tooling to provide labeling data in ```sensor_msgs/PointCloud2``` format
* Start ```annotation_node``` and ```rviz``` to create and edit annotation tracks
* Labeled data is stored in a YAML file of your choice
See [labeling](docs/labeling.md) for a detailled description of label creation.

## Getting Started
Clone this repository in a ROS workspace to build and run annotate. For a successful build please install the dependency pcl-ros as well. You can also use rosdep to pick it up for you.

```bash
cd /path/to/your/workspace/src
git clone https://github.com/Earthwings/annotate.git
catkin build  # or catkin_make or your other favorite build method
source /path/to/your/workspace/devel/setup.bash  # or setup.zsh for ZSH users
```

Use the ```demo.launch``` launch file shipped with annotate to see it in action quickly. Here is a sample call:

```bash
roslaunch annotate demo.launch \
  bag:="/kitti/2011_09_26_drive_0005_sync_pointcloud.bag --pause-topics /velodyne_points" \
  pointcloud_topic:=/velodyne_points \
  annotations:=/kitti/2011_09_26_drive_0005_sync_pointcloud.yaml
```
The above call will run ```rosbag play``` in a separate xterm window. Annotate is told that the topic with to be labeled data is ```/velodyne_points```, and that it should save labels in the file ```/kitti/2011_09_26_drive_0005_sync_pointcloud.yaml```. Furthermore ```annotate_node``` and RViz are started. The RViz window will look similar like this:

![RViz/Annotate screenshot](docs/rviz-full.png "RViz screenshot with annotate")

Please see [labeling](docs/labeling.md) for a detailed description of label creation.

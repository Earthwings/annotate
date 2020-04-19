# Labeling with annotate

## Quick start
Let's start with a quick introduction for the impatient. It will cover the basic annotation steps. The rest of the document will cover these steps in more details, including tips and tricks for efficient annotation.

We will use a real-world labeling example taken from the [KITTI Vision Benchmark Suite](http://www.cvlibs.net/datasets/kitti/). The bag file we use is a 16 seconds long traffic sequence called 2011_09_26_drive_0005.bag created using [kitti_to_rosbag](https://github.com/ethz-asl/kitti_to_rosbag). It provides lidar data from a Velodyne HDL-64 in the velodyne_points topic. Our goal is to label the cars, bicycles and pedestrians in it. We can tell the ```demo.launch``` launch file about this setting:

```bash
roslaunch annotate demo.launch \
  bag:="/workspace/kitti/2011_09_26/2011_09_26_drive_0005_sync_pointcloud.bag --pause-topics velodyne_points" \
  pointcloud_topic:=/velodyne_points \
  labels:=person,car,bicycle \
  annotations:=/kitti/2011_09_26_drive_0005.yaml
```

Two windows will open:
* One RViz window for data labeling
* One xterm window to control data playback. Press <kbd>space bar</kbd> to move to the next pointcloud data. Press it twice to see the first pointcloud in RViz.

You are ready to create the first annotation now. The next section explains that in detail. It uses the cyclist in the beginning of the scene as its annotation object. Do you want to follow up in parallel? Then please spot the cyclist in the scene and focus the RViz view on it before reading on.

## Creating a new annotation
After you have spotted a new object for annotation, create a new annotation for it in two steps:
* Click on the **New Annotation** button in the RViz tools panel.
* Click on the object in the pointcloud.

Annotate will create a new annotation at the point you clicked in the scene. It has a default size and no label attached yet like in the following video:

![RViz screenshot for creating a new annotation](new-annotation.gif "Creating a new annotation")



Assign a label for the new annotation using the context menu.
* Right-click on the annotation cube (bounding box).
* Select **car** in the **Label** menu of the context menu.
* Note how the annotation description above the annotation cube now includes the car label.

## Summary
In order to create a new annotation for an object that has not been annotated yet in the scene, follow these steps:

1. Spot the object in RViz and focus the view on it.
2. Click on the '''New Annotation''' button.
3. Click on the object in the RViz view.
4. Right-click on the object's annotation box and assign a label.
5. Change the RViz view to see the object from top (birds-eye view).
6. Drag the black ring to move the annotation box. Align its center with the object's center
7. Click on the object's annotation box to switch from move to rotation mode. The formerly black ring becomes blue.
8. Drag the blue ring to align the red axis with the object's front
9. Click on the object's annotation box once again to switch from rotation mode to resize mode. The blue ring is replaced with six colored resize handles (arrows).
10. Use the resize handles to roughly align the objects' annotation box size with the size of the object.
11. Right-click on the annotation box and choose **Auto-fit Box** from the **Edit** menu.
12. Perform a sanity check of the annotation box: The red axis should be aligned with the object front. All object points have to be within the annotation box. The annotation box should be tight around the object points.
13. If the sanity check fails, repeat the steps above to correct the found problem.
14. If the sanity check succeeds, right-click on the annotation box and choose **Commit**. The annotation box turns green.

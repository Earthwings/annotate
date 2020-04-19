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

## Assigning a label

Assign a label for the new annotation using the context menu.
* Right-click on the annotation cube (bounding box).
* Select a label in the **Label** menu of the context menu.
* Note how the annotation description above the annotation cube now includes the chosen label.

In the following video the bicyclist is labeled ```bicycle```:

![Assign a label to an annotation](set-label.gif "Assign a label to an annotation")

A label is carried over to subsequent annotations of the same object. You can, however, change an annotation's label at any time. This is useful, for example, to mark special situations like partial occlusions of objects. Consult your teams annotation guidelines for instructions on special situations.

## Bounding box alignment

The position and size of an annotation's bounding box is crucial information. A carefully aligned bounding box:

* Contains all points of the object.
* Tightly fits all object points such that the distance between the outmost points and the annotation box walls is small in all dimensions.
* Has the red axis point towards the primary direction of the object.

The primary direction of an object can mean different things. For example, for a car it could mean its front. But it could also mean its driving direction &mdash; its front for forward driving and standstill and its rear for reverse driving. Consult your teams annotation guidelines to learn about the meaning of the primary direction of each object type.

Follow these steps to get a carefully aligned bounding box:

1. Change the annotation's **box mode** to **Move**. A black circle around the annotation box appears.
2. Change the RViz view to see the object from top (birds-eye view).
3. Drag the black ring to move the annotation box. Align its center with the object's center
4. Click on the object's annotation box to switch from move to rotation mode. The formerly black ring becomes blue.
5. Drag the blue ring to align the red axis with the object's front
6. Click on the object's annotation box once again to switch from rotation mode to resize mode. The blue ring is replaced with six colored resize handles (arrows).
7. Use the resize handles to roughly align the objects' annotation box size with the size of the object.
8. Right-click on the annotation box and choose **Auto-fit Box** from the **Edit** menu.

While the list of steps may seem daunting at first sight, the process is actually a quick one as the following video shows:

![Bounding box alignment](align-bounding-box.gif "Align the bounding box of an annotation")

Please note the annotation's description text above the box in the end of the video. It reads

> ```bicycle #1```<br />
> ```1.80 (+0.05) x 0.60 (+0.05) x 1.66 (+0.05)```<br/>
> ```841 points inside, 0 nearby```

The second line tells us that our box now has a length of 1.80 m, a width of 0.60 m and a height of 1.66 m. It also tells us that for each of those dimensions, the box could be shrinked by just 0.05 m. This is because the *auto-fit* feature (just like the *shrink to points* feature) leaves a margin of 0.025 m (2.5 cm) to every side. This means the box is a tight fit around all points inside.

The third line tells us that there are 841 points inside the annotation box, and zero points in a distance of up to 0.25 m to it. Visual inspection also shows that we did not forget any point in the vicinity of the object.

You can conclude that an annotation box is a well-aligned box if the following holds:

* The second line of the annotation description reads ```(+0.05)``` for all three dimensions &mdash; we have a tight fit.
* Visual inspection shows that there are no object points outside of the annotation box.

## Committing

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

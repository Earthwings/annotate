#pragma once

#include "annotation_marker.h"
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/tf.h>
#include <geometry_msgs/PointStamped.h>
#include <interactive_markers/menu_handler.h>
#include <memory>
#include <stack>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <QTime>
#include <limits>

namespace annotate
{

class Markers
{
public:
  Markers();
  bool save() const;
  void publishTrackMarkers();
  sensor_msgs::PointCloud2ConstPtr cloud() const;
  tf::TransformListener& transformListener();

private:
  void load();
  void createNewAnnotation(const geometry_msgs::PointStamped::ConstPtr& message);
  void handlePointcloud(const sensor_msgs::PointCloud2ConstPtr& cloud);

  ros::NodeHandle node_handle_;
  ros::Subscriber new_annotation_subscriber_;
  ros::Subscriber pointcloud_subscriber_;
  ros::Publisher track_marker_publisher_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  size_t current_marker_id_{ 0 };
  std::vector<AnnotationMarker::Ptr> markers_;
  std::vector<std::string> labels_;
  std::string filename_;
  ros::Time time_;
  ros::Time last_track_publish_time_;
  sensor_msgs::PointCloud2ConstPtr cloud_;
  tf::TransformListener transform_listener_;
  bool ignore_ground_{ false };
};
}  // namespace annotate
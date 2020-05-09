#pragma once

#include "annotation_marker.h"
#include "file_dialog_property.h"
#include "shortcut_property.h"
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
#include <rviz/display_group.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <functional>

namespace annotate
{
class CloudDisplay;
class MarkerDisplay;
class TrackDisplay;

class AnnotateDisplay : public rviz::DisplayGroup
{
  Q_OBJECT
public:
  AnnotateDisplay();
  void onInitialize() override;
  void setTopic(const QString& topic, const QString& datatype) override;
  void load(const rviz::Config& config) override;
  void setCurrentMarker(AnnotationMarker* marker);

  bool save();
  void publishTrackMarkers();
  sensor_msgs::PointCloud2ConstPtr cloud() const;
  tf::TransformListener& transformListener();

private Q_SLOTS:
  void updateTopic();
  void updateLabels();
  void openFile();
  void updateAnnotationFile();
  void updateIgnoreGround();
  void autoFitPoints();
  void undo();
  void commit();
  void rotateClockwise();
  void rotateAntiClockwise();
  void togglePlayPause();
  void updateShortcuts();

private:
  template <class T>
  void modifyChild(rviz::Property* parent, QString const& name, std::function<void(T*)> modifier);
  void adjustView();
  bool load(std::string const& file);
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
  rviz::RosTopicProperty* topic_property_{ nullptr };
  rviz::BoolProperty* ignore_ground_property_{ nullptr };
  rviz::StringProperty* labels_property_{ nullptr };
  FileDialogProperty* open_file_property_{ nullptr };
  FileDialogProperty* annotation_file_property_{ nullptr };
  rviz::Display* cloud_display_{ nullptr };
  rviz::Display* marker_display_{ nullptr };
  rviz::Display* track_display_{ nullptr };
  AnnotationMarker* current_marker_{ nullptr };
  BoolProperty* shortcuts_property_{ nullptr };
  ros::ServiceClient playback_client_;
};
}  // namespace annotate

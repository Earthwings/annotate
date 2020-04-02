#pragma once

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/tf.h>
#include <geometry_msgs/PointStamped.h>
#include <interactive_markers/menu_handler.h>
#include <memory>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace annotate
{
struct BoxSize
{
  explicit BoxSize(double length = 0.0, double width = 0.0, double height = 0.0);
  double length{ 0.0 };
  double width{ 0.0 };
  double height{ 0.0 };
};

struct TrackInstance
{
  std::string label;
  tf::StampedTransform center;
  BoxSize box;
};

using Track = std::vector<TrackInstance>;

class Markers;

class AnnotationMarker
{
public:
  using Ptr = std::shared_ptr<AnnotationMarker>;

  AnnotationMarker(Markers* markers, const std::shared_ptr<interactive_markers::InteractiveMarkerServer>& server,
                   const TrackInstance& trackInstance, int marker_id, const std::vector<std::string>& labels);

  int id() const;
  Track const& track() const;
  void setTrack(const Track& track);

  void setTime(const ros::Time& time);

private:
  enum Mode
  {
    Locked,
    Move,
    Scale
  };

  enum State
  {
    New,
    Committed,
    Modified
  };

  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void nextMode();
  void changeSize(const tf::Pose& new_pose);
  void lock();
  void lock(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void changeScale();
  void changeScale(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void changePosition();
  void changePosition(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void createMarker(const TrackInstance& instance);
  void setLabel(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void commit(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void updateDescription();
  void updateState(State state);
  void expand(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void shrink(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void pull();
  void push();
  void removeControls();
  void createCubeControl();
  void createPositionControl();
  void createScaleControl();
  void setBoxSize(geometry_msgs::Vector3& scale, const BoxSize& box_size);

  visualization_msgs::InteractiveMarker marker_;
  interactive_markers::MenuHandler menu_handler_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  Mode mode_{ Move };
  bool can_change_size_{ false };
  tf::Pose last_pose_;
  tf::Point last_mouse_point_;
  int id_{ -1 };
  std::map<interactive_markers::MenuHandler::EntryHandle, std::string> labels_;
  std::string label_;
  Track track_;
  Markers* markers_;
  ros::Time time_;
  tf::TransformBroadcaster tf_broadcaster_;
  State state_{ New };
};

class Markers
{
public:
  Markers();
  void save() const;
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
};
}  // namespace annotate
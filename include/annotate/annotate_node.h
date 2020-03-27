#pragma once

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/tf.h>
#include <geometry_msgs/PointStamped.h>
#include <interactive_markers/menu_handler.h>
#include <memory>

namespace annotate
{
class AnnotationMarker
{
public:
  using Ptr = std::shared_ptr<AnnotationMarker>;

  AnnotationMarker(const std::shared_ptr<interactive_markers::InteractiveMarkerServer>& server,
                   const tf::Vector3& position, const std::string& frame_id, int marker_id,
                   const std::vector<std::string>& labels);

private:
  enum Mode
  {
    Locked,
    Move,
    Scale
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
  void createMarker(const tf::Vector3& position, const std::string& frame_id);
  void addMarker(visualization_msgs::InteractiveMarker& int_marker);
  void setLabel(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  interactive_markers::MenuHandler menu_handler_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  Mode mode_{ Move };
  bool can_change_size_{ false };
  tf::Pose last_pose_;
  tf::Point last_mouse_point_;
  int id_{ -1 };
  std::string name_;
  std::map<interactive_markers::MenuHandler::EntryHandle, std::string> labels_;
  std::string label_;
};

class Markers
{
public:
  Markers();

private:
  void createNewAnnotation(const geometry_msgs::PointStamped::ConstPtr& message);

  ros::NodeHandle node_handle_;
  ros::Subscriber subscriber_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  size_t current_marker_id_{ 0 };
  std::vector<AnnotationMarker::Ptr> markers_;
  std::vector<std::string> labels_;
};
}  // namespace annotate
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
#include <tf/transform_listener.h>
#include <QTime>
#include <limits>

namespace annotate
{
void setRotation(geometry_msgs::Quaternion& quaternion, double x, double y, double z);

struct TrackInstance
{
  std::string label;
  std::vector<std::string> tags;
  tf::StampedTransform center;
  tf::Vector3 box_size;

  double timeTo(ros::Time const& time) const;
};

using Track = std::vector<TrackInstance>;

class AnnotateDisplay;

class AnnotationMarker
{
public:
  using Ptr = std::shared_ptr<AnnotationMarker>;

  AnnotationMarker(AnnotateDisplay* markers,
                   const std::shared_ptr<interactive_markers::InteractiveMarkerServer>& server,
                   const TrackInstance& trackInstance, int marker_id);

  int id() const;
  Track const& track() const;
  void setTrack(const Track& track);
  void setPadding(double padding);
  void setMargin(double margin);
  void setIgnoreGround(bool enabled);
  void setLabels(const std::vector<std::string>& labels);
  void setTags(const std::vector<std::string>& tags);

  void setTime(const ros::Time& time);
  void shrinkToPoints();
  void autoFit();
  void undo();
  void commit();
  void rotateYaw(double delta_rad);

private:
  using MenuHandler = interactive_markers::MenuHandler;

  enum Mode
  {
    Locked,
    Move,
    Rotate,
    Resize
  };

  enum State
  {
    Hidden,
    New,
    Committed,
    Modified
  };

  struct UndoState
  {
    QTime time;
    std::string undo_description;

    geometry_msgs::Pose pose;
    tf::Vector3 box_size;
    std::string label;
    std::vector<std::string> tags;
    State state;
  };

  struct PointContext
  {
    ros::Time time;
    size_t points_inside{ 0u };
    size_t points_nearby{ 0u };
    tf::Vector3 minimum{ std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                         std::numeric_limits<float>::max() };
    tf::Vector3 maximum{ std::numeric_limits<float>::min(), std::numeric_limits<float>::min(),
                         std::numeric_limits<float>::min() };
  };

  void updateMenu(const PointContext& context);
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void nextMode();
  void changeSize(const tf::Pose& new_pose);
  void lock();
  void lock(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void enableResizeControl();
  void enableResizeControl(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void enableMoveControl();
  void enableMoveControl(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void enableRotationControl();
  void enableRotationControl(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void createMarker(const TrackInstance& instance);
  void setLabel(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void setTag(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void commit(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void updateDescription(const PointContext& context);
  void updateState(State state);
  bool hasMoved(geometry_msgs::Pose const& a, geometry_msgs::Pose const& b) const;
  void saveMove();
  void saveForUndo(const std::string& description);
  void undo(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void resize(double offset);
  PointContext analyzePoints() const;
  void shrinkTo(const PointContext& context);
  void shrink(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  bool fitNearbyPoints();
  void autoFit(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void pull();
  void push();
  void removeControls();
  void createCubeControl();
  void createMoveControl();
  void createRotationControl();
  void createResizeControl();
  void setBoxSize(const tf::Vector3& box_size);
  tf::Vector3 boxSize() const;

  visualization_msgs::InteractiveMarker marker_;
  MenuHandler menu_handler_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  Mode mode_{ Move };
  bool can_change_size_{ false };
  tf::Pose last_pose_;
  bool button_click_active_{ false };
  tf::Point last_mouse_point_;
  int id_{ -1 };
  std::vector<std::string> label_keys_;
  std::map<MenuHandler::EntryHandle, std::string> labels_;
  std::string label_;
  std::vector<std::string> tag_keys_;
  std::map<MenuHandler::EntryHandle, std::string> tags_menu_;
  std::vector<std::string> tags_;
  Track track_;
  AnnotateDisplay* annotate_display_;
  ros::Time time_;
  State state_{ Hidden };
  std::stack<UndoState> undo_stack_;
  bool ignore_ground_{ false };
  double padding_ { 0.05 };
  double margin_ { 0.25 };
};

}  // namespace annotate

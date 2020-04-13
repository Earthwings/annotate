#pragma once

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
struct TrackInstance
{
  std::string label;
  tf::StampedTransform center;
  tf::Vector3 box_size;

  double timeTo(ros::Time const& time) const;
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
  void setIgnoreGround(bool enabled);

  void setTime(const ros::Time& time);

private:
  using MenuHandler = interactive_markers::MenuHandler;

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

  struct UndoState
  {
    QTime time;
    std::string undo_description;

    geometry_msgs::Pose pose;
    tf::Vector3 box_size;
    std::string label;
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

  struct Automation
  {
    enum State
    {
      Disabled,
      Enabled
    };

    Automation(const std::string& title, State initial_state);

    std::string const title;
    bool enabled{ false };
    MenuHandler::EntryHandle handle;
    AnnotationMarker* annotation_marker{ nullptr };
    MenuHandler* menu_handler{ nullptr };

    void update(MenuHandler* menu_handler, const MenuHandler::EntryHandle& parent);
    void updateState(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  };

  struct Automations
  {
    Automation auto_fit_after_predict{ "After Time Changes: Auto-fit Box", Automation::Disabled };
    Automation shrink_after_resize{ "After Resizing: Shrink to Points", Automation::Enabled };
    Automation shrink_before_commit{ "Before Committing: Shrink to Points", Automation::Enabled };
  };

  void updateMenu(const PointContext& context);
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
  void updateDescription(const PointContext& context);
  void updateAutomations();
  void updateState(State state);
  bool hasMoved(geometry_msgs::Pose const& a, geometry_msgs::Pose const& b) const;
  void saveMove();
  void saveForUndo(const std::string& description);
  void undo();
  void undo(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void resize(double offset);
  PointContext analyzePoints() const;
  void expand(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void shrinkTo(const PointContext& context);
  void shrink(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  bool autoFit();
  void autoFit(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void pull();
  void push();
  void removeControls();
  void createCubeControl();
  void createPositionControl();
  void createScaleControl();
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
  Track track_;
  Markers* markers_;
  ros::Time time_;
  tf::TransformBroadcaster tf_broadcaster_;
  State state_{ New };
  std::stack<UndoState> undo_stack_;
  bool ignore_ground_{ false };
  Automations automations_;
};

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
#include <annotate/annotation_marker.h>
#include <annotate/annotate_display.h>
#include <sstream>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <QColor>
#include <random>

using namespace visualization_msgs;
using namespace interactive_markers;
using namespace tf;
using namespace std;

namespace annotate
{
namespace internal
{
template <typename T>
int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

Marker createCube(float scale)
{
  Marker marker;
  marker.type = Marker::CUBE;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 0.7;
  return marker;
}

}  // namespace internal

void setRotation(geometry_msgs::Quaternion& quaternion, double x, double y, double z)
{
  tf::Quaternion orientation(x, y, z, 1.0);
  orientation.normalize();
  quaternionTFToMsg(orientation, quaternion);
}

double TrackInstance::timeTo(ros::Time const& time) const
{
  return fabs((time - center.stamp_).toSec());
}

void AnnotationMarker::removeControls()
{
  marker_.controls.resize(1);
}

void AnnotationMarker::createCubeControl()
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(internal::createCube(marker_.scale - 0.2));
  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  marker_.controls.push_back(control);
}

void AnnotationMarker::createMoveControl()
{
  removeControls();
  InteractiveMarkerControl control;
  setRotation(control.orientation, 0.0, 1.0, 0.0);
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  marker_.controls.push_back(control);
}

void AnnotationMarker::createRotationControl()
{
  removeControls();
  InteractiveMarkerControl control;
  setRotation(control.orientation, 0.0, 1.0, 0.0);
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  marker_.controls.push_back(control);
}

void AnnotationMarker::createResizeControl()
{
  removeControls();
  InteractiveMarkerControl control;

  setRotation(control.orientation, 1.0, 0.0, 0.0);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker_.controls.push_back(control);

  setRotation(control.orientation, 0.0, 1.0, 0.0);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker_.controls.push_back(control);

  setRotation(control.orientation, 0.0, 0.0, 1.0);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker_.controls.push_back(control);
}

Vector3 AnnotationMarker::boxSize() const
{
  Vector3 box_size;
  if (!marker_.controls.empty() && !marker_.controls.front().markers.empty())
  {
    auto& box = marker_.controls.front().markers.front();
    vector3MsgToTF(box.scale, box_size);
  }
  return box_size;
}

void AnnotationMarker::setBoxSize(const Vector3& box_size)
{
  if (!marker_.controls.empty() && !marker_.controls.front().markers.empty())
  {
    auto& box = marker_.controls.front().markers.front();
    vector3TFToMsg(box_size, box.scale);
  }
  marker_.scale = 0.2 + box_size[box_size.maxAxis()];
}

AnnotationMarker::AnnotationMarker(AnnotateDisplay* annotate_display, const shared_ptr<InteractiveMarkerServer>& server,
                                   const TrackInstance& trackInstance, int marker_id)
  : server_(server), id_(marker_id), annotate_display_(annotate_display)
{
  time_ = trackInstance.center.stamp_;
  createMarker(trackInstance);
}

void AnnotationMarker::setLabels(const std::vector<std::string>& labels)
{
  label_keys_ = labels;
  if (state_ != Hidden)
  {
    pull();
    push();
  }
}

void AnnotationMarker::setTags(const std::vector<std::string>& tags)
{
  tag_keys_ = tags;
  if (state_ != Hidden)
  {
    pull();
    push();
  }
}

void AnnotationMarker::updateMenu(const PointContext& context)
{
  menu_handler_ = MenuHandler();

  MenuHandler::EntryHandle mode_menu = menu_handler_.insert("Box Mode");
  auto handle = menu_handler_.insert(mode_menu, "Locked", boost::bind(&AnnotationMarker::lock, this, _1));
  menu_handler_.setCheckState(handle, mode_ == Locked ? MenuHandler::CHECKED : MenuHandler::NO_CHECKBOX);
  handle = menu_handler_.insert(mode_menu, "Move", boost::bind(&AnnotationMarker::enableMoveControl, this, _1));
  menu_handler_.setCheckState(handle, mode_ == Move ? MenuHandler::CHECKED : MenuHandler::NO_CHECKBOX);
  handle = menu_handler_.insert(mode_menu, "Rotate", boost::bind(&AnnotationMarker::enableRotationControl, this, _1));
  menu_handler_.setCheckState(handle, mode_ == Rotate ? MenuHandler::CHECKED : MenuHandler::NO_CHECKBOX);
  handle = menu_handler_.insert(mode_menu, "Resize", boost::bind(&AnnotationMarker::enableResizeControl, this, _1));
  menu_handler_.setCheckState(handle, mode_ == Resize ? MenuHandler::CHECKED : MenuHandler::NO_CHECKBOX);

  labels_.clear();
  if (!label_keys_.empty())
  {
    MenuHandler::EntryHandle label_menu = menu_handler_.insert("Label");
    for (auto const& label : label_keys_)
    {
      auto const handle = menu_handler_.insert(label_menu, label, boost::bind(&AnnotationMarker::setLabel, this, _1));
      labels_[handle] = label;
      menu_handler_.setCheckState(handle, label_ == label ? MenuHandler::CHECKED : MenuHandler::NO_CHECKBOX);
    }
  }

  tags_menu_.clear();
  if (!tag_keys_.empty())
  {
    MenuHandler::EntryHandle tags_menu = menu_handler_.insert("Tags");
    for (auto const& tag : tag_keys_)
    {
      auto const handle = menu_handler_.insert(tags_menu, tag, boost::bind(&AnnotationMarker::setTag, this, _1));
      tags_menu_[handle] = tag;
      auto const has_tag = find(tags_.cbegin(), tags_.cend(), tag) != tags_.cend();
      menu_handler_.setCheckState(handle, has_tag ? MenuHandler::CHECKED : MenuHandler::NO_CHECKBOX);
    }
  }

  MenuHandler::EntryHandle edit_menu = menu_handler_.insert("Actions");
  if (!undo_stack_.empty())
  {
    menu_handler_.insert(edit_menu, "Undo " + undo_stack_.top().undo_description,
                         boost::bind(&AnnotationMarker::undo, this, _1));
  }
  menu_handler_.insert(edit_menu, "Shrink to Points", boost::bind(&AnnotationMarker::shrink, this, _1));
  menu_handler_.insert(edit_menu, "Auto-fit Box", boost::bind(&AnnotationMarker::autoFit, this, _1));

  string commit_title = "Commit";
  if (context.points_nearby)
  {
    commit_title += " (despite " + to_string(context.points_nearby) + " nearby points)";
  }
  menu_handler_.insert(commit_title, boost::bind(&AnnotationMarker::commit, this, _1));

  menu_handler_.apply(*server_, marker_.name);
}

void AnnotationMarker::processFeedback(const InteractiveMarkerFeedbackConstPtr& feedback)
{
  switch (feedback->event_type)
  {
    case InteractiveMarkerFeedback::POSE_UPDATE:
      if (!button_click_active_ && mode_ == Move)
      {
        saveMove();
      }
      break;
    case InteractiveMarkerFeedback::BUTTON_CLICK:
      button_click_active_ = true;
      annotate_display_->setCurrentMarker(this);
      return;
      break;
    case InteractiveMarkerFeedback::MOUSE_DOWN:
      if (mode_ == Resize && feedback->mouse_point_valid)
      {
        poseMsgToTF(feedback->pose, last_pose_);
        pointMsgToTF(feedback->mouse_point, last_mouse_point_);
        can_change_size_ = true;
      }
      break;
    case InteractiveMarkerFeedback::MOUSE_UP:
      if (button_click_active_)
      {
        nextMode();
        button_click_active_ = false;
        can_change_size_ = false;
      }
      else if (mode_ == Resize && can_change_size_)
      {
        Pose pose;
        poseMsgToTF(feedback->pose, pose);
        UndoState state;
        state.time.start();
        state.undo_description = "change size";
        poseTFToMsg(last_pose_, state.pose);
        state.box_size = boxSize();
        state.state = state_;
        state.label = label_;
        state.tags = tags_;
        undo_stack_.push(state);
        changeSize(pose);
        can_change_size_ = false;
        return;
      }
      break;
  }
}

void AnnotationMarker::changeSize(const Pose& new_pose)
{
  Pose last_mouse_pose = new_pose;
  last_mouse_pose.setOrigin(last_mouse_point_);
  auto const mouse_diff = last_pose_.inverseTimes(last_mouse_pose).getOrigin();
  auto const diff = last_pose_.inverseTimes(new_pose).getOrigin();
  array<double, 3> const components({ fabs(diff.x()), fabs(diff.y()), fabs(diff.z()) });
  size_t const max_component = distance(components.cbegin(), max_element(components.cbegin(), components.cend()));
  auto const change = internal::sgn(mouse_diff.m_floats[max_component]) * diff.m_floats[max_component];

  pull();
  if (!marker_.controls.empty() && !marker_.controls.front().markers.empty())
  {
    auto& box = marker_.controls.front().markers.front();
    Vector3 scale;
    vector3MsgToTF(box.scale, scale);
    scale.m_floats[max_component] = max(0.05, scale[max_component] + change);
    setBoxSize(scale);
    Pose new_center = new_pose;
    new_center.setOrigin(last_pose_ * (0.5 * diff));
    poseTFToMsg(new_center, marker_.pose);

    if (annotate_display_->shrinkAfterResize())
    {
      auto const context = analyzePoints();
      if (context.points_inside)
      {
        saveForUndo("shrink to points");
        shrinkTo(context);
      }
    }

    updateState(Modified);
    push();
  }
}

void AnnotationMarker::pull()
{
  if (!server_->get(marker_.name, marker_))
  {
    push();
  }
}

void AnnotationMarker::push()
{
  auto const context = analyzePoints();
  updateDescription(context);
  server_->insert(marker_, boost::bind(&AnnotationMarker::processFeedback, this, _1));
  updateMenu(context);
  server_->applyChanges();
}

void AnnotationMarker::nextMode()
{
  if (mode_ == Move)
  {
    can_change_size_ = false;
    enableRotationControl();
  }
  else if (mode_ == Rotate)
  {
    enableResizeControl();
  }
  else if (mode_ == Resize)
  {
    enableMoveControl();
  }
}

void AnnotationMarker::createMarker(const TrackInstance& instance)
{
  marker_.header.frame_id = instance.center.frame_id_;
  marker_.header.stamp = time_;
  auto const center = instance.center.getOrigin();
  pointTFToMsg(center, marker_.pose.position);
  Quaternion rotation;
  rotation.setRPY(0.0, 0.0, 0.0);
  quaternionTFToMsg(rotation, marker_.pose.orientation);
  marker_.scale = 1;
  marker_.name = string("annotation_") + to_string(id_);
  createCubeControl();
  createMoveControl();
}

void AnnotationMarker::updateDescription(const PointContext& context)
{
  stringstream stream;
  stream << label_ << " #" << id_;
  if (!tags_.empty())
  {
    stream << " (";
    auto iter = tags_.begin();
    stream << *iter;
    ++iter;
    for (; iter != tags_.cend(); ++iter)
    {
      stream << ", ";
      stream << *iter;
    }
    stream << ")";
  }
  if (!marker_.controls.empty() && !marker_.controls.front().markers.empty())
  {
    auto const& box = marker_.controls.front().markers.front();
    Vector3 diff;
    if (context.points_inside)
    {
      Vector3 box_min;
      vector3MsgToTF(box.scale, box_min);
      box_min = -0.5 * box_min;
      Vector3 const box_max = -box_min;
      diff = (box_min - context.minimum).absolute() + (box_max - context.maximum).absolute();
    }

    stream << setiosflags(ios::fixed) << setprecision(2);
    stream << "\n" << box.scale.x << " (+" << diff.x() << ")";
    stream << " x " << box.scale.y << " (+" << diff.y() << ")";
    stream << " x " << box.scale.z << " (+" << diff.z() << ")";
  }

  stream << "\n" << context.points_inside << " points inside, ";
  stream << context.points_nearby << " nearby";

  marker_.description = stream.str();
}

void AnnotationMarker::enableMoveControl()
{
  mode_ = Move;
  pull();
  createMoveControl();
  push();
}

void AnnotationMarker::enableMoveControl(const InteractiveMarkerFeedbackConstPtr& feedback)
{
  enableMoveControl();
}

void AnnotationMarker::enableResizeControl()
{
  mode_ = Resize;
  pull();
  createResizeControl();
  push();
}

void AnnotationMarker::enableResizeControl(const InteractiveMarkerFeedbackConstPtr& feedback)
{
  enableResizeControl();
}

void AnnotationMarker::enableRotationControl()
{
  mode_ = Rotate;
  pull();
  createRotationControl();
  push();
}

void AnnotationMarker::enableRotationControl(const InteractiveMarkerFeedbackConstPtr& feedback)
{
  enableRotationControl();
}

void AnnotationMarker::lock()
{
  mode_ = Locked;
  pull();
  removeControls();
  push();
}

void AnnotationMarker::lock(const InteractiveMarkerFeedbackConstPtr& feedback)
{
  lock();
}

void AnnotationMarker::setLabel(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  if (feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT)
  {
    pull();
    if (label_ != labels_[feedback->menu_entry_id])
    {
      saveForUndo("label change");
      label_ = labels_[feedback->menu_entry_id];
      updateState(Modified);
      push();
    }
  }
}

void AnnotationMarker::setTag(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  if (feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT)
  {
    pull();
    MenuHandler::CheckState check_state;
    menu_handler_.getCheckState(feedback->menu_entry_id, check_state);
    auto const tag = tags_menu_[feedback->menu_entry_id];
    if (check_state == MenuHandler::CHECKED)
    {
      saveForUndo("remove tag " + tag);
      tags_.erase(remove(tags_.begin(), tags_.end(), tag), tags_.end());
    }
    else
    {
      saveForUndo("add tag " + tag);
      tags_.push_back(tag);
    }
    updateState(Modified);
    push();
  }
}

void AnnotationMarker::shrinkTo(const PointContext& context)
{
  if (!context.points_inside)
  {
    return;
  }

  Stamped<Point> input(0.5 * (context.maximum + context.minimum), context.time, "current_annotation");
  Stamped<Point> output;
  annotate_display_->transformListener().transformPoint(marker_.header.frame_id, input, output);
  pointTFToMsg(output, marker_.pose.position);
  double const offset = 0.05;
  Vector3 const margin(offset, offset, offset);
  setBoxSize(margin + context.maximum - context.minimum);
}

void AnnotationMarker::shrink(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  pull();
  auto const context = analyzePoints();
  if (context.points_inside)
  {
    saveForUndo("shrink to points");
    shrinkTo(context);
    updateState(Modified);
    push();
  }
}

bool AnnotationMarker::fitNearbyPoints()
{
  {
    auto const context = analyzePoints();
    if (context.points_nearby == 0)
    {
      shrinkTo(context);
      return true;
    }
  }

  for (int i = 0; i < 4; ++i)
  {
    resize(0.25);
    auto const context = analyzePoints();
    if (context.points_nearby == 0)
    {
      shrinkTo(context);
      return true;
    }
  }

  return false;
}

void AnnotationMarker::autoFit()
{
  pull();
  saveForUndo("auto-fit box");
  if (fitNearbyPoints())
  {
    updateState(Modified);
    push();
  }
  else if (!undo_stack_.empty() && undo_stack_.top().undo_description == "auto-fit box")
  {
    undo();
  }
}

void AnnotationMarker::autoFit(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  autoFit();
}

void AnnotationMarker::saveMove()
{
  bool const merge_with_previous =
      !undo_stack_.empty() && undo_stack_.top().undo_description == "move" && undo_stack_.top().time.elapsed() < 800;
  if (merge_with_previous)
  {
    undo_stack_.top().time.restart();
  }
  else
  {
    auto const pose = marker_.pose;
    pull();
    if (hasMoved(pose, marker_.pose))
    {
      saveForUndo("move");
    }
    push();
  }
}

bool AnnotationMarker::hasMoved(geometry_msgs::Pose const& one, geometry_msgs::Pose const& two) const
{
  Transform a;
  poseMsgToTF(one, a);
  Transform b;
  poseMsgToTF(two, b);
  return !(a.getOrigin() - b.getOrigin()).fuzzyZero();
}

void AnnotationMarker::saveForUndo(const string& description)
{
  if (!undo_stack_.empty())
  {
    auto const& last_state = undo_stack_.top();
    if (last_state.state == state_ && last_state.label == label_ && last_state.tags == tags_ && last_state.box_size == boxSize() &&
        !hasMoved(last_state.pose, marker_.pose))
    {
      return;
    }
  }

  UndoState state;
  state.time.start();
  state.undo_description = description;
  state.pose = marker_.pose;
  state.box_size = boxSize();
  state.state = state_;
  state.label = label_;
  state.tags = tags_;
  undo_stack_.push(state);
}

void AnnotationMarker::undo()
{
  if (undo_stack_.empty())
  {
    return;
  }

  auto const state = undo_stack_.top();
  marker_.pose = state.pose;
  label_ = state.label;
  tags_ = state.tags;
  undo_stack_.pop();
  setBoxSize(state.box_size);
  updateState(state.state);
  push();
}

void AnnotationMarker::undo(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  undo();
}

void AnnotationMarker::resize(double offset)
{
  if (!marker_.controls.empty() && !marker_.controls.front().markers.empty())
  {
    auto& box = marker_.controls.front().markers.front();
    if (ignore_ground_)
    {
      marker_.pose.position.z += offset / 4.0;
      setBoxSize({ box.scale.x + offset, box.scale.y + offset, box.scale.z + offset / 2.0 });
    }
    else
    {
      setBoxSize({ box.scale.x + offset, box.scale.y + offset, box.scale.z + offset });
    }
  }
}

AnnotationMarker::PointContext AnnotationMarker::analyzePoints() const
{
  Transform transform;
  poseMsgToTF(marker_.pose, transform);
  auto const cloud = annotate_display_->cloud();
  PointContext context;
  if (!cloud)
  {
    return context;
  }
  context.time = min(time_, cloud->header.stamp);
  auto const stamped_transform =
      StampedTransform(transform, context.time, marker_.header.frame_id, "current_annotation");

  auto& transform_listener = annotate_display_->transformListener();
  transform_listener.setTransform(stamped_transform);
  string error;
  bool const can_transform = transform_listener.waitForTransform("current_annotation", cloud->header.frame_id,
                                                                 context.time, ros::Duration(0.25));
  auto const time = can_transform ? context.time : ros::Time();
  if (transform_listener.canTransform("current_annotation", cloud->header.frame_id, time, &error))
  {
    StampedTransform trafo;
    transform_listener.lookupTransform("current_annotation", cloud->header.frame_id, time, trafo);
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    pcl::fromROSMsg(*cloud, pointcloud);
    if (pointcloud.points.empty())
    {
      return context;
    }
    pcl::PointCloud<pcl::PointXYZ> annotation_cloud;
    pcl_ros::transformPointCloud(pointcloud, annotation_cloud, trafo);
    Vector3 points_min(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
    Vector3 points_max(numeric_limits<float>::min(), numeric_limits<float>::min(), numeric_limits<float>::min());
    if (!marker_.controls.empty() && !marker_.controls.front().markers.empty())
    {
      auto& box = marker_.controls.front().markers.front();
      Vector3 box_min;
      vector3MsgToTF(box.scale, box_min);
      box_min = -0.5 * box_min;
      Vector3 const box_max = -box_min;
      Vector3 offset(0.25, 0.25, 0.25);
      Vector3 const nearby_max = box_max + offset;
      Vector3 nearby_min = box_min - offset;
      if (ignore_ground_)
      {
        nearby_min.setZ(box_min.z());
      }
      for (auto const& p : annotation_cloud.points)
      {
        Vector3 const point(p.x, p.y, p.z);
        Vector3 alien = point;
        alien.setMax(nearby_min);
        alien.setMin(nearby_max);
        if (alien == point)
        {
          Vector3 canary = point;
          canary.setMax(box_min);
          canary.setMin(box_max);
          if (canary == point)
          {
            ++context.points_inside;
            context.minimum.setMin(point);
            context.maximum.setMax(point);
          }
          else
          {
            ++context.points_nearby;
          }
        }
      }
    }
  }
  else
  {
    ROS_WARN_STREAM("Transformation failed: " << error);
  }
  return context;
}

void AnnotationMarker::rotateYaw(double delta_rad)
{
  pull();
  Quaternion rotation;
  quaternionMsgToTF(marker_.pose.orientation, rotation);
  Quaternion delta;
  delta.setRPY(0.0, 0.0, delta_rad);
  auto rotated = delta * rotation;
  rotated.normalize();
  quaternionTFToMsg(rotated, marker_.pose.orientation);
  push();
}

void AnnotationMarker::commit()
{
  pull();
  if (annotate_display_->shrinkBeforeCommit())
  {
    auto const context = analyzePoints();
    if (context.points_inside)
    {
      saveForUndo("shrink to points");
      shrinkTo(context);
      updateState(Modified);
    }
  }

  TrackInstance instance;
  instance.label = label_;
  instance.tags = tags_;

  Transform transform;
  poseMsgToTF(marker_.pose, transform);
  instance.center = StampedTransform(transform, time_, marker_.header.frame_id, "current_annotation");

  if (!marker_.controls.empty() && !marker_.controls.front().markers.empty())
  {
    auto& box = marker_.controls.front().markers.front();
    vector3MsgToTF(box.scale, instance.box_size);
  }

  tf_broadcaster_.sendTransform(instance.center);

  track_.erase(
      remove_if(track_.begin(), track_.end(), [this](const TrackInstance& t) { return t.center.stamp_ == time_; }),
      track_.end());
  track_.push_back(instance);
  sort(track_.begin(), track_.end(),
       [](TrackInstance const& a, TrackInstance const& b) -> bool { return a.center.stamp_ < b.center.stamp_; });
  if (annotate_display_->save())
  {
    updateState(Committed);
  }
  annotate_display_->publishTrackMarkers();
  push();
}

void AnnotationMarker::commit(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  if (feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT)
  {
    commit();
  }
}

void AnnotationMarker::updateState(State state)
{
  if (state_ == state)
  {
    return;
  }

  state_ = state;
  if (!marker_.controls.empty() && !marker_.controls.front().markers.empty())
  {
    auto& box = marker_.controls.front().markers.front();

    switch (state_)
    {
      case Hidden:
      case New:
        box.color.r = 0.5;
        box.color.g = 0.5;
        box.color.b = 0.5;
        break;
      case Committed:
        box.color.r = 60 / 255.0;
        box.color.g = 180 / 255.0;
        box.color.b = 75 / 255.0;
        break;
      case Modified:
        box.color.r = 245 / 255.0;
        box.color.g = 130 / 255.0;
        box.color.b = 48 / 255.0;
        break;
    }
  }
}

int AnnotationMarker::id() const
{
  return id_;
}

Track const& AnnotationMarker::track() const
{
  return track_;
}

StampedTransform estimatePose(StampedTransform const& a, StampedTransform const& b, ros::Time const& time)
{
  auto const time_diff = (b.stamp_ - a.stamp_).toSec();
  if (fabs(time_diff) < 0.001)
  {
    // Avoid division by zero and measurement noise affecting interpolation results
    return a;
  }
  auto const ratio = (time - a.stamp_).toSec() / time_diff;
  Transform transform;
  transform.setOrigin(a.getOrigin().lerp(b.getOrigin(), ratio));
  transform.setRotation(a.getRotation().slerp(b.getRotation(), ratio));
  return StampedTransform(transform, time, a.frame_id_, a.child_frame_id_);
}

void AnnotationMarker::setTime(const ros::Time& time)
{
  time_ = time;
  if (!track_.empty())
  {
    auto const prune_before_track_start = time < track_.front().center.stamp_ && track_.front().timeTo(time) > 1.0;
    auto const prune_after_track_end = time > track_.back().center.stamp_ && track_.back().timeTo(time) > 1.0;
    if (prune_before_track_start || prune_after_track_end)
    {
      server_->erase(marker_.name);
      server_->applyChanges();
      updateState(Hidden);
      return;
    }
  }

  pull();
  while (!undo_stack_.empty())
  {
    undo_stack_.pop();
  }
  marker_.header.stamp = time;

  // Find an existing annotation for this point in time, if any
  for (auto const& instance : track_)
  {
    if (instance.timeTo(time) < 0.01)
    {
      label_ = instance.label;
      tags_ = instance.tags;
      updateState(Committed);
      poseTFToMsg(instance.center, marker_.pose);
      setBoxSize(instance.box_size);
      push();
      return;
    }
  }

  // Estimate a suitable pose from nearby annotations
  Track track = track_;
  sort(track.begin(), track.end(),
       [time](TrackInstance const& a, TrackInstance const& b) -> bool { return a.timeTo(time) < b.timeTo(time); });
  double const extrapolation_limit = 2.0;
  if (track.size() > 1 && track[1].timeTo(time) < extrapolation_limit)
  {
    auto const transform = estimatePose(track[0].center, track[1].center, time);
    poseTFToMsg(transform, marker_.pose);
    if (annotate_display_->autoFitAfterPredict())
    {
      fitNearbyPoints();
    }
  }

  updateState(New);
  push();
}

void AnnotationMarker::setTrack(const Track& track)
{
  track_ = track;
}

void AnnotationMarker::setIgnoreGround(bool enabled)
{
  ignore_ground_ = enabled;
}

}  // namespace annotate

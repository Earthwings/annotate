#include <annotate/annotate_node.h>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <fstream>
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

void setRotation(geometry_msgs::Quaternion& quaternion, double x, double y, double z)
{
  Quaternion orientation(x, y, z, 1.0);
  orientation.normalize();
  quaternionTFToMsg(orientation, quaternion);
}

std_msgs::ColorRGBA createColor(int id)
{
  // Cache color such that subsequent calls return the same one
  static map<int, std_msgs::ColorRGBA> colors;
  auto const iter = colors.find(id);
  if (iter != colors.end())
  {
    return iter->second;
  }

  // Reverse the ID bits to spread them far over the hue range of 0..360
  // 0 => 0, 1 => 180, 2 => 90, 3 => 270, ...
  uchar h = uchar(id);
  h = (h & 0xF0) >> 4 | (h & 0x0F) << 4;
  h = (h & 0xCC) >> 2 | (h & 0x33) << 2;
  h = (h & 0xAA) >> 1 | (h & 0x55) << 1;
  int const hue = int(h * 360.0 / 256);

  // Vary saturation and value slightly
  random_device rd;
  mt19937 mt(rd());
  uniform_int_distribution<int> dist(210, 240);
  QColor color;
  color.setHsv(hue, dist(mt), dist(mt));

  std_msgs::ColorRGBA result;
  qreal r, g, b;
  color.getRgbF(&r, &g, &b);
  result.r = r;
  result.g = g;
  result.b = b;
  colors[id] = result;
  return result;
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

Marker createTrackLine(float scale, const std_msgs::ColorRGBA& color)
{
  Marker marker;
  marker.type = Marker::LINE_STRIP;
  setRotation(marker.pose.orientation, 0.0, 0.0, 0.0);
  marker.color = color;
  marker.color.a = 0.7;
  marker.scale.x = scale;
  return marker;
}

Marker createTrackSpheres(float scale, const std_msgs::ColorRGBA& color)
{
  Marker marker;
  marker.type = Marker::SPHERE_LIST;
  setRotation(marker.pose.orientation, 0.0, 0.0, 0.0);
  marker.color = color;
  marker.color.a = 0.7;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  return marker;
}

}  // namespace internal

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

void AnnotationMarker::createPositionControl()
{
  removeControls();
  InteractiveMarkerControl control;
  internal::setRotation(control.orientation, 0.0, 1.0, 0.0);
  control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
  marker_.controls.push_back(control);
}

void AnnotationMarker::createScaleControl()
{
  removeControls();
  InteractiveMarkerControl control;

  internal::setRotation(control.orientation, 1.0, 0.0, 0.0);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker_.controls.push_back(control);

  internal::setRotation(control.orientation, 0.0, 1.0, 0.0);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker_.controls.push_back(control);

  internal::setRotation(control.orientation, 0.0, 0.0, 1.0);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker_.controls.push_back(control);
}

tf::Vector3 AnnotationMarker::boxSize() const
{
  tf::Vector3 box_size;
  if (!marker_.controls.empty() && !marker_.controls.front().markers.empty())
  {
    auto& box = marker_.controls.front().markers.front();
    vector3MsgToTF(box.scale, box_size);
  }
  return box_size;
}

void AnnotationMarker::setBoxSize(const tf::Vector3& box_size)
{
  if (!marker_.controls.empty() && !marker_.controls.front().markers.empty())
  {
    auto& box = marker_.controls.front().markers.front();
    vector3TFToMsg(box_size, box.scale);
  }
  marker_.scale = 0.2 + box_size[box_size.maxAxis()];
}

AnnotationMarker::AnnotationMarker(Markers* markers, const shared_ptr<InteractiveMarkerServer>& server,
                                   const TrackInstance& trackInstance, int marker_id,
                                   const std::vector<std::string>& label_keys)
  : server_(server), id_(marker_id), label_keys_(label_keys), markers_(markers)
{
  time_ = trackInstance.center.stamp_;
  createMarker(trackInstance);
}

void AnnotationMarker::updateMenu()
{
  menu_handler_ = MenuHandler();
  MenuHandler::EntryHandle mode_menu = menu_handler_.insert("Mode");
  menu_handler_.insert(mode_menu, "Lock", boost::bind(&AnnotationMarker::lock, this, _1));
  menu_handler_.insert(mode_menu, "Move", boost::bind(&AnnotationMarker::changePosition, this, _1));
  menu_handler_.insert(mode_menu, "Resize", boost::bind(&AnnotationMarker::changeScale, this, _1));

  labels_.clear();
  if (!label_keys_.empty())
  {
    MenuHandler::EntryHandle label_menu = menu_handler_.insert("Label");
    for (auto const& label : label_keys_)
    {
      auto const handle = menu_handler_.insert(label_menu, label, boost::bind(&AnnotationMarker::setLabel, this, _1));
      labels_[handle] = label;
    }
  }

  string commit_title = "Commit";
  auto const context = analyzePoints();
  if (context.points_nearby)
  {
    commit_title += " (despite " + to_string(context.points_nearby) + " nearby points)";
  }

  MenuHandler::EntryHandle edit_menu = menu_handler_.insert("Edit");
  if (!undo_stack_.empty())
  {
    menu_handler_.insert(edit_menu, "Undo " + undo_stack_.top().undo_description,
                         boost::bind(&AnnotationMarker::undo, this, _1));
  }
  menu_handler_.insert(edit_menu, "Expand Box", boost::bind(&AnnotationMarker::expand, this, _1));
  menu_handler_.insert(edit_menu, "Shrink to Points", boost::bind(&AnnotationMarker::shrink, this, _1));
  menu_handler_.insert(edit_menu, "Auto-fit Box", boost::bind(&AnnotationMarker::autoFit, this, _1));
  menu_handler_.insert(edit_menu, commit_title, boost::bind(&AnnotationMarker::commit, this, _1));
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
      return;
      break;
    case InteractiveMarkerFeedback::MOUSE_DOWN:
      if (mode_ == Scale && feedback->mouse_point_valid)
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
      else if (mode_ == Scale && can_change_size_)
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
  updateDescription();
  server_->insert(marker_, boost::bind(&AnnotationMarker::processFeedback, this, _1));
  updateMenu();
  server_->applyChanges();
}

void AnnotationMarker::nextMode()
{
  if (mode_ == Move)
  {
    can_change_size_ = false;
    changeScale();
  }
  else if (mode_ == Scale)
  {
    changePosition();
  }
}

void AnnotationMarker::createMarker(const TrackInstance& instance)
{
  marker_.header.frame_id = instance.center.frame_id_;
  marker_.header.stamp = time_;
  auto const center = instance.center.getOrigin();
  pointTFToMsg(center, marker_.pose.position);
  marker_.scale = 1;
  marker_.name = string("annotation_") + to_string(id_);
  createCubeControl();
  createPositionControl();
}

void AnnotationMarker::updateDescription()
{
  stringstream stream;
  stream << label_ << " #" << id_;
  if (!marker_.controls.empty() && !marker_.controls.front().markers.empty())
  {
    auto const& box = marker_.controls.front().markers.front();
    stream << "\n" << setiosflags(ios::fixed) << setprecision(2) << box.scale.x;
    stream << " x " << setiosflags(ios::fixed) << setprecision(2) << box.scale.y;
    stream << " x " << setiosflags(ios::fixed) << setprecision(2) << box.scale.z;
  }
  marker_.description = stream.str();
}

void AnnotationMarker::changePosition()
{
  mode_ = Move;
  pull();
  createPositionControl();
  push();
}

void AnnotationMarker::changePosition(const InteractiveMarkerFeedbackConstPtr& feedback)
{
  changePosition();
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

void AnnotationMarker::changeScale()
{
  mode_ = Scale;
  pull();
  createScaleControl();
  push();
}

void AnnotationMarker::changeScale(const InteractiveMarkerFeedbackConstPtr& feedback)
{
  changeScale();
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

void AnnotationMarker::shrinkTo(const PointContext& context)
{
  Stamped<tf::Point> input(0.5 * (context.maximum + context.minimum), context.time, "current_annotation");
  Stamped<tf::Point> output;
  markers_->transformListener().transformPoint(marker_.header.frame_id, input, output);
  pointTFToMsg(output, marker_.pose.position);
  double const offset = 0.05;
  if (ignore_ground_)
  {
    marker_.pose.position.z += offset / 4.0;
    tf::Vector3 const margin(offset, offset, offset / 2.0);
    setBoxSize(margin + context.maximum - context.minimum);
  }
  else
  {
    tf::Vector3 const margin(offset, offset, offset);
    setBoxSize(margin + context.maximum - context.minimum);
  }
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

void AnnotationMarker::autoFit(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  pull();
  saveForUndo("auto-fit box");
  if (analyzePoints().points_nearby == 0)
  {
    return;
  }
  for (int i = 0; i < 4; ++i)
  {
    resize(0.25);
    auto const context = analyzePoints();
    if (context.points_nearby == 0)
    {
      shrinkTo(context);
      updateState(Modified);
      push();
      return;
    }
  }
  if (!undo_stack_.empty() && undo_stack_.top().undo_description == "auto-fit box")
  {
    undo();
  }
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
  tf::Transform a;
  tf::poseMsgToTF(one, a);
  tf::Transform b;
  tf::poseMsgToTF(two, b);
  return !(a.getOrigin() - b.getOrigin()).fuzzyZero();
}

void AnnotationMarker::saveForUndo(const string& description)
{
  if (!undo_stack_.empty())
  {
    auto const& last_state = undo_stack_.top();
    if (last_state.state == state_ && last_state.label == label_ && last_state.box_size == boxSize() &&
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
  undo_stack_.pop();
  setBoxSize(state.box_size);
  updateState(state.state);
  push();
}

void AnnotationMarker::undo(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  undo();
}

void AnnotationMarker::expand(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  if (feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT)
  {
    pull();
    saveForUndo("expand box");
    resize(0.25);
    updateState(Modified);
    push();
  }
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
  tf::Transform transform;
  tf::poseMsgToTF(marker_.pose, transform);
  auto const stamped_transform = StampedTransform(transform, time_, marker_.header.frame_id, "current_annotation");

  auto const cloud = markers_->cloud();
  auto& transform_listener = markers_->transformListener();
  transform_listener.setTransform(stamped_transform);
  string error;
  PointContext context;
  context.time = cloud->header.stamp;
  if (transform_listener.canTransform("current_annotation", cloud->header.frame_id, cloud->header.stamp, &error))
  {
    tf::StampedTransform trafo;
    transform_listener.lookupTransform("current_annotation", cloud->header.frame_id, cloud->header.stamp, trafo);
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    pcl::fromROSMsg(*cloud, pointcloud);
    if (pointcloud.points.empty())
    {
      return context;
    }
    pcl::PointCloud<pcl::PointXYZ> annotation_cloud;
    pcl_ros::transformPointCloud(pointcloud, annotation_cloud, trafo);
    tf::Vector3 points_min(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
    tf::Vector3 points_max(numeric_limits<float>::min(), numeric_limits<float>::min(), numeric_limits<float>::min());
    if (!marker_.controls.empty() && !marker_.controls.front().markers.empty())
    {
      auto& box = marker_.controls.front().markers.front();
      tf::Vector3 box_min;
      vector3MsgToTF(box.scale, box_min);
      box_min = -0.5 * box_min;
      tf::Vector3 const box_max = -box_min;
      tf::Vector3 offset(0.25, 0.25, 0.25);
      tf::Vector3 const nearby_max = box_max + offset;
      tf::Vector3 nearby_min = box_min - offset;
      if (ignore_ground_)
      {
        nearby_min.setZ(box_min.z());
      }
      for (auto const& p : annotation_cloud.points)
      {
        tf::Vector3 const point(p.x, p.y, p.z);
        tf::Vector3 alien = point;
        alien.setMax(nearby_min);
        alien.setMin(nearby_max);
        if (alien == point)
        {
          tf::Vector3 canary = point;
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

void AnnotationMarker::commit(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  if (feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT)
  {
    pull();
    TrackInstance instance;
    instance.label = label_;

    tf::Transform transform;
    tf::poseMsgToTF(marker_.pose, transform);
    instance.center = StampedTransform(transform, time_, marker_.header.frame_id, "current_annotation");

    if (!marker_.controls.empty() && !marker_.controls.front().markers.empty())
    {
      auto& box = marker_.controls.front().markers.front();
      vector3MsgToTF(box.scale, instance.box_size);
    }

    tf_broadcaster_.sendTransform(instance.center);

    track_.erase(std::remove_if(track_.begin(), track_.end(),
                                [this](const TrackInstance& t) { return t.center.stamp_ == time_; }),
                 track_.end());
    track_.push_back(instance);
    sort(track_.begin(), track_.end(),
         [](TrackInstance const& a, TrackInstance const& b) -> bool { return a.center.stamp_ < b.center.stamp_; });
    if (markers_->save())
    {
      updateState(Committed);
    }
    markers_->publishTrackMarkers();
    push();
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

tf::StampedTransform estimatePose(tf::StampedTransform const& a, tf::StampedTransform const& b, ros::Time const& time)
{
  auto const time_diff = (b.stamp_ - a.stamp_).toSec();
  if (fabs(time_diff) < 0.001)
  {
    // Avoid division by zero and measurement noise affecting interpolation results
    return a;
  }
  auto const ratio = (time - a.stamp_).toSec() / time_diff;
  tf::Transform transform;
  transform.setOrigin(a.getOrigin().lerp(b.getOrigin(), ratio));
  transform.setRotation(a.getRotation().slerp(b.getRotation(), ratio));
  return tf::StampedTransform(transform, time, a.frame_id_, a.child_frame_id_);
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

void Markers::createNewAnnotation(const geometry_msgs::PointStamped::ConstPtr& message)
{
  tf::Transform transform;
  transform.setOrigin({ message->point.x, message->point.y, message->point.z });
  TrackInstance instance;
  instance.center = StampedTransform(transform, time_, message->header.frame_id, "current_annotation");
  instance.label = "unknown";
  instance.box_size.setValue(1.0, 1.0, 1.0);
  ++current_marker_id_;
  auto marker = make_shared<AnnotationMarker>(this, server_, instance, current_marker_id_, labels_);
  markers_.push_back(marker);
  marker->setIgnoreGround(ignore_ground_);
  marker->setTime(time_);
}

void Markers::handlePointcloud(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  cloud_ = cloud;
  time_ = cloud->header.stamp;
  for (auto& marker : markers_)
  {
    marker->setTime(time_);
  }
  publishTrackMarkers();
}

Markers::Markers()
{
  ros::NodeHandle private_node_handle("~");
  auto const labels = private_node_handle.param<string>("labels", "object");
  filename_ = private_node_handle.param<string>("annotations", "annotate.yaml");
  ignore_ground_ = private_node_handle.param<bool>("ignore_ground", ignore_ground_);

  istringstream stream(labels);
  string value;
  while (getline(stream, value, ','))
  {
    labels_.push_back(value);
  }

  server_ = make_shared<InteractiveMarkerServer>("annotate_node", "", false);
  track_marker_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("tracks", 10, true);
  load();
  new_annotation_subscriber_ = node_handle_.subscribe("/new_annotation", 10, &Markers::createNewAnnotation, this);
  auto const pointcloud_topic = private_node_handle.param<string>("pointcloud_topic", "/velodyne_points");
  pointcloud_subscriber_ = node_handle_.subscribe(pointcloud_topic, 10, &Markers::handlePointcloud, this);
  publishTrackMarkers();
}

void Markers::load()
{
  using namespace YAML;
  Node node;
  try
  {
    node = LoadFile(filename_);
  }
  catch (Exception const& e)
  {
    ROS_DEBUG_STREAM("Failed to open " << filename_ << ": " << e.msg);
    return;
  }
  Node tracks = node["tracks"];
  for (size_t i = 0; i < tracks.size(); ++i)
  {
    Track track;
    Node annotation = tracks[i];
    auto const id = annotation["id"].as<size_t>();
    Node t = annotation["track"];
    for (size_t j = 0; j < t.size(); ++j)
    {
      Node inst = t[j];
      TrackInstance instance;
      instance.label = inst["label"].as<string>();

      Node header = inst["header"];
      instance.center.frame_id_ = header["frame_id"].as<string>();
      instance.center.stamp_.sec = header["stamp"]["secs"].as<uint32_t>();
      instance.center.stamp_.nsec = header["stamp"]["nsecs"].as<uint32_t>();

      Node origin = inst["translation"];
      instance.center.setOrigin({ origin["x"].as<double>(), origin["y"].as<double>(), origin["z"].as<double>() });
      Node rotation = inst["rotation"];
      instance.center.setRotation({ rotation["x"].as<double>(), rotation["y"].as<double>(), rotation["z"].as<double>(),
                                    rotation["w"].as<double>() });

      Node box = inst["box"];
      instance.box_size.setX(box["length"].as<double>());
      instance.box_size.setY(box["width"].as<double>());
      instance.box_size.setZ(box["height"].as<double>());

      track.push_back(instance);
    }

    if (!track.empty())
    {
      current_marker_id_ = max(current_marker_id_, id);
      auto marker = make_shared<AnnotationMarker>(this, server_, track.front(), id, labels_);
      marker->setTrack(track);
      markers_.push_back(marker);
    }
  }
}

bool Markers::save() const
{
  using namespace YAML;
  Node node;
  for (auto const& marker : markers_)
  {
    Node annotation;
    annotation["id"] = marker->id();
    for (auto const& instance : marker->track())
    {
      Node i;
      i["label"] = instance.label;

      Node header;
      header["frame_id"] = instance.center.frame_id_;
      Node stamp;
      stamp["secs"] = instance.center.stamp_.sec;
      stamp["nsecs"] = instance.center.stamp_.nsec;
      header["stamp"] = stamp;
      i["header"] = header;

      Node origin;
      auto const o = instance.center.getOrigin();
      origin["x"] = o.x();
      origin["y"] = o.y();
      origin["z"] = o.z();
      i["translation"] = origin;

      Node rotation;
      auto const q = instance.center.getRotation();
      rotation["x"] = q.x();
      rotation["y"] = q.y();
      rotation["z"] = q.z();
      rotation["w"] = q.w();
      i["rotation"] = rotation;

      Node box;
      box["length"] = instance.box_size.x();
      box["width"] = instance.box_size.y();
      box["height"] = instance.box_size.z();
      i["box"] = box;

      annotation["track"].push_back(i);
    }
    node["tracks"].push_back(annotation);
  }

  ofstream stream(filename_);
  if (stream.is_open())
  {
    stream << node;
    if (stream.bad())
    {
      ROS_WARN_STREAM("Failed to write annotations to " << filename_);
      return false;
    }
  }
  else
  {
    ROS_WARN_STREAM("Failed to open " << filename_ << " for writing. Annotations will not be saved.");
    return false;
  }
  stream.close();
  return true;
}

void Markers::publishTrackMarkers()
{
  visualization_msgs::MarkerArray message;

  Marker delete_all;
  delete_all.action = Marker::DELETEALL;
  message.markers.push_back(delete_all);

  for (auto const& marker : markers_)
  {
    auto const color = internal::createColor(marker->id());
    Marker line = internal::createTrackLine(0.02, color);
    line.id = marker->id();
    line.ns = "Path";
    Marker dots = internal::createTrackSpheres(0.1, color);
    dots.id = marker->id() << 16;
    dots.ns = "Positions";

    for (auto const& instance : marker->track())
    {
      if (instance.center.stamp_ <= time_ && instance.timeTo(time_) <= 5.0)
      {
        geometry_msgs::Point point;
        auto const p = instance.center.getOrigin();
        pointTFToMsg(p, point);
        line.points.push_back(point);
        line.header.frame_id = instance.center.frame_id_;
        dots.points.push_back(point);
        dots.header.frame_id = instance.center.frame_id_;
      }
    }

    line.action = line.points.size() < 2 ? Marker::DELETE : Marker::ADD;
    message.markers.push_back(line);
    dots.action = dots.points.empty() ? Marker::DELETE : Marker::ADD;
    message.markers.push_back(dots);
  }

  track_marker_publisher_.publish(message);
}

sensor_msgs::PointCloud2ConstPtr Markers::cloud() const
{
  return cloud_;
}

tf::TransformListener& Markers::transformListener()
{
  return transform_listener_;
}

}  // namespace annotate

int main(int argc, char** argv)
{
  ros::init(argc, argv, "annotate_node");
  annotate::Markers markers;
  ros::spin();
  return 0;
}
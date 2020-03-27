#include <annotate/annotate_node.h>
#include <sstream>

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
  marker.color.a = 0.8;
  return marker;
}

void removeControls(InteractiveMarker& int_marker)
{
  int_marker.controls.resize(1);
}

InteractiveMarkerControl& createCubeControl(InteractiveMarker& marker)
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(createCube(marker.scale - 0.2));
  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  marker.controls.push_back(control);
  return marker.controls.back();
}

void createPositionControl(InteractiveMarker& marker)
{
  removeControls(marker);
  InteractiveMarkerControl control;
  Quaternion orien(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  quaternionTFToMsg(orien, control.orientation);
  control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
  marker.controls.push_back(control);
}

void createScaleControl(InteractiveMarker& marker)
{
  removeControls(marker);
  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);
}

}  // namespace internal

AnnotationMarker::AnnotationMarker(const shared_ptr<InteractiveMarkerServer>& server, const Vector3& position,
                                   const string& frame_id, int marker_id, const std::vector<std::string>& labels)
  : server_(server), id_(marker_id)
{
  MenuHandler::EntryHandle mode_menu = menu_handler_.insert("Mode");
  menu_handler_.insert(mode_menu, "Locked", boost::bind(&AnnotationMarker::lock, this, _1));
  menu_handler_.insert(mode_menu, "Change Position", boost::bind(&AnnotationMarker::changePosition, this, _1));
  menu_handler_.insert(mode_menu, "Change Scale", boost::bind(&AnnotationMarker::changeScale, this, _1));

  if (!labels.empty())
  {
    MenuHandler::EntryHandle label_menu = menu_handler_.insert("Label");
    for (auto const& label : labels)
    {
      auto const handle = menu_handler_.insert(label_menu, label, boost::bind(&AnnotationMarker::setLabel, this, _1));
      labels_[handle] = label;
    }
  }

  createMarker(position, frame_id);
  server_->applyChanges();
}

void AnnotationMarker::processFeedback(const InteractiveMarkerFeedbackConstPtr& feedback)
{
  switch (feedback->event_type)
  {
    case InteractiveMarkerFeedback::BUTTON_CLICK:
      nextMode();
      return;
      break;
    case InteractiveMarkerFeedback::MENU_SELECT:
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
      if (mode_ == Scale && can_change_size_)
      {
        Pose pose;
        poseMsgToTF(feedback->pose, pose);
        changeSize(pose);
        can_change_size_ = false;
        return;
      }
      break;
  }
  server_->applyChanges();
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

  InteractiveMarker marker;
  if (server_->get(name_, marker))
  {
    if (!marker.controls.empty() && !marker.controls.front().markers.empty())
    {
      auto& box = marker.controls.front().markers.front();
      Vector3 scale;
      vector3MsgToTF(box.scale, scale);
      scale.m_floats[max_component] = max(0.05, scale[max_component] + change);
      vector3TFToMsg(scale, box.scale);
      marker.scale = 0.2 + max(box.scale.x, max(box.scale.y, box.scale.z));
      Pose new_center = new_pose;
      new_center.setOrigin(last_pose_ * (0.5 * diff));
      poseTFToMsg(new_center, marker.pose);
      addMarker(marker);
    }
  }
}

void AnnotationMarker::addMarker(InteractiveMarker& int_marker)
{
  server_->insert(int_marker);
  server_->setCallback(int_marker.name, boost::bind(&AnnotationMarker::processFeedback, this, _1));
  menu_handler_.apply(*server_, int_marker.name);
  server_->applyChanges();
}

void AnnotationMarker::nextMode()
{
  switch (mode_)
  {
    case Locked:
      changePosition();
      break;
    case Move:
      changeScale();
      break;
    case Scale:
      lock();
      break;
  }
}

void AnnotationMarker::createMarker(const Vector3& position, const string& frame_id)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  name_ = string("annotation_") + to_string(id_);
  int_marker.name = name_;
  int_marker.description = "Annotation #" + to_string(id_);

  internal::createCubeControl(int_marker);
  internal::createPositionControl(int_marker);
  addMarker(int_marker);
}

void AnnotationMarker::changePosition()
{
  mode_ = Move;
  InteractiveMarker marker;
  if (server_->get(name_, marker))
  {
    internal::createPositionControl(marker);
    addMarker(marker);
  }
}

void AnnotationMarker::changePosition(const InteractiveMarkerFeedbackConstPtr& feedback)
{
  changePosition();
}

void AnnotationMarker::lock()
{
  mode_ = Locked;
  InteractiveMarker marker;
  if (server_->get(name_, marker))
  {
    internal::removeControls(marker);
    addMarker(marker);
  }
}

void AnnotationMarker::lock(const InteractiveMarkerFeedbackConstPtr& feedback)
{
  lock();
}

void AnnotationMarker::changeScale()
{
  mode_ = Scale;
  InteractiveMarker marker;
  if (server_->get(name_, marker))
  {
    internal::createScaleControl(marker);
    addMarker(marker);
  }
}

void AnnotationMarker::changeScale(const InteractiveMarkerFeedbackConstPtr& feedback)
{
  changeScale();
}

void AnnotationMarker::setLabel(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  if (feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT)
  {
    InteractiveMarker marker;
    if (server_->get(name_, marker))
    {
      label_ = labels_[feedback->menu_entry_id];
      marker.description = label_ + string(" #") + to_string(id_);
      addMarker(marker);
    }
  }
}

void Markers::createNewAnnotation(const geometry_msgs::PointStamped::ConstPtr& message)
{
  Vector3 const position(message->point.x, message->point.y, message->point.z);
  ++current_marker_id_;
  auto marker = make_shared<AnnotationMarker>(server_, position, message->header.frame_id, current_marker_id_, labels_);
  markers_.push_back(marker);
}

Markers::Markers()
{
  ros::NodeHandle private_node_handle("~");
  auto const labels = private_node_handle.param<string>("labels", "object");
  std::cout << "Configured labels: " << labels << std::endl;

  istringstream stream(labels);
  string value;
  while (getline(stream, value, ','))
  {
    labels_.push_back(value);
  }

  server_ = make_shared<InteractiveMarkerServer>("annotate_node", "", false);
  subscriber_ = node_handle_.subscribe("/new_annotation", 10, &Markers::createNewAnnotation, this);
}

}  // namespace annotate

int main(int argc, char** argv)
{
  ros::init(argc, argv, "annotate_node");
  annotate::Markers markers;
  ros::spin();
  return 0;
}
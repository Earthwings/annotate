#include <annotate/annotate_tool.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/geometry.h>
#include <rviz/display_context.h>
#include <geometry_msgs/PointStamped.h>
#include <OgreVector3.h>
#include <OgrePlane.h>

namespace annotate
{
void AnnotateTool::onInitialize()
{
  publisher_ = node_handle_.advertise<geometry_msgs::PointStamped>("/new_annotation", 1);
}

int AnnotateTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x, event.y, intersection))
  {
    if (event.leftDown())
    {
      geometry_msgs::PointStamped message;
      message.header.stamp = ros::Time::now();
      message.header.frame_id = context_->getFixedFrame().toStdString();
      message.point.x = intersection.x;
      message.point.y = intersection.y;
      message.point.z = intersection.z;
      publisher_.publish(message);
      return Render | Finished;
    }
  }
  return Render;
}

void AnnotateTool::save(rviz::Config config) const
{
  // Required to restore tool button visibility by ToolManager
  config.mapSetValue("Class", getClassId());
}

void AnnotateTool::activate()
{
  // does nothing
}

void AnnotateTool::deactivate()
{
  // does nothing
}

}  // namespace annotate

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(annotate::AnnotateTool, rviz::Tool)

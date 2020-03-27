#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>
#include <geometry_msgs/PointStamped.h>

#include <annotate/annotate_tool.h>

namespace annotate
{
AnnotateTool::AnnotateTool() : current_annotation_property_(nullptr)
{
  // does nothing
}

void AnnotateTool::onInitialize()
{
  publisher_ = node_handle_.advertise<geometry_msgs::PointStamped>("/new_annotation", 1);
}

void AnnotateTool::activate()
{
  current_annotation_property_ = new rviz::VectorProperty("Annotation");
  current_annotation_property_->setReadOnly(true);
  getPropertyContainer()->addChild(current_annotation_property_);
}

void AnnotateTool::deactivate()
{
  delete current_annotation_property_;
  current_annotation_property_ = nullptr;
}

int AnnotateTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x, event.y, intersection))
  {
    current_annotation_property_->setVector(intersection);
    if (event.leftDown())
    {
      createAnnotation(intersection);
      current_annotation_property_ = nullptr;  // Drop the reference so that deactivate() won't remove it.
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

void AnnotateTool::createAnnotation(const Ogre::Vector3& position)
{
  geometry_msgs::PointStamped message;
  message.header.stamp = ros::Time::now();
  message.header.frame_id = context_->getFixedFrame().toStdString();
  message.point.x = position.x;
  message.point.y = position.y;
  message.point.z = position.z;
  publisher_.publish(message);
}

}  // namespace annotate

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(annotate::AnnotateTool, rviz::Tool)

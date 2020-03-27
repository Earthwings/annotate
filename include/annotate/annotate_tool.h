#pragma once

#include <rviz/tool.h>
#include <ros/ros.h>

namespace Ogre
{
class Vector3;
}

namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
}  // namespace rviz

namespace annotate
{
class AnnotateTool : public rviz::Tool
{
  Q_OBJECT
public:
  AnnotateTool();

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz::ViewportMouseEvent& event) override;
  void save(rviz::Config config) const override;

private:
  void createAnnotation(const Ogre::Vector3& position);

  rviz::VectorProperty* current_annotation_property_;
  ros::NodeHandle node_handle_;
  ros::Publisher publisher_;
};

}  // namespace annotate

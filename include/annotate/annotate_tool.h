#pragma once

#include <rviz/tool.h>
#include <ros/ros.h>

namespace annotate
{
class AnnotateTool : public rviz::Tool
{
  Q_OBJECT
public:
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz::ViewportMouseEvent& event) override;
  void save(rviz::Config config) const override;

private:
  ros::NodeHandle node_handle_;
  ros::Publisher publisher_;
};

}  // namespace annotate

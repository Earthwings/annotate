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

void Markers::createNewAnnotation(const geometry_msgs::PointStamped::ConstPtr& message)
{
  Transform transform;
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
      marker->setIgnoreGround(ignore_ground_);
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

TransformListener& Markers::transformListener()
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
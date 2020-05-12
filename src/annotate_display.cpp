#include <annotate/annotate_display.h>
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
#include <QShortcut>
#include <random>
#include <QFileInfo>
#include <rviz/default_plugin/point_cloud2_display.h>
#include <rviz/default_plugin/marker_array_display.h>
#include <rviz/default_plugin/interactive_marker_display.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <std_srvs/SetBool.h>

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

void AnnotateDisplay::createNewAnnotation(const geometry_msgs::PointStamped::ConstPtr& message)
{
  Transform transform;
  transform.setOrigin({ message->point.x, message->point.y, message->point.z });
  TrackInstance instance;
  instance.center = StampedTransform(transform, time_, message->header.frame_id, "current_annotation");
  instance.label = "unknown";
  instance.box_size.setValue(1.0, 1.0, 1.0);
  ++current_marker_id_;
  auto marker = make_shared<AnnotationMarker>(this, server_, instance, current_marker_id_);
  marker->setLabels(labels_);
  marker->setTags(tags_);
  marker->setIgnoreGround(ignore_ground_property_->getBool());
  marker->setTime(time_);
  markers_.push_back(marker);
}

void AnnotateDisplay::handlePointcloud(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  cloud_ = cloud;
  time_ = cloud->header.stamp;
  if (pause_after_data_change_->getBool())
  {
    sendPlaybackCommand(Pause);
  }
  for (auto& marker : markers_)
  {
    marker->setTime(time_);
  }
  publishTrackMarkers();
}

AnnotateDisplay::AnnotateDisplay()
{
  server_ = make_shared<InteractiveMarkerServer>("annotate_node", "", false);
  track_marker_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("tracks", 10, true);
  new_annotation_subscriber_ =
      node_handle_.subscribe("/new_annotation", 10, &AnnotateDisplay::createNewAnnotation, this);
}

template <class T>
void AnnotateDisplay::modifyChild(rviz::Property* parent, QString const& name, std::function<void(T*)> modifier)
{
  for (int i = 0; i < parent->numChildren(); ++i)
  {
    auto* property = dynamic_cast<T*>(parent->childAt(i));
    if (property && (name.isEmpty() || property->getName() == name))
    {
      modifier(property);
    }
  }
}

void AnnotateDisplay::autoFitPoints()
{
  if (current_marker_)
  {
    current_marker_->autoFit();
  }
}

void AnnotateDisplay::undo()
{
  if (current_marker_)
  {
    current_marker_->undo();
  }
}

void AnnotateDisplay::commit()
{
  if (current_marker_)
  {
    current_marker_->commit();
    if (play_after_commit_->getBool())
    {
      sendPlaybackCommand(Play);
    }
  }
}

void AnnotateDisplay::rotateClockwise()
{
  if (current_marker_)
  {
    current_marker_->rotateYaw(-0.01);
  }
}

void AnnotateDisplay::rotateAntiClockwise()
{
  if (current_marker_)
  {
    current_marker_->rotateYaw(0.01);
  }
}

void AnnotateDisplay::sendPlaybackCommand(PlaybackCommand command)
{
  if (!playback_client_.isValid())
  {
    QStringList services;
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = "/annotate";

    if (!ros::master::execute("getSystemState", args, result, payload, true))
    {
      return;
    }

    for (int i = 0; i < payload.size(); ++i)
    {
      for (int j = 0; j < payload[i].size(); ++j)
      {
        XmlRpc::XmlRpcValue val = payload[i][j];
        if (val.size() > 0)
        {
          string const ending = "/pause_playback";
          string const value = val[0];
          auto const v = value.length();
          auto const e = ending.length();
          bool const ends_with = v >= e && 0 == value.compare(v - e, e, ending);
          if (ends_with)
          {
            services.push_back(QString::fromStdString(value));
          }
        }
      }
    }

    QString const status_name = "rosbag play service";
    if (services.empty())
    {
      setStatus(rviz::StatusProperty::Warn, status_name,
                "Found no pause_playback ROS service to toggle playback. Maybe 'rosbag play' is not running?");
    }
    else if (services.size() == 1)
    {
      ros::NodeHandle node_handle;
      playback_client_ = node_handle.serviceClient<std_srvs::SetBool>(services.front().toStdString());
      deleteStatus(status_name);
    }
    else
    {
      QString message = "Found multiple pause_playback ROS services to toggle playback: %1";
      setStatus(rviz::StatusProperty::Warn, status_name, message.arg(services.join(", ")));
    }
  }

  if (playback_client_.isValid())
  {
    std_srvs::SetBool value;
    value.request.data = command == Toggle || command == Pause;
    if (playback_client_.call(value))
    {
      if (command == Toggle && !value.response.success)
      {
        value.request.data = !value.request.data;
        if (playback_client_.call(value))
        {
          if (!value.response.success)
          {
            ROS_WARN_STREAM("Play/pause toggle failed.");
          }
        }
      }
    }
  }
}

void AnnotateDisplay::togglePlayPause()
{
  sendPlaybackCommand(Toggle);
}

void AnnotateDisplay::updateShortcuts()
{
  auto const enabled = shortcuts_property_->getBool();
  for (int i = 0; i < shortcuts_property_->numChildren(); ++i)
  {
    auto* child = shortcuts_property_->childAt(i);
    ShortcutProperty* property = qobject_cast<ShortcutProperty*>(child);
    if (property)
    {
      property->setEnabled(enabled);
    }
  }
}

void AnnotateDisplay::onInitialize()
{
  cloud_display_ = createDisplay("rviz/PointCloud2");
  addDisplay(cloud_display_);
  cloud_display_->initialize(context_);
  cloud_display_->setName("Point Cloud");
  cloud_display_->setEnabled(true);

  marker_display_ = createDisplay("rviz/InteractiveMarkers");
  addDisplay(marker_display_);
  marker_display_->initialize(context_);
  marker_display_->setName("Annotations");
  marker_display_->setTopic("/annotate_node/update", "visualization_msgs/InteractiveMarkerUpdate");
  marker_display_->setEnabled(true);
  modifyChild<rviz::BoolProperty>(marker_display_, "Show Axes",
                                  [](rviz::BoolProperty* property) { property->setBool(true); });

  track_display_ = createDisplay("rviz/MarkerArray");
  addDisplay(track_display_);
  track_display_->initialize(context_);
  track_display_->setName("Tracks");
  track_display_->setTopic("/tracks", "visualization_msgs/MarkerArray");
  track_display_->setEnabled(true);

  topic_property_ = new rviz::RosTopicProperty("Topic", QString(), "sensor_msgs/PointCloud2", "Point cloud to annotate",
                                               this, SLOT(updateTopic()), this);
  open_file_property_ = new FileDialogProperty("Open", QString(), "Open an existing annotation file for editing", this,
                                               SLOT(openFile()), this);
  open_file_property_->setIcon(rviz::loadPixmap(QString("package://annotate/icons/open.svg")));
  open_file_property_->setMode(FileDialogProperty::OpenFileName);
  annotation_file_property_ = new FileDialogProperty("Annotation File", "annotate.yaml", "Annotation storage file",
                                                     this, SLOT(updateAnnotationFile()), this);
  annotation_file_property_->setIcon(rviz::loadPixmap(QString("package://annotate/icons/save.svg")));
  annotation_file_property_->setMode(FileDialogProperty::SaveFileName);
  labels_property_ = new rviz::StringProperty("Labels", "object, unknown", "Available labels (separated by comma)",
                                              this, SLOT(updateLabels()), this);
  labels_property_->setIcon(rviz::loadPixmap(QString("package://annotate/icons/labels.svg")));
  tags_property_ = new rviz::StringProperty("Tags", "easy, moderate, hard", "Available tags (separated by comma)", this,
                                            SLOT(updateTags()), this);
  tags_property_->setIcon(rviz::loadPixmap(QString("package://annotate/icons/tags.svg")));
  ignore_ground_property_ = new rviz::BoolProperty("Ignore Ground", false,
                                                   "Enable to ignore the ground direction (negative z) when "
                                                   "shrinking "
                                                   "or fitting boxes. This is useful if the point cloud contains "
                                                   "ground points that should not be included in annotations.",
                                                   this, SLOT(updateIgnoreGround()), this);

  auto* automations =
      new rviz::Property("Linked Actions", QVariant(), "Configure the interaction of related actions.", this);
  automations->setIcon(rviz::loadPixmap("package://annotate/icons/automations.svg"));
  shrink_after_resize_ = new rviz::BoolProperty("Shrink after resize", false,
                                                "Shrink annotation box to fit points after adjusting its size with the "
                                                "handles.",
                                                automations);
  shrink_before_commit_ = new rviz::BoolProperty(
      "Shrink before commit", true, "Shrink annotation box to fit points when committing an annotation.", automations);
  auto_fit_after_predict_ = new rviz::BoolProperty(
      "Auto-fit after points change", false, "Auto-fit annotation boxes when the point cloud changes.", automations);
  pause_after_data_change_ = new rviz::BoolProperty("Pause playback after points change", false,
                                                    "Pause playback when the point cloud changes.", automations);
  play_after_commit_ = new rviz::BoolProperty("Resume playback after commit", false,
                                              "Resume playback after committing an annotation", automations);

  auto* render_panel = context_->getViewManager()->getRenderPanel();
  shortcuts_property_ =
      new BoolProperty("Keyboard Shortcuts", true, "Keyboard shortcuts that affect the currently selected annotation",
                       this, SLOT(updateShortcuts()), this);
  shortcuts_property_->setDisableChildrenIfFalse(true);
  QIcon icon = rviz::loadPixmap("package://annotate/icons/keyboard.svg");
  shortcuts_property_->setIcon(icon);

  auto* undo = new ShortcutProperty("undo", "Ctrl+Z", "Undo last action", shortcuts_property_);
  undo->createShortcut(this, render_panel, this, SLOT(undo()));

  auto* play_pause =
      new ShortcutProperty("toggle pause", "space", "Toggle play and pause state of rosbag play", shortcuts_property_);
  play_pause->createShortcut(this, render_panel, this, SLOT(togglePlayPause()));

  auto* rotate_clockwise_ =
      new ShortcutProperty("rotate clockwise", "right", "Rotate the current annotation clockwise", shortcuts_property_);
  rotate_clockwise_->createShortcut(this, render_panel, this, SLOT(rotateClockwise()));
  auto* rotate_anti_clockwise_ = new ShortcutProperty(
      "rotate anti-clockwise", "left", "Rotate the current annotation anti-clockwise", shortcuts_property_);
  rotate_anti_clockwise_->createShortcut(this, render_panel, this, SLOT(rotateAntiClockwise()));

  auto* auto_fit =
      new ShortcutProperty("auto fit points", "Ctrl+F", "Auto-fit annotation to nearby points", shortcuts_property_);
  auto_fit->createShortcut(this, render_panel, this, SLOT(autoFitPoints()));
  auto* commit =
      new ShortcutProperty("commit annotation", "return", "Commit current annotation and save", shortcuts_property_);
  commit->createShortcut(this, render_panel, this, SLOT(commit()));

  adjustView();
  expand();
}

void AnnotateDisplay::load(const rviz::Config& config)
{
  /*
  Display::load(config);

  auto const display_list_config = config.mapGetChild("Displays");
  for (int i = 0; i < display_list_config.listLength(); ++i)
  {
    auto const display_config = display_list_config.listChildAt(i);
    QString display_class = "(no class name found)";
    display_config.mapGetString("Class", &display_class);
    QString display_name;
    display_config.mapGetString("Name", &display_name);
    if (display_class == "rviz/InteractiveMarkers")
    {
      marker_display_->load(display_config);
      marker_display_->setObjectName(display_name);
    }
    else if (display_class == "rviz/MarkerArray")
    {
      track_display_->load(display_config);
      track_display_->setObjectName(display_name);
    }
    else if (display_class == "rviz/PointCloud2")
    {
      cloud_display_->load(display_config);
      cloud_display_->setObjectName(display_name);
    }
  }
  adjustView();
  */

  cloud_display_ = nullptr;
  track_display_ = nullptr;
  marker_display_ = nullptr;
  DisplayGroup::load(config);
  modifyChild<rviz::PointCloud2Display>(this, QString(),
                                        [this](rviz::PointCloud2Display* property) { cloud_display_ = property; });
  modifyChild<rviz::MarkerArrayDisplay>(this, QString(),
                                        [this](rviz::MarkerArrayDisplay* property) { track_display_ = property; });
  modifyChild<rviz::InteractiveMarkerDisplay>(
      this, QString(), [this](rviz::InteractiveMarkerDisplay* property) { marker_display_ = property; });
  adjustView();
}

void AnnotateDisplay::adjustView()
{
  for (auto display : { cloud_display_, marker_display_, track_display_ })
  {
    modifyChild<rviz::RosTopicProperty>(display, QString(),
                                        [](rviz::RosTopicProperty* property) { property->setHidden(true); });
  }
}

void AnnotateDisplay::setTopic(const QString& topic, const QString& datatype)
{
  if (topic_property_)
  {
    topic_property_->setString(topic);
  }
  pointcloud_subscriber_ = node_handle_.subscribe(topic.toStdString(), 10, &AnnotateDisplay::handlePointcloud, this);
  if (cloud_display_)
  {
    cloud_display_->setTopic(topic, datatype);
  }
}

void AnnotateDisplay::updateTopic()
{
  if (topic_property_)
  {
    setTopic(topic_property_->getTopic(), topic_property_->getMessageType());
  }
}

void AnnotateDisplay::updateLabels()
{
  labels_.clear();
  auto const labels = labels_property_->getString().split(',', QString::SkipEmptyParts);
  for (auto const& label : labels)
  {
    labels_.push_back(label.trimmed().toStdString());
  }
  for (auto const& marker : markers_)
  {
    marker->setLabels(labels_);
  }
}

void AnnotateDisplay::updateTags()
{
  tags_.clear();
  auto const tags = tags_property_->getString().split(',', QString::SkipEmptyParts);
  for (auto const& tag : tags)
  {
    tags_.push_back(tag.trimmed().toStdString());
  }
  for (auto const& marker : markers_)
  {
    marker->setTags(tags_);
  }
}

void AnnotateDisplay::openFile()
{
  auto const file = open_file_property_->getValue().toString();
  if (!file.isEmpty())
  {
    server_->clear();
    server_->applyChanges();
    markers_.clear();
    open_file_property_->setValue(QString());
    if (load(file.toStdString()))
    {
      annotation_file_property_->blockSignals(true);
      annotation_file_property_->setValue(file);
      filename_ = file.toStdString();
      annotation_file_property_->blockSignals(false);
    }
  }
}

void AnnotateDisplay::updateAnnotationFile()
{
  auto const file = annotation_file_property_->getValue().toString();
  if (filename_.empty() && QFileInfo::exists(file))
  {
    if (load(file.toStdString()))
    {
      filename_ = file.toStdString();
    }
  }
  else
  {
    filename_ = file.toStdString();
    save();
  }
}

void AnnotateDisplay::updateIgnoreGround()
{
  auto const ignore_ground = ignore_ground_property_->getBool();
  for (auto const& marker : markers_)
  {
    marker->setIgnoreGround(ignore_ground);
  }
}

bool AnnotateDisplay::load(string const& file)
{
  using namespace YAML;
  Node node;
  try
  {
    node = LoadFile(file);
  }
  catch (Exception const& e)
  {
    stringstream stream;
    stream << "Failed to open " << file << ": " << e.msg;
    ROS_DEBUG_STREAM(stream.str());
    setStatusStd(rviz::StatusProperty::Error, "Annotation File", stream.str());
    return false;
  }
  labels_.clear();
  Node labels = node["labels"];
  string joined_labels;
  for (size_t i = 0; i < labels.size(); ++i)
  {
    auto const value = labels[i].as<string>();
    labels_.push_back(value);
    if (joined_labels.empty())
    {
      joined_labels = value;
    }
    else
    {
      joined_labels += ", " + value;
    }
  }
  labels_property_->setStdString(joined_labels);
  tags_.clear();
  Node tags = node["tags"];
  string joined_tags;
  for (size_t i = 0; i < tags.size(); ++i)
  {
    auto const value = tags[i].as<string>();
    tags_.push_back(value);
    if (joined_tags.empty())
    {
      joined_tags = value;
    }
    else
    {
      joined_tags += ", " + value;
    }
  }
  tags_property_->setStdString(joined_tags);
  Node tracks = node["tracks"];
  size_t annotations = 0;
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
      if (inst["tags"])
      {
        for (auto const& tag : inst["tags"].as<vector<string>>())
        {
          instance.tags.push_back(tag);
        }
      }

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
      ++annotations;
    }

    if (!track.empty())
    {
      current_marker_id_ = max(current_marker_id_, id);
      auto marker = make_shared<AnnotationMarker>(this, server_, track.front(), id);
      marker->setLabels(labels_);
      marker->setTrack(track);
      bool ignore_ground = ignore_ground_property_ ? ignore_ground_property_->getBool() : false;
      marker->setIgnoreGround(ignore_ground);
      marker->setTime(time_);
      markers_.push_back(marker);
    }
  }
  stringstream stream;
  stream << "Loaded " << markers_.size() << " tracks with " << annotations << " annotations";
  setStatusStd(rviz::StatusProperty::Ok, "Annotation File", stream.str());
  publishTrackMarkers();
  return true;
}

bool AnnotateDisplay::save()
{
  using namespace YAML;
  Node node;
  size_t annotations = 0;
  for (auto const& label : labels_)
  {
    node["labels"].push_back(label);
  }
  for (auto const& tag : tags_)
  {
    node["tags"].push_back(tag);
  }
  for (auto const& marker : markers_)
  {
    Node annotation;
    annotation["id"] = marker->id();
    for (auto const& instance : marker->track())
    {
      Node i;
      i["label"] = instance.label;

      Node tags;
      for (auto const& tag : instance.tags)
      {
        tags.push_back(tag);
      }
      i["tags"] = tags;

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
      ++annotations;
    }
    node["tracks"].push_back(annotation);
  }

  ofstream stream(filename_);
  if (stream.is_open())
  {
    stream << node;
    if (stream.bad())
    {
      stringstream status_stream;
      status_stream << "Failed to write annotations to " << filename_;
      setStatusStd(rviz::StatusProperty::Error, "Annotation File", status_stream.str());
      ROS_WARN_STREAM(status_stream.str());
      return false;
    }
  }
  else
  {
    stringstream status_stream;
    status_stream << "Failed to open " << filename_ << " for writing. Annotations will not be saved.";
    setStatusStd(rviz::StatusProperty::Error, "Annotation File", status_stream.str());
    ROS_WARN_STREAM(status_stream.str());
    return false;
  }
  stream.close();
  stringstream status_stream;
  status_stream << "Saved " << markers_.size() << " tracks with " << annotations << " annotations";
  setStatusStd(rviz::StatusProperty::Ok, "Annotation File", status_stream.str());
  return true;
}

void AnnotateDisplay::publishTrackMarkers()
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

sensor_msgs::PointCloud2ConstPtr AnnotateDisplay::cloud() const
{
  return cloud_;
}

TransformListener& AnnotateDisplay::transformListener()
{
  return transform_listener_;
}

void AnnotateDisplay::setCurrentMarker(AnnotationMarker* marker)
{
  current_marker_ = marker;
}

bool AnnotateDisplay::shrinkAfterResize() const
{
  return shrink_after_resize_ && shrink_after_resize_->getBool();
}

bool AnnotateDisplay::shrinkBeforeCommit() const
{
  return shrink_before_commit_ && shrink_before_commit_->getBool();
}

bool AnnotateDisplay::autoFitAfterPredict() const
{
  return auto_fit_after_predict_ && auto_fit_after_predict_->getBool();
}

}  // namespace annotate

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(annotate::AnnotateDisplay, rviz::Display)

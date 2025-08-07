#include "mpc_tools/ros_visuals.h"

std::vector<double> ROSMarker::VIRIDIS_ = {
    253, 231, 37,  234, 229, 26,  210, 226, 27,  186, 222, 40,  162, 218,
    55,  139, 214, 70,  119, 209, 83,  99,  203, 95,  80,  196, 106, 63,
    188, 115, 49,  181, 123, 38,  173, 129, 33,  165, 133, 30,  157, 137,
    31,  148, 140, 34,  140, 141, 37,  131, 142, 41,  123, 142, 44,  115,
    142, 47,  107, 142, 51,  98,  141, 56,  89,  140};
std::vector<double> ROSMarker::INFERNO_ = {
    252, 255, 164, 241, 237, 113, 246, 213, 67,  251, 186, 31,  252, 161,
    8,   248, 135, 14,  241, 113, 31,  229, 92,  48,  215, 75,  63,  196,
    60,  78,  177, 50,  90,  155, 41,  100, 135, 33,  107, 113, 25,  110,
    92,  18,  110, 69,  10,  105, 47,  10,  91,  24,  12,  60};
std::vector<double> ROSMarker::BRUNO_ = {
    217, 83, 25,  0,   114, 189, 119, 172, 48,  126, 47, 142, 237, 177,
    32,  77, 190, 238, 162, 19,  47,  256, 153, 256, 0,  103, 256};
// marker_list_ holds the rosmarkers to draw
// ros_markers_ is the list of ROSMarker objects defined in this file

ROSMarkerPublisher::ROSMarkerPublisher(ros::NodeHandle &nh,
                                       const char *topic_name,
                                       const std::string &frame_id,
                                       int max_size)
    : frame_id_(frame_id), max_size_(max_size) {
  pub_ = nh.advertise<visualization_msgs::MarkerArray>(topic_name, 3);
  id_ = 0;
  prev_id_ = 0;

  topic_name_ = std::string(topic_name);

  // We allocate space for the markers in initialization
  marker_list_.markers.reserve(max_size);
  ros_markers_.reserve(max_size);
  // prev_marker_list_.markers.reserve(max_size);

  // for (int i = 0; i < max_size; i++)
  // {
  //     marker_list_.markers[i].id = i;
  //     // marker_list_.markers[i].action = visualization_msgs::Marker::DELETE;
  //     marker_list_.markers[i].header.frame_id = frame_id_;
  // }
}

void ROSMarkerPublisher::add(const visualization_msgs::Marker &marker) {
  // If we exceed the max size, allocate 1.5 times the space and give a warning
  if (marker.id > max_size_ - 1) {
    double prev_size = max_size_;
    // If we exceed the max size, allocate 1.5 times the space and give an error
    max_size_ = std::ceil(max_size_ * 1.5);
    ROS_WARN_STREAM("ROS VISUALS (topic = "
                    << topic_name_ << "): exceeded max size of " << prev_size
                    << ". New size: " << max_size_);

    max_size_ *= 1.5;
    marker_list_.markers.reserve(max_size_);
    ros_markers_.reserve(max_size_);
  }

  // Add the marker
  marker_list_.markers.push_back(marker);
}

ROSLine &ROSMarkerPublisher::getNewLine() {
  // Create a line
  // For vector of std::unique_ptr preferably use push_back (not emplace_back),
  // see https://stackoverflow.com/a/15784813
  ros_markers_.push_back(
      std::unique_ptr<ROSMarker>(new ROSLine(this, frame_id_)));

  // Return a pointer
  return *((ROSLine *)(ros_markers_.back().get()));
}

ROSPointMarker &ROSMarkerPublisher::getNewPointMarker(
    const std::string &marker_type) {
  // Create a point marker
  ros_markers_.push_back(std::unique_ptr<ROSMarker>(
      new ROSPointMarker(this, frame_id_, marker_type)));

  // Return a pointer
  return *((ROSPointMarker *)(ros_markers_.back().get()));
}

ROSMultiplePointMarker &ROSMarkerPublisher::getNewMultiplePointMarker(
    const std::string &marker_type) {
  // Create a point marker
  ros_markers_.push_back(std::unique_ptr<ROSMultiplePointMarker>(
      new ROSMultiplePointMarker(this, frame_id_, marker_type)));

  // Return a pointer
  return *((ROSMultiplePointMarker *)(ros_markers_.back().get()));
}

ROSTextMarker &ROSMarkerPublisher::getNewTextMarker() {
  // Create a point marker
  ros_markers_.emplace_back(nullptr);
  ros_markers_.back().reset(new ROSTextMarker(this, frame_id_));

  // Return a pointer
  return (*((ROSTextMarker *)(&(*ros_markers_.back()))));
}

ROSModelMarker &ROSMarkerPublisher::getNewModelMarker(
    const std::string &model_path) {
  // Create a point marker
  ros_markers_.emplace_back(nullptr);
  ros_markers_.back().reset(new ROSModelMarker(this, frame_id_, model_path));

  // Return a pointer
  return (*((ROSModelMarker *)(&(*ros_markers_.back()))));
}

void ROSMarkerPublisher::publish(bool keep_markers) {
  // If less markers are published, remove the extra markers explicitly
  visualization_msgs::Marker remove_marker_;
  remove_marker_.action = visualization_msgs::Marker::DELETE;
  remove_marker_.header.frame_id = frame_id_;

  if (prev_id_ > id_) {
    for (int i = id_; i < prev_id_; i++) {
      remove_marker_.id = i;
      marker_list_.markers.push_back(remove_marker_);
    }
  }

  // Add new markers
  for (std::unique_ptr<ROSMarker> &marker : ros_markers_) marker->stamp();

  pub_.publish(marker_list_);

  if (!keep_markers) {
    // Clear marker data for the next iteration
    marker_list_.markers.clear();
    ros_markers_.clear();

    prev_id_ = id_;
    id_ = 0;
  }
}

ROSMarkerPublisher::~ROSMarkerPublisher() {
  // for (visualization_msgs::Marker &marker : prev_marker_list_.markers)
  // {
  //     marker.action = visualization_msgs::Marker::DELETE;
  // }

  // pub_.publish(prev_marker_list_);

  visualization_msgs::Marker remove_marker_;
  remove_marker_.action = visualization_msgs::Marker::DELETE;
  remove_marker_.header.frame_id = frame_id_;

  // Remove all markers
  marker_list_.markers.clear();

  for (int i = 0; i < prev_id_; i++) {
    remove_marker_.id = i;
    marker_list_.markers.push_back(remove_marker_);
  }
  pub_.publish(marker_list_);
}

int ROSMarkerPublisher::getID() {
  int cur_id = id_;
  id_++;

  return cur_id;
}

ROSMarker::ROSMarker(ROSMarkerPublisher *ros_publisher,
                     const std::string &frame_id)
    : ros_publisher_(ros_publisher) {
  marker_.header.frame_id = frame_id;
}

inline void ROSMarker::stamp() { marker_.header.stamp = ros::Time::now(); }

void ROSMarker::setColor(double r, double g, double b, double alpha) {
  marker_.color.r = r;
  marker_.color.g = g;
  marker_.color.b = b;
  marker_.color.a = alpha;
}

void ROSMarker::setColor(double ratio, double alpha) {
  double red, green, blue;

  getColorFromRange(ratio, red, green, blue);
  setColor(red, green, blue, alpha);
}

void ROSMarker::setColorInt(int select, double alpha,
                            const Colormap &&colormap) {
  double red, green, blue;

  getColorFromRangeInt(select, red, green, blue, colormap);
  setColor(red, green, blue, alpha);
}

void ROSMarker::setColorInt(int select, int range, double alpha,
                            const Colormap &&colormap) {
  double red, green, blue;

  select = std::floor(((double)select) *
                      (((double)(getColors(colormap).size() / 3)) /
                       ((double)range)));  // Scale the selection with the range

  getColorFromRangeInt(select, red, green, blue, colormap);
  setColor(red, green, blue, alpha);

  // if (colormap == Colormap::VIRIDIS)
  //     std::cout << std::hex << (int)(red * 256) << (int)(green * 256) <<
  //     (int)(blue * 256) << std::dec << std::endl;
}

void ROSMarker::setScale(double x, double y, double z) {
  marker_.scale.x = x;
  marker_.scale.y = y;
  marker_.scale.z = z;
}

void ROSMarker::setScale(double x, double y) {
  marker_.scale.x = x;
  marker_.scale.y = y;
}

void ROSMarker::setScale(double x) { marker_.scale.x = x; }

void ROSMarker::setOrientation(double psi) {
  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, psi);
  setOrientation(q);
}

void ROSMarker::setOrientation(const geometry_msgs::Quaternion &msg) {
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg, q);
  setOrientation(q);
}

void ROSMarker::setOrientation(const tf::Quaternion &q) {
  marker_.pose.orientation.x = q.x();
  marker_.pose.orientation.y = q.y();
  marker_.pose.orientation.z = q.z();
  marker_.pose.orientation.w = q.w();
}

inline void ROSMarker::setLifetime(double lifetime) {
  marker_.lifetime = ros::Duration(lifetime);
}

inline void ROSMarker::setActionDelete() {
  marker_.action = visualization_msgs::Marker::DELETE;
}

geometry_msgs::Point ROSMarker::vecToPoint(const Eigen::Vector3d &v) {
  geometry_msgs::Point p;
  p.x = v(0);
  p.y = v(1);
  p.z = v(2);
  return p;
}

void ROSMarker::getColorFromRangeInt(int select, double &red, double &green,
                                     double &blue, const Colormap &colormap) {
  // // Obtained from https://waldyrious.net/viridis-palette-generator/
  // std::vector<int> viridis_vals = {
  //     253, 231, 37, 234, 229, 26, 210, 226, 27, 186, 222, 40, 162, 218, 55,
  //     139, 214, 70, 119, 209, 83, 99, 203, 95, 80, 196, 106, 63, 188, 115,
  //     49, 181, 123, 38, 173, 129, 33, 165, 133, 30, 157, 137, 31, 148, 140,
  //     34, 140, 141, 37, 131, 142, 41, 123, 142, 44, 115, 142, 47, 107, 142,
  //     51, 98, 141, 56, 89, 140};

  std::vector<double> &colors = getColors(colormap);

  select %= (int)(colors.size() / 3);  // We only have 20 values
  // Invert the color range
  select = (int)(colors.size() / 3) - 1 - select;

  red = colors[select * 3 + 0];
  green = colors[select * 3 + 1];
  blue = colors[select * 3 + 2];

  red /= 256.0;
  green /= 256.0;
  blue /= 256.0;
}

std::vector<double> &ROSMarker::getColors(const Colormap &colormap) {
  switch (colormap) {
    case Colormap::VIRIDIS:
      return ROSMarker::VIRIDIS_;
      break;
    case Colormap::INFERNO:
      return ROSMarker::INFERNO_;
      break;
    case Colormap::BRUNO:
      return ROSMarker::BRUNO_;
      break;
    default:
      throw std::runtime_error("Invalid colormap given");
  }
}

void ROSMarker::getColorFromRange(double ratio, double &red, double &green,
                                  double &blue) {
  // we want to normalize ratio so that it fits in to 6 regions
  // where each region is 256 units long
  int normalized = int(ratio * 256 * 6);

  // find the distance to the start of the closest region
  int x = normalized % 256;

  switch (normalized / 256) {
    case 0:
      red = 255;
      green = x;
      blue = 0;
      break;  // red
    case 1:
      red = 255 - x;
      green = 255;
      blue = 0;
      break;  // yellow
    case 2:
      red = 0;
      green = 255;
      blue = x;
      break;  // green
    case 3:
      red = 0;
      green = 255 - x;
      blue = 255;
      break;  // cyan
    case 4:
      red = x;
      green = 0;
      blue = 255;
      break;  // blue
    case 5:
      red = 255;
      green = 0;
      blue = 255 - x;
      break;  // magenta
  }

  red /= 256.0;
  green /= 256.0;
  blue /= 256.0;
}

ROSLine::ROSLine(ROSMarkerPublisher *ros_publisher, const std::string &frame_id)
    : ROSMarker(ros_publisher, frame_id) {
  marker_.type = visualization_msgs::Marker::LINE_LIST;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the
  // line width
  marker_.scale.x = 0.5;
  // marker_.scale.y = 0.5;

  // Line strip is red
  setColor(1, 0, 0);

  marker_.pose.orientation.x = 0.0;
  marker_.pose.orientation.y = 0.0;
  marker_.pose.orientation.z = 0.0;
  marker_.pose.orientation.w = 1.0;
}

void ROSLine::addLine(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2) {
  addLine(vecToPoint(p1), vecToPoint(p2));
}

void ROSLine::addLine(const geometry_msgs::Point &p1,
                      const geometry_msgs::Point &p2) {
  // Request an ID
  marker_.id = ros_publisher_->getID();

  // Add the points
  marker_.points.push_back(p1);
  marker_.points.push_back(p2);

  // Add line to the publisher
  ros_publisher_->add(marker_);

  // Clear points
  marker_.points.clear();
}

void ROSLine::addBrokenLine(const Eigen::Vector3d &p1,
                            const Eigen::Vector3d &p2, double dist) {
  addBrokenLine(vecToPoint(p1), vecToPoint(p2), dist);
}

// Not perfect, but good enough
void ROSLine::addBrokenLine(const geometry_msgs::Point &p1,
                            const geometry_msgs::Point &p2, double dist) {
  // Interpolate the points, ensure 0.5 of the broken line at both sides!
  double dpoints =
      std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
  int num_lines = std::floor(dpoints / (2.0 * dist));  // Also includes spaces!
  double extra = dpoints - num_lines * (2.0 * dist);   // The difference

  int num_elements = num_lines * 2;

  Eigen::Vector2d dir_vec =
      Eigen::Vector2d(p2.x - p1.x, p2.y - p1.y).normalized();

  geometry_msgs::Point cur_p = p1;
  for (int i = 0; i < num_elements + 1;
       i++)  // The first line is split in an end and start
  {
    geometry_msgs::Point next_p;

    if (i == 0 || i == num_elements) {
      next_p.x = cur_p.x + dir_vec(0) * 0.5 * (dist + extra / 2.0);
      next_p.y = cur_p.y + dir_vec(1) * 0.5 * (dist + extra / 2.0);
    } else {
      next_p.x = cur_p.x + dir_vec(0) * dist;
      next_p.y = cur_p.y + dir_vec(1) * dist;
    }
    if (i % 2 == 0)  // If this is a line, not a space
      addLine(cur_p, next_p);

    cur_p = next_p;
  }
}

void ROSLine::addCircularLine(const Eigen::Vector2d &center, double radius,
                              double z, double n_points) {
  // Create n_points points in a circle centered at center with specific radius
  // at height z
  std::vector<geometry_msgs::Point> points(n_points);
  for (int i = 0; i < n_points; i++) {
    points[i].x = center(0) + radius * std::cos(2 * M_PI * i / n_points);
    points[i].y = center(1) + radius * std::sin(2 * M_PI * i / n_points);
    points[i].z = z;
  }

  // Create marker
  marker_.id = ros_publisher_->getID();
  for (int i = 0; i < n_points; i++) {
    if (i == n_points - 1) {
      marker_.points.push_back(points[i]);
      marker_.points.push_back(points[0]);
    } else {
      marker_.points.push_back(points[i]);
      marker_.points.push_back(points[i + 1]);
    }
  }

  // Add marker to marker publisher
  ros_publisher_->add(marker_);

  // Clear points
  marker_.points.clear();
}

ROSPointMarker::ROSPointMarker(ROSMarkerPublisher *ros_publisher,
                               const std::string &frame_id,
                               const std::string &marker_type)
    : ROSMarker(ros_publisher, frame_id), marker_type_(marker_type) {
  marker_.type = getMarkerType(marker_type_);

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the
  // line width
  marker_.scale.x = 0.5;
  marker_.scale.y = 0.5;

  setColor(1., 0., 0.);

  marker_.pose.orientation.x = 0.0;
  marker_.pose.orientation.y = 0.0;
  marker_.pose.orientation.z = 0.0;
  marker_.pose.orientation.w = 1.0;
}

void ROSPointMarker::addPointMarker(const geometry_msgs::Pose &pose) {
  // Request an ID
  marker_.id = ros_publisher_->getID();

  marker_.pose = pose;

  // Add line to the publisher
  ros_publisher_->add(marker_);
}

void ROSPointMarker::addPointMarker(const Eigen::Vector3d &p1) {
  addPointMarker(vecToPoint(p1));
}

void ROSPointMarker::addPointMarker(const geometry_msgs::Point &p1) {
  // Request an ID
  marker_.id = ros_publisher_->getID();

  marker_.pose.position = p1;

  // Add line to the publisher
  ros_publisher_->add(marker_);
}

inline void ROSPointMarker::setZ(double z) { marker_.pose.position.z = z; }

uint ROSPointMarker::getMarkerType(const std::string &marker_type) {
  if (marker_type == "CUBE") return visualization_msgs::Marker::CUBE;
  if (marker_type == "ARROW") return visualization_msgs::Marker::ARROW;
  if (marker_type == "SPHERE") return visualization_msgs::Marker::SPHERE;
  if (marker_type == "POINTS") return visualization_msgs::Marker::POINTS;
  if (marker_type == "CYLINDER") return visualization_msgs::Marker::CYLINDER;

  return visualization_msgs::Marker::CUBE;
}

ROSMultiplePointMarker::ROSMultiplePointMarker(
    ROSMarkerPublisher *ros_publisher, const std::string &frame_id,
    const std::string &type = "POINTS")
    : ROSMarker(ros_publisher, frame_id) {
  marker_.type = getMultipleMarkerType(type);

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the
  // line width
  marker_.scale.x = 0.5;
  marker_.scale.y = 0.5;

  setColor(0, 1, 0);

  marker_.pose.orientation.x = 0.0;
  marker_.pose.orientation.y = 0.0;
  marker_.pose.orientation.z = 0.0;
  marker_.pose.orientation.w = 1.0;
}

inline void ROSMultiplePointMarker::addPointMarker(
    const geometry_msgs::Point &p1) {
  marker_.points.push_back(p1);
}

void ROSMultiplePointMarker::addPointMarker(const Eigen::Vector3d &p1) {
  geometry_msgs::Point result;
  result.x = p1(0);
  result.y = p1(1);
  result.z = p1(2);

  addPointMarker(result);
}

void ROSMultiplePointMarker::addPointMarker(const geometry_msgs::Pose &pose) {
  geometry_msgs::Point result;
  result.x = pose.position.x;
  result.y = pose.position.y;
  result.z = pose.position.z;

  addPointMarker(result);
}

void ROSMultiplePointMarker::finishPoints() {
  marker_.id = ros_publisher_->getID();
  ros_publisher_->add(marker_);
}

uint ROSMultiplePointMarker::getMultipleMarkerType(
    const std::string &marker_type) {
  if (marker_type == "CUBE") return visualization_msgs::Marker::CUBE_LIST;
  if (marker_type == "SPHERE") return visualization_msgs::Marker::SPHERE_LIST;
  if (marker_type == "POINTS") return visualization_msgs::Marker::POINTS;

  return visualization_msgs::Marker::CUBE_LIST;
}

ROSTextMarker::ROSTextMarker(ROSMarkerPublisher *ros_publisher,
                             const std::string &frame_id)
    : ROSMarker(ros_publisher, frame_id) {
  marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the
  // line width
  marker_.scale.x = 1.;   // Scale "z" sets the size of an uppercase "A"
  marker_.scale.y = 1.;   // Scale "z" sets the size of an uppercase "A"
  marker_.scale.z = 0.2;  // Scale "z" sets the size of an uppercase "A"
  marker_.text = "TEST";

  marker_.color.a = 1.0;

  setColor(0, 1, 0);

  marker_.pose.orientation.x = 0.0;
  marker_.pose.orientation.y = 0.0;
  marker_.pose.orientation.z = 0.0;
  marker_.pose.orientation.w = 1.0;
}

void ROSTextMarker::addPointMarker(const Eigen::Vector3d &p1) {
  geometry_msgs::Point result;
  result.x = p1(0);
  result.y = p1(1);
  result.z = p1(2);

  addPointMarker(result);
}

void ROSTextMarker::addPointMarker(const geometry_msgs::Point &p1) {
  // Request an ID
  marker_.id = ros_publisher_->getID();

  marker_.pose.position = p1;

  ros_publisher_->add(marker_);
}

void ROSTextMarker::addPointMarker(const geometry_msgs::Pose &pose) {
  geometry_msgs::Point result;
  result.x = pose.position.x;
  result.y = pose.position.y;
  result.z = pose.position.z;

  addPointMarker(result);
}

inline void ROSTextMarker::setZ(double z) { marker_.pose.position.z = z; }

inline void ROSTextMarker::setText(const std::string &text) {
  marker_.text = text;
}

inline void ROSTextMarker::setText(const std::string &&text) {
  marker_.text = text;
}

inline void ROSTextMarker::setScale(double z) {
  marker_.scale.z = z;  // Only scale "z" is used for text
}

ROSModelMarker::ROSModelMarker(ROSMarkerPublisher *ros_publisher,
                               const std::string &frame_id,
                               const std::string &model_path)
    : ROSMarker(ros_publisher, frame_id) {
  marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker_.mesh_resource = model_path;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the
  // line width
  marker_.scale.x = 1.;
  marker_.scale.y = 1.;
  marker_.scale.z = 1.;

  marker_.color.a = 1.0;

  setColor(0, 1, 0);

  marker_.pose.orientation.x = 0.0;
  marker_.pose.orientation.y = 0.0;
  marker_.pose.orientation.z = 0.0;
  marker_.pose.orientation.w = 1.0;
}

void ROSModelMarker::addPointMarker(const Eigen::Vector3d &p1) {
  geometry_msgs::Point result;
  result.x = p1(0);
  result.y = p1(1);
  result.z = p1(2);

  addPointMarker(result);
}

void ROSModelMarker::addPointMarker(const geometry_msgs::Point &p1) {
  // Request an ID
  marker_.id = ros_publisher_->getID();

  marker_.pose.position = p1;

  ros_publisher_->add(marker_);
}

void ROSModelMarker::addPointMarker(const geometry_msgs::Pose &pose) {
  geometry_msgs::Point result;
  result.x = pose.position.x;
  result.y = pose.position.y;
  result.z = pose.position.z;

  addPointMarker(result);
}

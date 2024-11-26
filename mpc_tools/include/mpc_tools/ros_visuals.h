#ifndef ROS_VISUALS_H
#define ROS_VISUALS_H

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <ros/ros.h>
#include <tf/tf.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class ROSMarker;
class ROSLine;
class ROSPointMarker;
class ROSMultiplePointMarker;
class ROSTextMarker;
class ROSModelMarker;

enum class Colormap
{
    VIRIDIS = 0,
    INFERNO,
    BRUNO
};

class ROSMarkerPublisher
{

public:
    ROSMarkerPublisher(ros::NodeHandle &nh, const char *topic_name, const std::string &frame_id, int max_size);
    ~ROSMarkerPublisher();

private:
    // One publisher
    ros::Publisher pub_;

    // One marker list
    visualization_msgs::MarkerArray marker_list_;
    visualization_msgs::MarkerArray prev_marker_list_;

    // a set of ros_markers
    std::vector<std::unique_ptr<ROSMarker>> ros_markers_;
    std::string topic_name_;

    std::string frame_id_;
    int id_, prev_id_;
    int max_size_;

public:
    void add(const visualization_msgs::Marker &marker);

    ROSLine &getNewLine();
    ROSPointMarker &getNewPointMarker(const std::string &marker_type);
    ROSMultiplePointMarker &getNewMultiplePointMarker(const std::string &marker_type); // const std::string &marker_type);
    ROSTextMarker &getNewTextMarker();                                                 // const std::string &marker_type);
    ROSModelMarker &getNewModelMarker(const std::string &model_path);                  // const std::string &marker_type);
    // ROSEllipse& getNewEllipse();

    void publish(bool keep_markers = false);

    int getID();
    int numberOfMarkers() { return id_; };

    std::string getFrameID() const;
};

class ROSMarker
{

public:
    ROSMarker(ROSMarkerPublisher *ros_publisher, const std::string &frame_id);

protected:
    static std::vector<double> VIRIDIS_, INFERNO_, BRUNO_;

    visualization_msgs::Marker marker_;

    ROSMarkerPublisher *ros_publisher_;

    geometry_msgs::Point vecToPoint(const Eigen::Vector3d &v);

    std::vector<double> &getColors(const Colormap &colormap);

    void getColorFromRange(double ratio, double &red, double &green, double &blue);
    void getColorFromRangeInt(int select, double &red, double &green, double &blue, const Colormap &colormap = Colormap::VIRIDIS);

public:
    void setColor(double r, double g, double b, double alpha = 1.0);
    // void setColor(double ratio); // Color rainbow wise

    void setColorInt(int select, double alpha = 1.0, const Colormap &&colormap = Colormap::VIRIDIS);            // Viridis (select our of range)
    void setColorInt(int select, int range, double alpha = 1.0, const Colormap &&colormap = Colormap::VIRIDIS); // Viridis (select our of range)

    void setColor(double ratio, double alpha = 1.0);
    void setScale(double x, double y, double z);
    void setScale(double x, double y);
    void setScale(double x);
    void setOrientation(double psi);
    void setOrientation(const geometry_msgs::Quaternion &msg);
    void setOrientation(const tf::Quaternion &q);
    void setLifetime(double lifetime);

    void setActionDelete();
    // void setIDRange(int range);
    // void resetID(); // Internal id count

    void stamp();
};

class ROSLine : public ROSMarker
{

public:
    ROSLine(ROSMarkerPublisher *ros_publisher, const std::string &frame_id);

    void addLine(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2);
    void addLine(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2);

    void addBrokenLine(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, double dist);
    void addBrokenLine(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2, double dist);

    void addCircularLine(const Eigen::Vector2d &center, double radius, double z, double n_points);
};

class ROSPointMarker : public ROSMarker
{

public:
    ROSPointMarker(ROSMarkerPublisher *ros_publisher, const std::string &frame_id, const std::string &marker_type);

    void addPointMarker(const Eigen::Vector3d &p1);
    void addPointMarker(const geometry_msgs::Point &p1);
    void addPointMarker(const geometry_msgs::Pose &pose);

    void setZ(double z);

private:
    std::string marker_type_;

    uint getMarkerType(const std::string &marker_type);
};

class ROSMultiplePointMarker : public ROSMarker
{

public:
    ROSMultiplePointMarker(ROSMarkerPublisher *ros_publisher, const std::string &frame_id, const std::string &type);

    void addPointMarker(const Eigen::Vector3d &p1);
    void addPointMarker(const geometry_msgs::Point &p1);
    void addPointMarker(const geometry_msgs::Pose &pose);

    void finishPoints();

private:
    uint getMultipleMarkerType(const std::string &marker_type);
};

class ROSTextMarker : public ROSMarker
{

public:
    ROSTextMarker(ROSMarkerPublisher *ros_publisher, const std::string &frame_id);

    void addPointMarker(const Eigen::Vector3d &p1);
    void addPointMarker(const geometry_msgs::Point &p1);
    void addPointMarker(const geometry_msgs::Pose &pose);

    void setZ(double z);
    void setText(const std::string &text);
    void setText(const std::string &&text);

    void setScale(double z);

    // void setScale(double x) override;

private:
};

class ROSModelMarker : public ROSMarker
{

public:
    ROSModelMarker(ROSMarkerPublisher *ros_publisher, const std::string &frame_id, const std::string &model_path);

    void addPointMarker(const Eigen::Vector3d &p1);
    void addPointMarker(const geometry_msgs::Point &p1);
    void addPointMarker(const geometry_msgs::Pose &pose);

private:
};

#endif
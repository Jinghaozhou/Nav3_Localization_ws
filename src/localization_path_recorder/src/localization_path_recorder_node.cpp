#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class LocalizationPathRecorder : public rclcpp::Node {
 public:
  LocalizationPathRecorder() : Node("path_recorder") {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    tracked_frame_ = declare_parameter<std::string>("tracked_frame", "base_link");
    const double sample_hz = declare_parameter<double>("sample_hz", 5.0);
    min_distance_m_ = declare_parameter<double>("min_distance_m", 0.05);
    max_tf_age_sec_ = declare_parameter<double>("max_tf_age_sec", 1.0);
    max_poses_ = declare_parameter<int>("max_poses", 10000);
    if (map_frame_.empty() || tracked_frame_.empty() || map_frame_ == tracked_frame_ ||
        !std::isfinite(sample_hz) || sample_hz <= 0.0 ||
        !std::isfinite(min_distance_m_) || min_distance_m_ < 0.0 ||
        !std::isfinite(max_tf_age_sec_) || max_tf_age_sec_ <= 0.0 ||
        max_poses_ <= 0) {
      throw std::invalid_argument("invalid path recorder parameters");
    }

    path_.header.frame_id = map_frame_;
    rclcpp::QoS qos(1);
    qos.reliable().transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("map_path", qos);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("map_pose", qos);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / sample_hz),
                               [this]() { samplePose(); });
    RCLCPP_INFO(get_logger(), "Waiting for %s <- %s TF to record /localization/map_path",
                map_frame_.c_str(), tracked_frame_.c_str());
  }

 private:
  void samplePose() {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
          map_frame_, tracked_frame_, tf2::TimePointZero,
          tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Waiting for %s <- %s TF: %s",
                           map_frame_.c_str(), tracked_frame_.c_str(), error.what());
      return;
    }

    const rclcpp::Time stamp(transform.header.stamp);
    const double age = (get_clock()->now() - stamp).seconds();
    if (stamp.nanoseconds() == 0 || age > max_tf_age_sec_ || age < -1.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Ignoring stale or future-dated %s <- %s TF (age %.2f s)",
                           map_frame_.c_str(), tracked_frame_.c_str(), age);
      return;
    }
    if (has_last_stamp_ && stamp <= last_stamp_) {
      if ((last_stamp_ - stamp).seconds() > 1.0) {
        path_.poses.clear();
        RCLCPP_INFO(get_logger(), "TF time moved backwards; starting a new path");
      } else {
        return;
      }
    }
    const auto &position = transform.transform.translation;
    const auto &orientation = transform.transform.rotation;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
        !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
        !std::isfinite(orientation.w)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Ignoring non-finite map pose");
      return;
    }
    has_last_stamp_ = true;
    last_stamp_ = stamp;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = transform.header.stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.position.x = position.x;
    pose.pose.position.y = position.y;
    pose.pose.position.z = position.z;
    pose.pose.orientation = orientation;
    pose_pub_->publish(pose);

    if (!path_.poses.empty()) {
      const auto &previous = path_.poses.back().pose.position;
      const double dx = position.x - previous.x;
      const double dy = position.y - previous.y;
      const double dz = position.z - previous.z;
      if (dx * dx + dy * dy + dz * dz < min_distance_m_ * min_distance_m_) {
        return;
      }
    }

    path_.poses.push_back(pose);
    if (path_.poses.size() > static_cast<std::size_t>(max_poses_)) {
      path_.poses.erase(path_.poses.begin());
    }
    path_.header.stamp = transform.header.stamp;
    path_pub_->publish(path_);
    if (path_.poses.size() == 1) {
      RCLCPP_INFO(get_logger(), "Started map path at (%.2f, %.2f, %.2f)",
                  position.x, position.y, position.z);
    }
  }

  std::string map_frame_;
  std::string tracked_frame_;
  double min_distance_m_ = 0.05;
  double max_tf_age_sec_ = 1.0;
  int max_poses_ = 10000;
  bool has_last_stamp_ = false;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Path path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationPathRecorder>());
  rclcpp::shutdown();
  return 0;
}

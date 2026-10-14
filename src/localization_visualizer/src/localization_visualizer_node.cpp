#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

class LocalizationVisualizer : public rclcpp::Node {
 public:
  LocalizationVisualizer() : Node("localization_visualizer") {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    comparison_frame_ = declare_parameter<std::string>("comparison_frame", "localization_comparison");
    comparison_gap_m_ = declare_parameter<double>("comparison_gap_m", 10.0);
    const auto map_pcd = declare_parameter<std::string>("map_pcd", "");
    const auto local_map_topic = declare_parameter<std::string>("local_map_topic", "/local_map");
    const auto local_map_reliability =
        declare_parameter<std::string>("local_map_reliability", "best_effort");
    const double map_voxel = declare_parameter<double>("map_voxel_resolution", 0.2);
    local_voxel_resolution_ = declare_parameter<double>("local_voxel_resolution", 0.2);
    const double local_publish_hz = declare_parameter<double>("local_publish_hz", 1.0);
    if (map_frame_.empty() || odom_frame_.empty() || comparison_frame_.empty() ||
        map_frame_ == odom_frame_ || comparison_frame_ == map_frame_ ||
        comparison_frame_ == odom_frame_ || !std::isfinite(comparison_gap_m_) ||
        comparison_gap_m_ <= 0.0 ||
        local_map_topic.empty() || map_voxel <= 0.0 || local_voxel_resolution_ <= 0.0 ||
        !std::isfinite(local_publish_hz) || local_publish_hz <= 0.0 ||
        (local_map_reliability != "best_effort" &&
         local_map_reliability != "reliable")) {
      throw std::invalid_argument("invalid visualization parameters");
    }

    if (!map_pcd.empty()) loadMap(map_pcd, map_voxel);

    rclcpp::QoS output_qos(1);
    output_qos.reliable().transient_local();
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("global_map", output_qos);
    local_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("local_map_odom", output_qos);
    aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "aligned_local_map", output_qos);
    comparison_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "comparison/global_map", output_qos);
    comparison_local_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "comparison/local_map", output_qos);
    comparison_aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "comparison/aligned_local_map", output_qos);
    comparison_labels_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "comparison/labels", output_qos);

    rclcpp::QoS input_qos = rclcpp::SensorDataQoS();
    if (local_map_reliability == "reliable") {
      input_qos.reliable();
    }
    local_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        local_map_topic, input_qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(input_mutex_);
          latest_local_map_ = std::move(message);
        });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    if (!map_cloud_.empty()) {
      map_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
        publishMap();
        map_timer_->cancel();
      });
    }
    local_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / local_publish_hz),
        [this]() { publishLocalMaps(); });
  }

 private:
  void loadMap(const std::string &path, double voxel_resolution) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr full(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(path, *full) < 0 || full->empty()) {
      throw std::runtime_error("failed to load nonempty global map PCD: " + path);
    }
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(voxel_resolution, voxel_resolution, voxel_resolution);
    filter.setInputCloud(full);
    filter.filter(map_cloud_);
    if (map_cloud_.empty()) {
      throw std::runtime_error("global map is empty after visualization downsampling: " + path);
    }
    pcl::PointXYZ min_point, max_point;
    pcl::getMinMax3D(map_cloud_, min_point, max_point);
    map_max_x_ = max_point.x;
    map_center_x_ = (min_point.x + max_point.x) / 2.0;
    map_center_y_ = (min_point.y + max_point.y) / 2.0;
    map_label_z_ = max_point.z + 2.0;
    RCLCPP_INFO(get_logger(), "Visualization map: %zu -> %zu points (%.3f m)",
                full->size(), map_cloud_.size(), voxel_resolution);
  }

  void publishMap() {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(map_cloud_, message);
    message.header.frame_id = map_frame_;
    message.header.stamp = get_clock()->now();
    map_pub_->publish(message);
    message.header.frame_id = comparison_frame_;
    comparison_map_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "Published global map once on /localization_visualization/global_map");
  }

  void publishLocalMaps() {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr snapshot;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      snapshot = latest_local_map_;
    }
    if (!snapshot) return;
    if (snapshot->header.frame_id != odom_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Local map frame is '%s', expected '%s'",
                           snapshot->header.frame_id.c_str(), odom_frame_.c_str());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*snapshot, *source);
    if (source->empty()) return;
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(local_voxel_resolution_, local_voxel_resolution_,
                       local_voxel_resolution_);
    filter.setInputCloud(source);
    pcl::PointCloud<pcl::PointXYZ> sampled;
    filter.filter(sampled);
    if (sampled.empty()) return;

    sensor_msgs::msg::PointCloud2 raw_message;
    pcl::toROSMsg(sampled, raw_message);
    raw_message.header = snapshot->header;
    local_pub_->publish(raw_message);

    // This shift belongs only to the comparison canvas; it is not a localization transform.
    pcl::PointXYZ local_min, local_max;
    pcl::getMinMax3D(sampled, local_min, local_max);
    const float display_shift_x = map_cloud_.empty()
        ? 0.0f : static_cast<float>(map_max_x_ + comparison_gap_m_ - local_min.x);
    const float display_shift_y = map_cloud_.empty()
        ? 0.0f : static_cast<float>(map_center_y_ - (local_min.y + local_max.y) / 2.0);
    pcl::PointCloud<pcl::PointXYZ> comparison_local = sampled;
    for (auto &point : comparison_local.points) {
      point.x += display_shift_x;
      point.y += display_shift_y;
    }
    sensor_msgs::msg::PointCloud2 comparison_message;
    pcl::toROSMsg(comparison_local, comparison_message);
    comparison_message.header.stamp = get_clock()->now();
    comparison_message.header.frame_id = comparison_frame_;
    comparison_local_pub_->publish(comparison_message);
    publishComparisonLabels(local_min, local_max, display_shift_x, display_shift_y);

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
          map_frame_, odom_frame_, tf2::TimePointZero,
          tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Raw local map published; waiting for %s -> %s TF: %s",
                           map_frame_.c_str(), odom_frame_.c_str(), error.what());
      return;
    }

    const auto &t = transform.transform.translation;
    const auto &r = transform.transform.rotation;
    const Eigen::Quaternionf q(r.w, r.x, r.y, r.z);
    Eigen::Matrix4f map_T_odom = Eigen::Matrix4f::Identity();
    map_T_odom.block<3, 3>(0, 0) = q.toRotationMatrix();
    map_T_odom.block<3, 1>(0, 3) = Eigen::Vector3f(t.x, t.y, t.z);
    pcl::PointCloud<pcl::PointXYZ> aligned;
    pcl::transformPointCloud(sampled, aligned, map_T_odom);

    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(aligned, message);
    message.header.frame_id = map_frame_;
    message.header.stamp = get_clock()->now();
    aligned_pub_->publish(message);
    message.header.frame_id = comparison_frame_;
    comparison_aligned_pub_->publish(message);
  }

  void publishComparisonLabels(const pcl::PointXYZ &local_min,
                               const pcl::PointXYZ &local_max,
                               float display_shift_x, float display_shift_y) {
    visualization_msgs::msg::MarkerArray labels;
    const auto make_label = [this](int id, const std::string &caption,
                                   double x, double y, double z,
                                   float red, float green, float blue) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = comparison_frame_;
      marker.header.stamp = get_clock()->now();
      marker.ns = "map_comparison";
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = z;
      marker.pose.orientation.w = 1.0;
      marker.scale.z = 2.0;
      marker.color.r = red;
      marker.color.g = green;
      marker.color.b = blue;
      marker.color.a = 1.0;
      marker.text = caption;
      return marker;
    };
    if (!map_cloud_.empty()) {
      labels.markers.push_back(make_label(0, "GLOBAL MAP / aligned result",
          map_center_x_, map_center_y_, map_label_z_, 0.8f, 0.8f, 0.8f));
    }
    labels.markers.push_back(make_label(1, "LOCAL MAP / raw odom (display offset)",
        (local_min.x + local_max.x) / 2.0 + display_shift_x,
        (local_min.y + local_max.y) / 2.0 + display_shift_y, local_max.z + 2.0,
        1.0f, 0.5f, 0.2f));
    comparison_labels_pub_->publish(labels);
  }

  std::string map_frame_;
  std::string odom_frame_;
  std::string comparison_frame_;
  double comparison_gap_m_ = 10.0;
  double map_max_x_ = 0.0;
  double map_center_x_ = 0.0;
  double map_center_y_ = 0.0;
  double map_label_z_ = 0.0;
  double local_voxel_resolution_ = 0.2;
  pcl::PointCloud<pcl::PointXYZ> map_cloud_;
  std::mutex input_mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_local_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr comparison_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr comparison_local_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr comparison_aligned_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr comparison_labels_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_sub_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr local_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationVisualizer>());
  rclcpp::shutdown();
  return 0;
}

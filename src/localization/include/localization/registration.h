#pragma once

#include <cstddef>
#include <memory>
#include <limits>

#include <Eigen/Core>
#include <kiss_matcher/KISSMatcher.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/logger.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>

namespace kiss_matcher {

using LocalizationPoint = pcl::PointXYZI;

struct LocalizationConfig {
  bool verbose = false;
  bool enable_coarse = true; // Standalone historical tools only; continuous node sets false.
  double voxel_resolution = 0.1;
  std::size_t min_inliers = 20;
  int num_threads = 4;
  int correspondence_randomness = 20;
  int max_num_iter = 64;
  double scale_factor_for_corr_dist = 3.0;
  double overlap_threshold = 70.0;
};

struct LocalizationResult {
  bool valid = false;
  bool converged = false;
  std::size_t num_final_inliers = 0;
  double overlap_percent = 0.0;
  std::size_t fine_inliers = 0;
  double inlier_rmse = std::numeric_limits<double>::infinity();
  // Diagnostics only: do not use counts from a stage which did not run.
  bool coarse_ran = false;
  bool fine_ran = false;
  std::size_t source_features = 0;
  std::size_t target_features = 0;
  std::size_t initial_matches = 0;
  std::size_t pruned_matches = 0;
  std::size_t rotation_inliers = 0;
  std::size_t target_points_used = 0;
  Eigen::Matrix4d target_T_source = Eigen::Matrix4d::Identity();
};

class LocalizationRegistration {
 public:
  explicit LocalizationRegistration(const LocalizationConfig &config, const rclcpp::Logger &logger);

  void setVisualizationClouds(const pcl::PointCloud<LocalizationPoint> &source,
                              const pcl::PointCloud<LocalizationPoint> &target);
  LocalizationResult align(const pcl::PointCloud<LocalizationPoint> &source,
                           const pcl::PointCloud<LocalizationPoint> &target);

  // Prior is T_map_odom; source remains in odom. Crop map around predicted source.
  LocalizationResult alignWithPrior(const pcl::PointCloud<LocalizationPoint> &source,
      const pcl::PointCloud<LocalizationPoint> &target,
      const Eigen::Matrix4d &prior, double margin);

  pcl::PointCloud<LocalizationPoint> getSourceCloud() const;
  pcl::PointCloud<LocalizationPoint> getTargetCloud() const;
  pcl::PointCloud<LocalizationPoint> getCoarseAlignedCloud() const;
  pcl::PointCloud<LocalizationPoint> getFinalAlignedCloud() const;

 private:
  LocalizationResult refine(const pcl::PointCloud<LocalizationPoint> &source,
                            const pcl::PointCloud<LocalizationPoint> &target);

  LocalizationConfig config_;
  rclcpp::Logger logger_;
  std::shared_ptr<KISSMatcher> coarse_matcher_;
  std::shared_ptr<small_gicp::RegistrationPCL<LocalizationPoint, LocalizationPoint>> fine_matcher_;
  pcl::PointCloud<LocalizationPoint>::Ptr source_vis_;
  pcl::PointCloud<LocalizationPoint>::Ptr target_vis_;
  pcl::PointCloud<LocalizationPoint>::Ptr coarse_aligned_;
  pcl::PointCloud<LocalizationPoint>::Ptr fine_aligned_;
};

}  // namespace kiss_matcher

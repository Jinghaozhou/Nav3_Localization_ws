#include "localization/registration.h"

#include <cmath>
#include <vector>

#include <pcl/common/transforms.h>
#include <rclcpp/rclcpp.hpp>

namespace kiss_matcher {
namespace {

std::vector<Eigen::Vector3f> toPoints(const pcl::PointCloud<LocalizationPoint> &cloud) {
  std::vector<Eigen::Vector3f> points;
  points.reserve(cloud.size());
  for (const auto &point : cloud.points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      points.emplace_back(point.x, point.y, point.z);
    }
  }
  return points;
}

}  // namespace

LocalizationRegistration::LocalizationRegistration(const LocalizationConfig &config,
                                                   const rclcpp::Logger &logger)
    : config_(config), logger_(logger),
      coarse_matcher_(std::make_shared<KISSMatcher>(
          KISSMatcherConfig(config.voxel_resolution, false))),
      fine_matcher_(std::make_shared<small_gicp::RegistrationPCL<LocalizationPoint,
                                                                  LocalizationPoint>>()),
      source_vis_(new pcl::PointCloud<LocalizationPoint>()),
      target_vis_(new pcl::PointCloud<LocalizationPoint>()),
      coarse_aligned_(new pcl::PointCloud<LocalizationPoint>()),
      fine_aligned_(new pcl::PointCloud<LocalizationPoint>()) {
  fine_matcher_->setNumThreads(config.num_threads);
  fine_matcher_->setCorrespondenceRandomness(config.correspondence_randomness);
  fine_matcher_->setMaxCorrespondenceDistance(
      config.voxel_resolution * config.scale_factor_for_corr_dist);
  fine_matcher_->setVoxelResolution(config.voxel_resolution);
  fine_matcher_->setRegistrationType("VGICP");
}

void LocalizationRegistration::setVisualizationClouds(
    const pcl::PointCloud<LocalizationPoint> &source,
    const pcl::PointCloud<LocalizationPoint> &target) {
  *source_vis_ = source;
  *target_vis_ = target;
}

LocalizationResult LocalizationRegistration::refine(
    const pcl::PointCloud<LocalizationPoint> &source,
    const pcl::PointCloud<LocalizationPoint> &target) {
  LocalizationResult result;
  fine_aligned_->clear();
  auto source_ptr = pcl::PointCloud<LocalizationPoint>::Ptr(
      new pcl::PointCloud<LocalizationPoint>(source));
  auto target_ptr = pcl::PointCloud<LocalizationPoint>::Ptr(
      new pcl::PointCloud<LocalizationPoint>(target));
  fine_matcher_->setInputTarget(target_ptr);
  fine_matcher_->setInputSource(source_ptr);
  fine_matcher_->align(*fine_aligned_);

  const auto &fine_result = fine_matcher_->getRegistrationResult();
  result.overlap_percent =
      static_cast<double>(fine_result.num_inliers) / source_ptr->size() * 100.0;
  result.target_T_source = fine_matcher_->getFinalTransformation().cast<double>();
  if (result.overlap_percent > config_.overlap_threshold) {
    result.valid = true;
    result.converged = true;
  }
  if (config_.verbose) {
    if (result.valid) {
      RCLCPP_INFO(logger_, "VGICP accepted: overlap=%.2f%% (%zu/%zu points; required > %.2f%%)",
                  result.overlap_percent, fine_result.num_inliers, source_ptr->size(),
                  config_.overlap_threshold);
    } else {
      RCLCPP_WARN(logger_, "VGICP rejected: overlap=%.2f%% (%zu/%zu points; required > %.2f%%)",
                  result.overlap_percent, fine_result.num_inliers, source_ptr->size(),
                  config_.overlap_threshold);
    }
  }
  return result;
}

LocalizationResult LocalizationRegistration::align(
    const pcl::PointCloud<LocalizationPoint> &source,
    const pcl::PointCloud<LocalizationPoint> &target) {
  LocalizationResult result;
  coarse_aligned_->clear();
  fine_aligned_->clear();
  const auto source_points = toPoints(source);
  const auto target_points = toPoints(target);
  const auto solution = coarse_matcher_->estimate(source_points, target_points);
  const auto initial_matches = coarse_matcher_->getInitialCorrespondences().size();
  const auto pruned_matches = coarse_matcher_->getFinalCorrespondences().size();
  // KISS does not run the solver when fewer than two pairs survive pruning.
  const auto rotation_inliers =
      pruned_matches >= 2 ? coarse_matcher_->getNumRotationInliers() : 0;
  result.num_final_inliers =
      pruned_matches >= 2 ? coarse_matcher_->getNumFinalInliers() : 0;

  Eigen::Matrix4d coarse = Eigen::Matrix4d::Identity();
  coarse.block<3, 3>(0, 0) = solution.rotation.cast<double>();
  coarse.topRightCorner(3, 1) = solution.translation.cast<double>();
  if (!source.empty()) {
    pcl::transformPointCloud(source, *coarse_aligned_, coarse);
  }
  if (config_.verbose) {
    const auto [source_features, target_features] =
        coarse_matcher_->getKeypointsFromFasterPFH();
    RCLCPP_INFO(logger_,
                "KISS features: local=%zu/%zu points, global=%zu/%zu points",
                source_features.size(), source_points.size(),
                target_features.size(), target_points.size());
    RCLCPP_INFO(logger_,
                "KISS matches: initial=%zu, after_pruning=%zu, "
                "rotation_inliers=%zu, translation_inliers=%zu",
                initial_matches, pruned_matches, rotation_inliers,
                result.num_final_inliers);
  }

  if (pruned_matches < 2 || !solution.valid ||
      result.num_final_inliers < config_.min_inliers) {
    if (config_.verbose) {
      RCLCPP_WARN(logger_,
                  "KISS rejected: solution_valid=%s, translation_inliers=%zu "
                  "(required >= %zu); VGICP skipped",
                  solution.valid ? "true" : "false", result.num_final_inliers,
                  config_.min_inliers);
    }
    return result;
  }

  const auto inliers = result.num_final_inliers;
  result = refine(*coarse_aligned_, target);
  result.num_final_inliers = inliers;
  result.target_T_source = result.target_T_source * coarse;
  return result;
}

pcl::PointCloud<LocalizationPoint> LocalizationRegistration::getSourceCloud() const {
  return *source_vis_;
}
pcl::PointCloud<LocalizationPoint> LocalizationRegistration::getTargetCloud() const {
  return *target_vis_;
}
pcl::PointCloud<LocalizationPoint> LocalizationRegistration::getCoarseAlignedCloud() const {
  return *coarse_aligned_;
}
pcl::PointCloud<LocalizationPoint> LocalizationRegistration::getFinalAlignedCloud() const {
  return *fine_aligned_;
}

}  // namespace kiss_matcher

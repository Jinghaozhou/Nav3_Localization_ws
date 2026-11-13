#include "localization/registration.h"

#include <cmath>
#include <limits>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
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
      coarse_matcher_(config.enable_coarse ? std::make_shared<KISSMatcher>(
          KISSMatcherConfig(config.voxel_resolution, false)) : nullptr),
      fine_matcher_(std::make_shared<small_gicp::RegistrationPCL<LocalizationPoint,
                                                                  LocalizationPoint>>()),
      source_vis_(new pcl::PointCloud<LocalizationPoint>()),
      target_vis_(new pcl::PointCloud<LocalizationPoint>()),
      coarse_aligned_(new pcl::PointCloud<LocalizationPoint>()),
      fine_aligned_(new pcl::PointCloud<LocalizationPoint>()) {
  fine_matcher_->setMaximumIterations(config.max_num_iter);
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
  result.fine_ran = true;
  result.target_points_used = target.size();
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
  result.fine_inliers = fine_result.num_inliers;
  result.converged = fine_result.converged;
  // Metric Euclidean residual; small_gicp's error is covariance weighted.
  if (!target_ptr->empty() && result.target_T_source.allFinite()) {
    pcl::KdTreeFLANN<LocalizationPoint> tree;
    tree.setInputCloud(target_ptr);
    const double distance = config_.voxel_resolution * config_.scale_factor_for_corr_dist;
    double sum = 0.0; std::size_t count = 0;
    std::vector<int> indices(1); std::vector<float> distances(1);
    for (const auto &point : *fine_aligned_) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
      if (tree.nearestKSearch(point, 1, indices, distances) > 0 && distances[0] <= distance*distance) {
        sum += distances[0]; ++count;
      }
    }
    if (count) result.inlier_rmse = std::sqrt(sum / count);
  }
  result.valid = result.overlap_percent > config_.overlap_threshold &&
      result.converged && result.target_T_source.allFinite();
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
  if(!coarse_matcher_) throw std::logic_error("Coarse registration disabled for continuous localization");
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
  const auto [source_features, target_features] =
      coarse_matcher_->getKeypointsFromFasterPFH();
  result.coarse_ran = true;
  result.source_features = source_features.size();
  result.target_features = target_features.size();
  result.initial_matches = initial_matches;
  result.pruned_matches = pruned_matches;
  result.rotation_inliers = rotation_inliers;
  result.target_points_used = target.size();

  Eigen::Matrix4d coarse = Eigen::Matrix4d::Identity();
  coarse.block<3, 3>(0, 0) = solution.rotation.cast<double>();
  coarse.topRightCorner(3, 1) = solution.translation.cast<double>();
  if (!source.empty()) {
    pcl::transformPointCloud(source, *coarse_aligned_, coarse);
  }
  if (config_.verbose) {
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
  const auto coarse_diagnostics = result;
  result = refine(*coarse_aligned_, target);
  result.coarse_ran = true;
  result.source_features = coarse_diagnostics.source_features;
  result.target_features = coarse_diagnostics.target_features;
  result.initial_matches = initial_matches;
  result.pruned_matches = pruned_matches;
  result.rotation_inliers = rotation_inliers;
  result.num_final_inliers = inliers;
  result.target_T_source = result.target_T_source * coarse;
  return result;
}

LocalizationResult LocalizationRegistration::alignWithPrior(
    const pcl::PointCloud<LocalizationPoint> &source,
    const pcl::PointCloud<LocalizationPoint> &target,
    const Eigen::Matrix4d &prior, double margin) {
  if (source.empty() || target.empty() || !prior.allFinite()) return {};
  pcl::PointCloud<LocalizationPoint> predicted, cropped;
  pcl::transformPointCloud(source, predicted, prior);
  Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d hi = -lo;
  for (const auto &p : predicted) {
    const Eigen::Vector3d v(p.x, p.y, p.z);
    if (!v.allFinite()) continue;
    lo = lo.cwiseMin(v); hi = hi.cwiseMax(v);
  }
  lo.array() -= margin; hi.array() += margin;
  for (const auto &p : target) {
    const Eigen::Vector3d v(p.x, p.y, p.z);
    if ((v.array() >= lo.array()).all() && (v.array() <= hi.array()).all())
      cropped.push_back(p);
  }
  if (cropped.size() < static_cast<std::size_t>(config_.correspondence_randomness) ||
      predicted.size() < static_cast<std::size_t>(config_.correspondence_randomness)) return {};
  auto result = refine(predicted, cropped);
  result.valid = result.valid && fine_matcher_->getRegistrationResult().converged &&
                 result.target_T_source.allFinite();
  result.converged = fine_matcher_->getRegistrationResult().converged;
  result.target_T_source = result.target_T_source * prior;
  // No KISS coarse stage in prior mode: report zero coarse inliers, never invent them.
  result.num_final_inliers = 0;
  RCLCPP_INFO(logger_, "Prior VGICP: target=%zu/%zu overlap=%.2f%% converged=%s accepted=%s",
      cropped.size(), target.size(), result.overlap_percent,
      result.converged ? "true" : "false", result.valid ? "true" : "false");
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

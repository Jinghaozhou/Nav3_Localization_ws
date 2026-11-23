#pragma once
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include "localization/registration.h"

namespace kiss_matcher {
struct InitializerConfig {
  int max_candidates = 64, confirm_windows = 2;
  double crop_margin = 5.0, max_translation_correction = 1.0;
  double max_rotation_correction_deg = 30.0;
  double cluster_translation = 0.5, cluster_rotation_deg = 10.0;
  double ambiguity_gap_percent = 5.0, confirm_min_dt = 1.0, confirm_timeout = 30.0;
  double consistency_translation = 0.5, consistency_rotation_deg = 5.0;
  double max_inlier_rmse = 0.2;
  int min_fine_inliers = 20;
};
struct InitializationCandidate {
  std::string id, stage, reason;
  LocalizationResult registration;
  Eigen::Matrix4d map_T_base = Eigen::Matrix4d::Identity();
  bool eligible = false;
  int cluster = -1;
};
struct InitializationDecision {
  bool ready = false;
  std::string reason;
  int selected = -1, clusters = 0, confirmations = 0;
};
class RegionInitializer {
 public:
  explicit RegionInitializer(const InitializerConfig &config);
  void load(const std::string &file, const std::string &map_frame, const std::string &base_frame);
  bool hasRegion(int id) const { return regions_.count(id) != 0; }
  void reset();
  // Expired registration invalidates confirmation, not the starting-location prior.
  void resetConfirmation();
  // A temporary missing query is not a contradictory pose. Still expire by time.
  int waitForQuery(std::int64_t now);
  // Freeze ALL seed map->odom guesses at the first snapshot. Subsequent snapshots
  // use those same transforms, i.e. propagate map->base with relative odometry.
  void begin(int region, std::int64_t stamp, const Eigen::Matrix4d &odom_T_base);
  const std::vector<std::pair<std::string, Eigen::Matrix4d>> &guesses() const { return guesses_; }
  InitializationCandidate evaluate(const std::string &id, const std::string &stage,
      const LocalizationResult &result, const Eigen::Matrix4d &odom_T_base,
      const std::optional<Eigen::Matrix4d> &guess, bool apply_region_bounds=true) const;
  InitializationDecision decide(std::vector<InitializationCandidate> &candidates,
      std::int64_t stamp, const Eigen::Matrix4d &odom_T_base);
  std::int64_t referenceStamp() const { return reference_stamp_; }
  const InitializerConfig &config() const { return config_; }
 private:
  struct Region {
    bool position_bounds_at_reference = false;
    std::optional<Eigen::Vector3d> position_min, position_max, rpy_min, rpy_max;
    std::vector<std::pair<std::string, Eigen::Matrix4d>> seeds;
  };
  InitializerConfig config_;
  std::map<int, Region> regions_;
  int active_region_ = -2, confirmations_ = 0;
  std::int64_t reference_stamp_ = 0, last_seen_stamp_ = 0, confirmation_stamp_ = 0;
  std::optional<Eigen::Matrix4d> previous_;
  Eigen::Matrix4d reference_odom_ = Eigen::Matrix4d::Identity();
  std::vector<std::pair<std::string, Eigen::Matrix4d>> guesses_;
};
double initializationRotationDegrees(const Eigen::Matrix3d &rotation);
}  // namespace kiss_matcher

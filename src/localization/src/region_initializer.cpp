#include "localization/region_initializer.h"
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace kiss_matcher {
namespace {
constexpr double rad = 3.14159265358979323846 / 180.0;
Eigen::Vector3d vector3(const YAML::Node &n) {
  if (!n.IsSequence() || n.size() != 3) throw std::invalid_argument("expected finite XYZ triple");
  Eigen::Vector3d v(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
  if (!v.allFinite()) throw std::invalid_argument("nonfinite profile range/position");
  return v;
}
bool rigid(const Eigen::Matrix4d &t) {
  const Eigen::Matrix3d r = t.block<3,3>(0,0);
  return t.allFinite() && (t.row(3) - Eigen::RowVector4d(0,0,0,1)).norm() < 1e-5 &&
      (r.transpose()*r - Eigen::Matrix3d::Identity()).norm() < 1e-4 && std::abs(r.determinant()-1) < 1e-4;
}
bool near(const Eigen::Matrix4d &a, const Eigen::Matrix4d &b, double meters, double degrees) {
  return (a.block<3,1>(0,3)-b.block<3,1>(0,3)).norm() <= meters &&
      initializationRotationDegrees(a.block<3,3>(0,0).transpose()*b.block<3,3>(0,0)) <= degrees;
}
}
double initializationRotationDegrees(const Eigen::Matrix3d &r) {
  return std::acos(std::clamp((r.trace()-1.0)/2.0,-1.0,1.0))/rad;
}
RegionInitializer::RegionInitializer(const InitializerConfig &c) : config_(c) {
  const double values[] = {c.crop_margin,c.max_translation_correction,c.max_rotation_correction_deg,
      c.cluster_translation,c.cluster_rotation_deg,c.ambiguity_gap_percent,c.confirm_min_dt,
      c.confirm_timeout,c.consistency_translation,c.consistency_rotation_deg,c.max_inlier_rmse};
  for (double v : values) if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("initializer thresholds must be finite positive");
  if (c.max_candidates < 1 || c.confirm_windows < 2 || c.min_fine_inliers < 1 || c.confirm_timeout <= c.confirm_min_dt)
    throw std::invalid_argument("invalid initializer candidate/confirmation limits");
}
void RegionInitializer::load(const std::string &file, const std::string &map_frame, const std::string &base_frame) {
  auto root = YAML::LoadFile(file);
  if (root["schema_version"].as<int>() != 1 || root["map_frame"].as<std::string>() != map_frame ||
      root["base_frame"].as<std::string>() != base_frame || !root["regions"].IsSequence())
    throw std::invalid_argument("initializer profile schema/frame mismatch");
  std::map<int,Region> parsed;
  for (const auto &entry : root["regions"]) {
    const int id = entry["region_id"].as<int>();
    if (id < 0 || parsed.count(id)) throw std::invalid_argument("duplicate/negative profile region");
    Region region;
    auto bounds = [&](const char *low, const char *high, auto &lo, auto &hi) {
      if (bool(entry[low]) != bool(entry[high])) throw std::invalid_argument("range bounds must be paired");
      if (entry[low]) {
        lo = vector3(entry[low]); hi = vector3(entry[high]);
        if ((lo->array() > hi->array()).any()) throw std::invalid_argument("range min exceeds max; split wraparound yaw ranges");
      }
    };
    bounds("position_min","position_max",region.position_min,region.position_max);
    const auto bounds_mode = entry["position_bounds_mode"] ? entry["position_bounds_mode"].as<std::string>() : "current";
    if (bounds_mode != "current" && bounds_mode != "initial_reference")
      throw std::invalid_argument("position_bounds_mode must be current or initial_reference");
    region.position_bounds_at_reference = bounds_mode == "initial_reference";
    bounds("rpy_min_deg","rpy_max_deg",region.rpy_min,region.rpy_max);
    if (region.rpy_min && ((region.rpy_min->array() < -180).any() || (region.rpy_max->array() > 180).any()))
      throw std::invalid_argument("RPY bounds must lie in [-180,180]");
    if (!entry["seeds"].IsSequence() || entry["seeds"].size() == 0 ||
        entry["seeds"].size() > static_cast<std::size_t>(config_.max_candidates))
      throw std::invalid_argument("empty/excessive region seeds");
    std::set<std::string> names;
    for (const auto &seed : entry["seeds"]) {
      const auto name = seed["id"].as<std::string>();
      if (name.empty() || !names.insert(name).second) throw std::invalid_argument("empty/duplicate seed ID");
      auto qn = seed["orientation_xyzw"];
      if (!qn.IsSequence() || qn.size()!=4) throw std::invalid_argument("orientation_xyzw requires four values");
      Eigen::Quaterniond q(qn[3].as<double>(),qn[0].as<double>(),qn[1].as<double>(),qn[2].as<double>());
      if (!q.coeffs().allFinite() || q.norm()<1e-8 || std::abs(q.norm()-1.0)>0.01)
        throw std::invalid_argument("seed quaternion must be finite and approximately unit");
      Eigen::Matrix4d pose=Eigen::Matrix4d::Identity();
      pose.block<3,3>(0,0)=q.normalized().toRotationMatrix(); pose.block<3,1>(0,3)=vector3(seed["position"]);
      region.seeds.emplace_back(name,pose);
    }
    parsed.emplace(id,std::move(region));
  }
  if (parsed.empty()) throw std::invalid_argument("empty initializer profile");
  regions_=std::move(parsed); reset();
}
void RegionInitializer::reset() {
  active_region_=-2; reference_stamp_=last_seen_stamp_=confirmation_stamp_=0;
  confirmations_=0; previous_.reset(); guesses_.clear(); reference_odom_.setIdentity();
}
void RegionInitializer::resetConfirmation() {
  confirmations_=0; confirmation_stamp_=0; previous_.reset();
}
int RegionInitializer::waitForQuery(std::int64_t now) {
  if(previous_ && (now<confirmation_stamp_ ||
      (now-confirmation_stamp_)/1e9>config_.confirm_timeout)) resetConfirmation();
  return confirmations_;
}
void RegionInitializer::begin(int region, std::int64_t stamp, const Eigen::Matrix4d &odom_T_base) {
  if (stamp<=0 || !rigid(odom_T_base)) throw std::invalid_argument("invalid snapshot timestamp/odom transform");
  if (active_region_!=region || (last_seen_stamp_ && stamp<last_seen_stamp_)) reset();
  else if (last_seen_stamp_ && (stamp-last_seen_stamp_)/1e9 > config_.confirm_timeout) {
    // A confirmation timeout does not move the physical starting-location prior.
    if (hasRegion(region) && regions_.at(region).position_bounds_at_reference) resetConfirmation();
    else reset();  // Preserve legacy current-position profiles.
  }
  if (active_region_==-2) {
    active_region_=region; reference_stamp_=stamp; reference_odom_=odom_T_base;
    if (hasRegion(region)) for (const auto &[id,pose] : regions_.at(region).seeds)
      guesses_.emplace_back(id,pose*odom_T_base.inverse());
  }
  last_seen_stamp_=stamp;
}
InitializationCandidate RegionInitializer::evaluate(const std::string &id,const std::string &stage,
    const LocalizationResult &result,const Eigen::Matrix4d &odom_T_base,
    const std::optional<Eigen::Matrix4d> &guess, bool apply_region_bounds) const {
  InitializationCandidate c; c.id=id; c.stage=stage; c.registration=result;
  c.map_T_base=result.target_T_source*odom_T_base;
  auto reject=[&](const char *reason) { c.reason=reason; return c; };
  if (!result.valid || !result.converged || !result.fine_ran || !rigid(result.target_T_source)) return reject("quality_rejected");
  if (result.fine_inliers < static_cast<std::size_t>(config_.min_fine_inliers) ||
      !std::isfinite(result.inlier_rmse) || result.inlier_rmse>config_.max_inlier_rmse) return reject("residual_or_support_rejected");
  if (guess && !near(*guess*odom_T_base,c.map_T_base,config_.max_translation_correction,config_.max_rotation_correction_deg))
    return reject("seed_correction_exceeded");
  if (apply_region_bounds && hasRegion(active_region_)) {
    const auto &r=regions_.at(active_region_);
    // Back-project ANY candidate, including KISS fallback, to the first snapshot.
    // Current map->base remains available above for clustering/confirmation/output.
    const Eigen::Matrix4d bounds_pose = r.position_bounds_at_reference ?
        (result.target_T_source * reference_odom_).eval() : c.map_T_base;
    const Eigen::Vector3d p=bounds_pose.block<3,1>(0,3);
    if (r.position_min && ((p.array()<r.position_min->array()).any() || (p.array()>r.position_max->array()).any()))
      return reject("region_position_rejected");
    const Eigen::Matrix3d rot=c.map_T_base.block<3,3>(0,0);
    const Eigen::Vector3d angles(std::atan2(rot(2,1),rot(2,2))/rad,
        std::atan2(-rot(2,0),std::hypot(rot(0,0),rot(1,0)))/rad,std::atan2(rot(1,0),rot(0,0))/rad);
    if (r.rpy_min && ((angles.array()<r.rpy_min->array()).any() || (angles.array()>r.rpy_max->array()).any()))
      return reject("region_attitude_rejected");
  }
  c.eligible=true; c.reason="eligible"; return c;
}
InitializationDecision RegionInitializer::decide(std::vector<InitializationCandidate> &candidates,
    std::int64_t stamp,const Eigen::Matrix4d &odom_T_base) {
  InitializationDecision d; std::vector<int> sorted;
  for (std::size_t i=0;i<candidates.size();++i) if(candidates[i].eligible) sorted.push_back(i);
  std::stable_sort(sorted.begin(),sorted.end(),[&](int a,int b) {
    const auto &x=candidates[a].registration;const auto &y=candidates[b].registration;
    if(x.overlap_percent!=y.overlap_percent) return x.overlap_percent>y.overlap_percent;
    return x.inlier_rmse<y.inlier_rmse;
  });
  std::vector<std::vector<int>> clusters;
  // Complete-link grouping avoids merging distinct modes via a chain of seeds.
  for(int i:sorted) {
    int group=-1;
    for(std::size_t j=0;j<clusters.size();++j) {
      bool fits=true;
      for(int k:clusters[j]) fits &= near(candidates[i].map_T_base,candidates[k].map_T_base,config_.cluster_translation,config_.cluster_rotation_deg);
      if(fits) { group=j; break; }
    }
    if(group<0) {group=clusters.size();clusters.emplace_back();}
    clusters[group].push_back(i);candidates[i].cluster=group;
  }
  d.clusters=clusters.size();
  if(sorted.empty()) { previous_.reset();confirmations_=0;d.reason="initialization_no_valid_candidate";return d; }
  d.selected=sorted[0];
  if(clusters.size()>1 && candidates[clusters[0][0]].registration.overlap_percent-
      candidates[clusters[1][0]].registration.overlap_percent<config_.ambiguity_gap_percent) {
    previous_.reset();confirmations_=0;d.reason="initialization_ambiguous";return d;
  }
  const auto &pose=candidates[d.selected].registration.target_T_source;
  if(previous_ && (!near(*previous_*odom_T_base,pose*odom_T_base,config_.consistency_translation,config_.consistency_rotation_deg) ||
      (stamp-confirmation_stamp_)/1e9>config_.confirm_timeout)) {previous_.reset();confirmations_=0;}
  if(!previous_) {previous_=pose;confirmation_stamp_=stamp;confirmations_=1;}
  else if(stamp>confirmation_stamp_ && (stamp-confirmation_stamp_)/1e9>=config_.confirm_min_dt) {
    ++confirmations_;previous_=pose;confirmation_stamp_=stamp;
  }
  d.confirmations=confirmations_;d.ready=confirmations_>=config_.confirm_windows;
  d.reason=d.ready?"initialization_ready":"initialization_wait_confirmation";
  return d;
}
}  // namespace kiss_matcher

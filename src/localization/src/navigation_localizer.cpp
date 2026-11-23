#include "localization/hierarchical_search.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <openssl/evp.h>
#include "kiss_matcher_localization/msg/keyframe_candidates.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include "kiss_matcher_localization/srv/race_initialization.hpp"
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "kiss_matcher_localization/msg/localization_result.hpp"
#include "kiss_matcher_localization/msg/localization_input.hpp"
#include "kiss_matcher_localization/srv/accept_candidate.hpp"
#include "kiss_matcher_localization/srv/relocalize.hpp"
#include "localization/registration.h"
#include "localization/region_map.h"
#include "localization/region_initializer.h"
#include "kiss_matcher_localization/srv/set_region.hpp"

namespace {
using Keyframes = kiss_matcher_localization::msg::KeyframeCandidates;
using Clock = std::chrono::steady_clock;
using Point = kiss_matcher::LocalizationPoint;
using Result = kiss_matcher_localization::msg::LocalizationResult;
using Relocalize = kiss_matcher_localization::srv::Relocalize;
using AcceptCandidate = kiss_matcher_localization::srv::AcceptCandidate;
using SetRegion = kiss_matcher_localization::srv::SetRegion;

std::string fileSha256(const std::string &path) {
  std::ifstream in(path,std::ios::binary);if(!in) throw std::runtime_error("Cannot hash map");
  auto *ctx=EVP_MD_CTX_new();if(!ctx) throw std::runtime_error("Digest allocation");
  EVP_DigestInit_ex(ctx,EVP_sha256(),nullptr);char data[65536];
  while(in){in.read(data,sizeof(data));EVP_DigestUpdate(ctx,data,in.gcount());}
  unsigned char out[EVP_MAX_MD_SIZE];unsigned int count=0;EVP_DigestFinal_ex(ctx,out,&count);EVP_MD_CTX_free(ctx);
  std::ostringstream result;for(unsigned int i=0;i<count;++i)result<<std::hex<<std::setw(2)<<std::setfill('0')<<int(out[i]);return result.str();
}

geometry_msgs::msg::TransformStamped toTransform(const Eigen::Matrix4d &pose,
                                                  const std::string &parent,
                                                  const std::string &child,
                                                  const rclcpp::Time &stamp) {
  geometry_msgs::msg::TransformStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = parent;
  message.child_frame_id = child;
  const Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
  message.transform.translation.x = pose(0, 3);
  message.transform.translation.y = pose(1, 3);
  message.transform.translation.z = pose(2, 3);
  message.transform.rotation.x = q.x();
  message.transform.rotation.y = q.y();
  message.transform.rotation.z = q.z();
  message.transform.rotation.w = q.w();
  return message;
}

double rotationDegrees(const Eigen::Matrix3d &rotation) {
  const double cosine = std::clamp((rotation.trace() - 1.0) / 2.0, -1.0, 1.0);
  return std::acos(cosine) * 180.0 / std::acos(-1.0);
}

}  // namespace

class NavigationLocalizer : public rclcpp::Node {
 public:
  NavigationLocalizer() : Node("navigation_localizer") {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const auto map_pcd = declare_parameter<std::string>("map_pcd", "");
    const auto local_map_topic = declare_parameter<std::string>("local_map_topic", "/local_map");
    const auto local_map_reliability =
        declare_parameter<std::string>("local_map_reliability", "best_effort");
    voxel_resolution_ = declare_parameter<double>("voxel_resolution", 0.1);
    max_local_map_age_sec_ = declare_parameter<double>("max_local_map_age_sec", 10.0);
    retry_interval_sec_ = declare_parameter<double>("retry_interval_sec", 1.0);
    periodic_interval_sec_ = declare_parameter<double>("periodic_interval_sec", 1.0);
    max_auto_translation_m_ = declare_parameter<double>("max_auto_translation_m", 1.0);
    max_auto_rotation_deg_ = declare_parameter<double>("max_auto_rotation_deg", 15.0);
    candidate_timeout_sec_ = declare_parameter<double>("candidate_timeout_sec", 60.0);

    permission_max_success_age_sec_ = declare_parameter<double>("navigation_permission.max_success_age_sec", 5.0);
    permission_max_local_map_age_sec_ = declare_parameter<double>("navigation_permission.max_local_map_age_sec", 2.5);
    permission_max_odom_age_sec_ = declare_parameter<double>("navigation_permission.max_odom_age_sec", 0.5);
    permission_max_consecutive_failures_ = declare_parameter<int>("navigation_permission.max_consecutive_failures", 3);
    if (map_pcd.empty() || map_frame_.empty() || odom_frame_.empty() ||
        base_frame_.empty() || map_frame_ == odom_frame_ ||
        odom_frame_ == base_frame_ || voxel_resolution_ <= 0.0 ||
        max_local_map_age_sec_ < 0.0 || retry_interval_sec_ <= 0.0 ||
        periodic_interval_sec_ < 0.0 || max_auto_translation_m_ < 0.0 ||
        max_auto_rotation_deg_ < 0.0 || candidate_timeout_sec_ <= 0.0 ||
        !std::isfinite(permission_max_success_age_sec_) || permission_max_success_age_sec_ <= 0.0 ||
        !std::isfinite(permission_max_local_map_age_sec_) || permission_max_local_map_age_sec_ <= 0.0 ||
        !std::isfinite(permission_max_odom_age_sec_) || permission_max_odom_age_sec_ <= 0.0 ||
        permission_max_consecutive_failures_ < 1 ||
        (local_map_reliability != "best_effort" &&
         local_map_reliability != "reliable")) {
      throw std::invalid_argument("invalid navigation localization parameters");
    }

    kiss_matcher::LocalizationConfig config;
    config.voxel_resolution = voxel_resolution_;
    config.verbose = declare_parameter<bool>("verbose", true);
    config.enable_coarse = false;  // Continuous localization never constructs or calls KISS.
    config.num_threads = declare_parameter<int>("vgicp_threads", 4);
    config.correspondence_randomness = declare_parameter<int>("vgicp_neighbors", 20);
    config.scale_factor_for_corr_dist =
        declare_parameter<double>("vgicp_max_correspondence_scale", 3.0);
    config.overlap_threshold = declare_parameter<double>("min_overlap_percent", 70.0);
    if (config.num_threads <= 0 ||
        config.correspondence_randomness <= 0 ||
        config.scale_factor_for_corr_dist <= 0.0 ||
        config.overlap_threshold < 0.0 || config.overlap_threshold > 100.0) {
      throw std::invalid_argument("invalid registration parameters");
    }

    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    race_validator_ = declare_parameter<bool>("race_validator", false);
    external_initialization_ = declare_parameter<bool>("external_initialization_enabled", false);
    race_keyframes_sha_ = declare_parameter<std::string>("race_keyframes_sha256", "");
    race_bbs_sha_ = declare_parameter<std::string>("race_bbs_sha256", "");
    race_bbs_profile_sha_ = declare_parameter<std::string>("race_bbs_profile_sha256", "");
    race_profile_sha_ = declare_parameter<std::string>("race_profile_sha256", "");
    race_map_sha_ = external_initialization_ ? fileSha256(map_pcd) : "";
    race_start_ns_ = get_clock()->now().nanoseconds();
    if (external_initialization_ && (race_validator_ || race_keyframes_sha_.size()!=64 || race_bbs_sha_.size()!=64 || race_profile_sha_.size()!=64 || race_bbs_profile_sha_.size()!=64))
      throw std::invalid_argument("Invalid race authority version configuration");
    if (race_validator_ && publish_tf_) throw std::invalid_argument("Private race validator cannot publish TF");
    use_prior_ = declare_parameter<bool>("use_prior_tracking", false);
    capture_inputs_ = declare_parameter<bool>("publish_attempt_inputs", false);
    selected_input_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("selected_cloud",rclcpp::QoS(2).reliable());
    input_capture_pub_ = create_publisher<kiss_matcher_localization::msg::LocalizationInput>(
        "input_snapshot", rclcpp::QoS(10).reliable());
    prior_margin_ = declare_parameter<double>("prior_crop_margin_m", 5.0);
    prior_max_age_ = declare_parameter<double>("prior_max_age_sec", 30.0);
    global_topic_ = declare_parameter<std::string>("global_local_map_topic", "");
    if (prior_margin_ <= 0 || prior_max_age_ <= 0)
      throw std::invalid_argument("invalid prior tracking parameters");
    region_labels_csv_ = declare_parameter<std::string>("region_labels_csv", "");
    const auto initial_region = declare_parameter<std::string>("initial_region", "");
    multistart_enabled_ = declare_parameter<bool>("region_multistart_enabled", false);
    const auto seed_file = declare_parameter<std::string>("region_seed_file", "");
    kiss_matcher::InitializerConfig init;
    init.max_candidates = declare_parameter<int>("region_max_candidates", 64);
    init.confirm_windows = declare_parameter<int>("initialization_confirm_windows", 2);
    init.crop_margin = declare_parameter<double>("region_seed_crop_margin_m", 5.0);
    init.max_translation_correction = declare_parameter<double>("seed_max_translation_correction_m", 1.0);
    init.max_rotation_correction_deg = declare_parameter<double>("seed_max_rotation_correction_deg", 30.0);
    init.cluster_translation = declare_parameter<double>("candidate_cluster_translation_m", 0.5);
    init.cluster_rotation_deg = declare_parameter<double>("candidate_cluster_rotation_deg", 10.0);
    init.ambiguity_gap_percent = declare_parameter<double>("ambiguity_overlap_gap_percent", 5.0);
    init.confirm_min_dt = declare_parameter<double>("initialization_confirm_min_dt_sec", 1.0);
    init.confirm_timeout = declare_parameter<double>("initialization_confirm_timeout_sec", 30.0);
    init.consistency_translation = declare_parameter<double>("initialization_consistency_translation_m", 0.5);
    init.consistency_rotation_deg = declare_parameter<double>("initialization_consistency_rotation_deg", 5.0);
    init.max_inlier_rmse = declare_parameter<double>("initialization_max_inlier_rmse_m", 0.2);
    init.min_fine_inliers = declare_parameter<int>("initialization_min_fine_inliers", 20);
    initialization_tf_timeout_ = declare_parameter<double>("initialization_tf_timeout_sec", 0.2);
    if (!std::isfinite(initialization_tf_timeout_) || initialization_tf_timeout_ <= 0)
      throw std::invalid_argument("invalid initialization TF timeout");
    initializer_ = std::make_unique<kiss_matcher::RegionInitializer>(init);
    if (multistart_enabled_) {
      if (seed_file.empty()) throw std::invalid_argument("region_seed_file required when multistart enabled");
      initializer_->load(seed_file, map_frame_, base_frame_);
      RCLCPP_WARN(get_logger(), "REGION_INIT enabled profile=%s confirmations=%d; exact snapshot TF required", seed_file.c_str(), init.confirm_windows);
    }
    candidate_source_=declare_parameter<std::string>("candidate_source","keyframes");
    if(candidate_source_!="keyframes" && candidate_source_!="bbs" && candidate_source_!="virtual") throw std::invalid_argument("Unsupported candidate source");
    keyframes_enabled_=declare_parameter<bool>("keyframe_initialization_enabled",false);
    if(candidate_source_!="keyframes" && !keyframes_enabled_) throw std::invalid_argument("Dynamic sources require candidate packet validation");
    hierarchy_enabled_=declare_parameter<bool>("hierarchical_initialization_enabled",false);
    hierarchy_confidence_=declare_parameter<double>("hierarchical_min_initial_overlap",80.0);
    if(!std::isfinite(hierarchy_confidence_) || hierarchy_confidence_<60 || hierarchy_confidence_>100)throw std::invalid_argument("invalid hierarchical confidence overlap");
    if(candidate_source_=="virtual" && !hierarchy_enabled_) throw std::invalid_argument("Virtual shortlist requires hierarchical competitor screening");
    if(candidate_source_=="bbs" && hierarchy_enabled_) throw std::invalid_argument("BBS competitors must all be refined; hierarchical screening is not allowed");
    if(hierarchy_enabled_ && !keyframes_enabled_) throw std::invalid_argument("Hierarchical trial requires keyframe candidate packet validation");
    keyframe_region_id_=declare_parameter<int>("keyframe_region_id",2);
    keyframe_expected_=declare_parameter<int>("keyframe_expected_candidates",28);
    keyframe_library_sha_=declare_parameter<std::string>("keyframe_library_sha256","");
    keyframe_profile_sha_=declare_parameter<std::string>("keyframe_profile_sha256","");
    if(keyframes_enabled_) {
      if(!multistart_enabled_ || keyframe_expected_<1 || keyframe_expected_>64 || keyframe_library_sha_.size()!=64 || keyframe_profile_sha_.size()!=64)
        throw std::invalid_argument("Keyframes require multistart and verified library/profile hashes");
      keyframe_map_sha_=fileSha256(map_pcd);
      if(candidate_source_=="bbs")
        RCLCPP_WARN(get_logger(),"BBS_INIT experimental, dynamic competing map-search peaks; no fixed entrance or keyframe position lookup; all retained peaks require VGICP/ambiguity/two windows");
      else if(candidate_source_=="virtual")
        RCLCPP_WARN(get_logger(),"VIRTUAL_INIT: map-raycast templates, NOT measured references; descriptor shortlist is not exhaustive global search; field acceptance pending");
      else
        RCLCPP_WARN(get_logger(),"KEYFRAME_INIT experimental, all database candidates; existing entrance box is replaced by 1m/30deg reference candidate support, NOT unrestricted full-building initialization");
    }
    loadMap(map_pcd);
    if (!initial_region.empty()) setRegionHint(initial_region);
    registration_ = std::make_unique<kiss_matcher::LocalizationRegistration>(
        config, get_logger());
    if(hierarchy_enabled_) {
      auto coarse_config=config;coarse_config.voxel_resolution=.25;coarse_config.max_num_iter=16;coarse_config.verbose=false;
      screening_=std::make_unique<kiss_matcher::LocalizationRegistration>(coarse_config,get_logger());
    }
    const auto metrics_csv = declare_parameter<std::string>("metrics_csv", "");
    if (!metrics_csv.empty()) {
      const std::filesystem::path path(metrics_csv);
      if (std::filesystem::exists(path))
        throw std::invalid_argument("metrics_csv already exists; use a new experiment directory");
      if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
      metrics_.open(path);
      if (!metrics_) throw std::runtime_error("cannot open metrics_csv: " + metrics_csv);
      metrics_ << "attempt,completed_ns,cloud_ns,mode,input_points,voxel_points,global_points,"
                  "target_points_used,source_features,target_features,initial_matches,"
                  "pruned_matches,rotation_inliers,coarse_inliers,coarse_ran,fine_ran,"
                  "matched,applied,reason,overlap_percent,registration_ms,total_ms,"
                  "input_age_at_result_sec,matched_total,applied_total,region_id,region_name,"
                  "region_target_points,region_registration_ms,region_full_fallback,"
                  "region_coarse_inliers,region_overlap_percent,initialization_state,seed_candidates,"
                  "candidate_clusters,confirmation_count,initialization_selected_seed\n";
      metrics_.flush();
      RCLCPP_INFO(get_logger(), "Matching metrics CSV: %s", metrics_csv.c_str());
    }
    auto candidate_csv = declare_parameter<std::string>("region_candidates_csv", "");
    if (multistart_enabled_) {
      if (candidate_csv.empty() && !metrics_csv.empty())
        candidate_csv = (std::filesystem::path(metrics_csv).parent_path() / "region_candidates.csv").string();
      if (!candidate_csv.empty()) {
        const std::filesystem::path path(candidate_csv);
        if (std::filesystem::exists(path)) throw std::invalid_argument("region_candidates_csv already exists");
        if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
        candidate_metrics_.open(path);
        if (!candidate_metrics_) throw std::runtime_error("cannot open region candidate CSV");
        candidate_metrics_ << "attempt,cloud_ns,reference_ns,region_id,seed_id,stage,eligible,reason,cluster,"
            "converged,fine_inliers,overlap_percent,inlier_rmse_m,selected,initialization_state,confirmation_count";
        for (const auto *prefix : {"map_T_base", "map_T_odom", "odom_T_base"})
          for (int i=0;i<4;++i) for(int j=0;j<4;++j) candidate_metrics_ << ',' << prefix << i << j;
        candidate_metrics_ << ",coarse_ran,fine_ran\n"; candidate_metrics_.flush();
        RCLCPP_INFO(get_logger(), "Region candidate CSV: %s", candidate_csv.c_str());
      } else RCLCPP_WARN(get_logger(), "REGION_INIT no candidate CSV configured; set metrics_csv or region_candidates_csv for replay evidence");
    }
    RCLCPP_INFO(get_logger(),
        "Localization config: VGICP-only voxel=%.3fm overlap>%.1f%% retry=%.1fs "
        "period=%.1fs (completion-to-next-start, 100ms polling) prior=%s; coarse features/inliers are N/A",
        voxel_resolution_, config.overlap_threshold, retry_interval_sec_,
        periodic_interval_sec_, use_prior_ ? "true" : "false");

    input_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    registration_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    tf_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options;
    options.callback_group = input_group_;
    rclcpp::QoS input_qos = rclcpp::SensorDataQoS();
    if (local_map_reliability == "reliable") {
      input_qos.reliable();
    }
    if(keyframes_enabled_) keyframe_sub_=create_subscription<Keyframes>("/localization/keyframe_candidates",rclcpp::QoS(2).reliable(),
        [this](Keyframes::ConstSharedPtr m){std::lock_guard<std::mutex> lock(input_mutex_);
          // Only actual odometry-window epochs may reset an accepted trajectory, not an unvalidated candidate packet.
          latest_keyframes_=std::move(m);},options);
    local_map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        local_map_topic, input_qos,
        [this, local_map_topic](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          const auto points = static_cast<std::size_t>(message->width) * message->height;
          const auto frame = message->header.frame_id;
          bool first_message = false;
          {
            std::lock_guard<std::mutex> lock(input_mutex_);
            if(multistart_enabled_ && latest_local_map_ &&
                rclcpp::Time(message->header.stamp)<rclcpp::Time(latest_local_map_->header.stamp)) {
              input_time_reset_=true;latest_global_map_.reset();
            }
            latest_local_map_ = std::move(message);
            last_local_map_wall_ = Clock::now();
            if (!first_local_map_logged_) {
              first_local_map_logged_ = true;
              first_message = true;
            }
          }
          if (first_message) {
            RCLCPP_INFO(get_logger(),
                        "Local map received: topic=%s, frame=%s, points=%zu",
                        local_map_topic.c_str(), frame.c_str(), points);
          }
        }, options);

    if (!global_topic_.empty()) {
      global_map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        global_topic_, input_qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(input_mutex_);
          latest_global_map_ = std::move(message);
        }, options);
    }

    rclcpp::QoS status_qos(1);
    status_qos.reliable().transient_local();
    status_pub_ = create_publisher<Result>("status", status_qos);
    if (!race_validator_) permission_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/localization/navigation_allowed", rclcpp::QoS(1).reliable());
    if(publish_tf_) tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    const auto service_qos = rmw_qos_profile_services_default;

    relocalize_service_ = create_service<Relocalize>(
        "relocalize",
        [this](const std::shared_ptr<Relocalize::Request> request,
               std::shared_ptr<Relocalize::Response> response) {
          response->result = runLocalization(request->apply_if_safe);
          response->result.completed_stamp = get_clock()->now();
          status_pub_->publish(response->result);
        },
        service_qos, registration_group_);

    accept_service_ = create_service<AcceptCandidate>(
        "accept_candidate",
        [this](const std::shared_ptr<AcceptCandidate::Request> request,
               std::shared_ptr<AcceptCandidate::Response> response) {
          response->result = acceptCandidate(request->candidate_id);
          response->result.completed_stamp = get_clock()->now();
          status_pub_->publish(response->result);
        },
        service_qos, registration_group_);

    region_service_ = create_service<SetRegion>(
        "set_region",
        [this](const std::shared_ptr<SetRegion::Request> request,
               std::shared_ptr<SetRegion::Response> response) {
          try {
            setRegionHint(request->region);
            response->success = true;
            response->region_id = active_region_id_;
            response->region_name = active_region_id_ < 0 ? "全图" : region_map_.region(active_region_id_).name;
            response->target_points = active_region_id_ < 0 ? map_cloud_->size() : region_map_.region(active_region_id_).cloud->size();
            response->message = "Hint armed for next attempt; current TF retained; hint consumed on next applied result";
          } catch (const std::exception &error) {
            response->success = false; response->region_id = -1; response->message = error.what();
          }
        }, service_qos, registration_group_);

    rclcpp::SubscriptionOptions race_options;race_options.callback_group=registration_group_;
    if (race_validator_) race_control_sub_ = create_subscription<std_msgs::msg::UInt64>(
        "/localization/race_control", rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::UInt64::ConstSharedPtr m) {
          if(m->data && m->data!=race_generation_) {
            setRegionHint("space_2");
            {std::lock_guard<std::mutex> lock(input_mutex_);latest_keyframes_.reset();}
            {std::lock_guard<std::mutex> lock(state_mutex_);has_applied_=false;last_apply_wall_.reset();}
          }
          race_generation_=m->data;
        }, race_options);
    if (external_initialization_) race_service_ = create_service<kiss_matcher_localization::srv::RaceInitialization>(
        "race_initialization", [this](const std::shared_ptr<kiss_matcher_localization::srv::RaceInitialization::Request> request,
        std::shared_ptr<kiss_matcher_localization::srv::RaceInitialization::Response> response) {
          if(request->submit) {
            race_proposal_=*request;
            response->result=runLocalization(true);
            response->result.completed_stamp=get_clock()->now();status_pub_->publish(response->result);
          }
          response->epoch=race_epoch_;response->waiting=race_waiting_;
        }, service_qos, registration_group_);

    trigger_timer_ = create_wall_timer(
        std::chrono::milliseconds(100), [this]() { triggerAutomatically(); }, registration_group_);
    tf_timer_ = create_wall_timer(
        std::chrono::milliseconds(100), [this]() { publishTf(); }, tf_group_);
    if (!race_validator_) permission_timer_ = create_wall_timer(
        std::chrono::milliseconds(200), [this]() { publishNavigationPermission(); }, tf_group_);
  }

 private:
  struct PendingCandidate {
    std::string id;
    Eigen::Matrix4d map_T_odom;
    Result result;
    Clock::time_point created;
    std::optional<Eigen::Matrix4d> snapshot_odom;
  };

  void loadMap(const std::string &path) {
    const auto cache=declare_parameter<std::string>("map_cache_directory", "");
    if(cache.empty())region_map_.load(path, region_labels_csv_, voxel_resolution_);
    else {
      std::ifstream meta(cache+"/source.txt");std::string source_sha;double voxel;
      if(!(meta>>source_sha>>voxel) || source_sha!=fileSha256(path) || std::abs(voxel-voxel_resolution_)>1e-9)
        throw std::runtime_error("Map cache source/voxel mismatch");
      std::ifstream checks(cache+"/checksums.txt");std::string name,digest;std::size_t verified=0;
      while(checks>>name>>digest) {
        if(name.find('/')!=std::string::npos || name.find("..")!=std::string::npos || fileSha256(cache+"/"+name)!=digest)throw std::runtime_error("Map cache content mismatch");
        ++verified;
      }
      if(verified<2)throw std::runtime_error("Map cache checksums missing");
      region_map_.loadCache(cache,region_labels_csv_);
    }
    map_cloud_ = region_map_.full();
    RCLCPP_INFO(get_logger(), "Global map loaded: path=%s frame=%s points=%zu -> %zu voxel=%.3fm region_id=%s skipped=%zu",
        path.c_str(), map_frame_.c_str(), region_map_.rawPoints(), map_cloud_->size(), voxel_resolution_,
        region_map_.hasLabels() ? "present" : "absent", region_map_.skippedPoints());
    for (const auto &[id, entry] : region_map_.regions())
      RCLCPP_INFO(get_logger(), "MAP_REGION id=%d name=%s points=%zu -> %zu", id, entry.name.c_str(), entry.raw_points, entry.cloud->size());
  }

  void setRegionHint(const std::string &hint) {
    const int id = region_map_.resolveHint(hint);  // Validate before changing current state.
    active_region_id_ = id;
    last_bbs_query_stamp_=0;
    if (initializer_) initializer_->reset();
    recovering_candidates_=false;initialization_reference_ns_=0;
    force_global_next_ = true;
    // Keep the completion cooldown when changing region hints.
    pending_.reset();
    RCLCPP_INFO(get_logger(), "REGION_HINT id=%d name=%s target=%zu; next attempt uses global/region registration, existing TF retained",
        id, id < 0 ? "全图" : region_map_.region(id).name.c_str(),
        id < 0 ? map_cloud_->size() : region_map_.region(id).cloud->size());
  }

  static std::string csvQuote(const std::string &value) {
    std::string out = "\"";
    for (char c : value) { if (c=='"') out += '"'; out += c; }
    return out + "\"";
  }

  Eigen::Matrix4d snapshotOdom(const builtin_interfaces::msg::Time &stamp) {
    // Never use TimePointZero or a saved map->odom as a seed substitute.
    if (rclcpp::Time(stamp).nanoseconds() <= 0) throw std::runtime_error("zero snapshot timestamp");
    const auto tf = tf_buffer_->lookupTransform(odom_frame_, base_frame_,
        rclcpp::Time(stamp, get_clock()->get_clock_type()), rclcpp::Duration::from_seconds(initialization_tf_timeout_));
    const auto &q = tf.transform.rotation;
    Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
    const auto &t = tf.transform.translation;
    if (!rotation.coeffs().allFinite() || std::abs(rotation.norm()-1)>0.01 ||
        !std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z))
      throw std::runtime_error("invalid odom TF");
    Eigen::Matrix4d pose=Eigen::Matrix4d::Identity();
    pose.block<3,3>(0,0)=rotation.normalized().toRotationMatrix();
    pose.block<3,1>(0,3)=Eigen::Vector3d(t.x,t.y,t.z);
    return pose;
  }

  kiss_matcher::LocalizationResult alignInitialized(const pcl::PointCloud<Point> &source) {
    attempt_mode_ = keyframe_attempt_ ? (candidate_source_=="bbs" ? "bbs_multistart" : (candidate_source_=="virtual" ? "virtual_hierarchical" : "keyframe_multistart")) : "region_multistart";
    attempt_region_id_=active_region_id_;
    const auto started=Clock::now();
    const pcl::PointCloud<Point> *target=map_cloud_.get();
    if(active_region_id_>=0) {
      const auto &entry=region_map_.region(active_region_id_);
      attempt_region_name_=entry.name;region_target_points_=entry.cloud->size();if(!keyframe_attempt_) target=entry.cloud.get();
    }
    initializer_->begin(active_region_id_,initialization_stamp_,initialization_odom_);
    initialization_reference_ns_=initializer_->referenceStamp();
    std::vector<kiss_matcher::InitializationCandidate> candidates;
    bool hierarchy_blocked=false;std::string hierarchy_reason;
    const auto &guesses=keyframe_attempt_ ? keyframe_guesses_ : initializer_->guesses();
    if(keyframe_attempt_ && hierarchy_enabled_) {
      attempt_mode_=candidate_source_=="virtual" ? "virtual_hierarchical" : "keyframe_hierarchical";
      auto search=kiss_matcher::hierarchicalSearch(source,*target,guesses,initialization_odom_,initialization_stamp_,*initializer_,*screening_,*registration_,hierarchy_confidence_);
      candidates=std::move(search.candidates);seed_candidates_=search.refined;hierarchy_blocked=search.unresolved_support || search.low_confidence;hierarchy_reason=search.unresolved_support?"hierarchy_unresolved_competitor":"hierarchy_low_confidence";
      RCLCPP_INFO(get_logger(),"HIERARCHY groups=%zu screened=%zu fine=%zu expanded=%s input_candidates=%zu; screen scores are ranking only",search.groups,search.screened,search.refined,search.expanded?"true":"false",guesses.size());
    } else {
      for(const auto &[id,guess]:guesses) {
        auto registration=registration_->alignWithPrior(source,*target,guess,initializer_->config().crop_margin);
        candidates.push_back(initializer_->evaluate(id,keyframe_attempt_ ? (candidate_source_=="bbs" ? "bbs_vgicp" : "keyframe_vgicp") : "seed_vgicp",registration,initialization_odom_,guess,!keyframe_attempt_));
      }
      seed_candidates_=candidates.size();
    }
    initialization_decision_=initializer_->decide(candidates,initialization_stamp_,initialization_odom_);
    if(hierarchy_blocked){initializer_->resetConfirmation();initialization_decision_.ready=false;initialization_decision_.selected=-1;initialization_decision_.confirmations=0;initialization_decision_.reason=hierarchy_reason;}
    if(keyframe_attempt_ && (hierarchy_enabled_ || candidate_source_!="keyframes") && initialization_decision_.selected>=0 && candidates[initialization_decision_.selected].registration.overlap_percent<hierarchy_confidence_){
      initializer_->resetConfirmation();initialization_decision_.ready=false;initialization_decision_.selected=-1;initialization_decision_.confirmations=0;initialization_decision_.reason=candidate_source_=="bbs" ? "bbs_low_confidence" : "hierarchy_low_confidence";
    }
    initialization_selected_seed_.clear();
    kiss_matcher::LocalizationResult selected;
    if(initialization_decision_.selected>=0) {
      const auto &winner=candidates.at(initialization_decision_.selected);
      selected=winner.registration;initialization_selected_seed_=winner.id;
    } else if(!candidates.empty()) selected=candidates.back().registration;
    region_ms_=std::chrono::duration<double,std::milli>(Clock::now()-started).count();
    for(std::size_t index=0;index<candidates.size();++index) {
      const auto &c=candidates[index];const auto &r=c.registration;
      RCLCPP_INFO(get_logger(), "REGION_SEED attempt=%llu seed=%s stage=%s eligible=%s reason=%s cluster=%d overlap=%.3f rmse=%.4f",
          static_cast<unsigned long long>(attempt_count_),c.id.c_str(),c.stage.c_str(),c.eligible?"true":"false",c.reason.c_str(),c.cluster,r.overlap_percent,r.inlier_rmse);
      if(candidate_metrics_) {
        candidate_metrics_ << std::setprecision(17) << attempt_count_ << ',' << initialization_stamp_ << ','
            << initialization_reference_ns_ << ',' << active_region_id_ << ',' << csvQuote(c.id) << ',' << csvQuote(c.stage)
            << ',' << c.eligible << ',' << csvQuote(c.reason) << ',' << c.cluster << ',' << r.converged << ',';
        if(r.fine_ran) candidate_metrics_ << r.fine_inliers;
        candidate_metrics_ << ',';
        if(r.fine_ran) candidate_metrics_ << r.overlap_percent;
        candidate_metrics_ << ',';
        if(r.fine_ran && std::isfinite(r.inlier_rmse)) candidate_metrics_ << r.inlier_rmse;
        candidate_metrics_ << ',' << (static_cast<int>(index)==initialization_decision_.selected) << ','
            << csvQuote(initialization_decision_.reason) << ',' << initialization_decision_.confirmations;
        for(const Eigen::Matrix4d *pose : std::initializer_list<const Eigen::Matrix4d*>{&c.map_T_base,&r.target_T_source,&initialization_odom_})
          for(int i=0;i<4;++i) for(int j=0;j<4;++j) candidate_metrics_ << ',' << (*pose)(i,j);
        candidate_metrics_ << ',' << r.coarse_ran << ',' << r.fine_ran << '\n';
      }
    }
    if(candidate_metrics_.is_open()) {candidate_metrics_.flush();if(!candidate_metrics_) RCLCPP_ERROR(get_logger(),"Cannot flush region candidate CSV");}
    RCLCPP_INFO(get_logger(),"REGION_INIT attempt=%llu state=%s seeds=%zu clusters=%d confirmations=%d selected=%s stamp=%lld",
        static_cast<unsigned long long>(attempt_count_),initialization_decision_.reason.c_str(),seed_candidates_,initialization_decision_.clusters,
        initialization_decision_.confirmations,initialization_selected_seed_.c_str(),static_cast<long long>(initialization_stamp_));
    return selected;
  }

  Result newResult() {
    Result result;
    result.completed_stamp = get_clock()->now();
    result.candidate_map_to_odom.header.frame_id = map_frame_;
    result.candidate_map_to_odom.child_frame_id = odom_frame_;
    std::lock_guard<std::mutex> lock(state_mutex_);
    result.has_previous = has_applied_;
    if (has_applied_) result.last_success_stamp = last_success_stamp_;
    return result;
  }

  Result runLocalization(bool apply_if_safe) {
    const auto started = Clock::now();
    ++attempt_count_;
    diagnostics_ = {};
    initialization_decision_ = {};
    initialization_selected_seed_.clear();
    seed_candidates_ = 0;
    initialization_tf_valid_ = false;
    region_diagnostics_ = {};
    attempt_region_id_ = -1; attempt_region_name_.clear();
    region_target_points_ = 0; region_ms_ = 0; region_did_fallback_ = false;
    attempt_mode_ = "not_run";
    input_points_ = voxel_points_ = 0;
    RCLCPP_INFO(get_logger(), "LOC_BEGIN attempt=%llu",
                static_cast<unsigned long long>(attempt_count_));
    auto result = runLocalizationImpl(apply_if_safe);
    const auto completed = get_clock()->now();
    const double total_ms = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
    matched_count_ += result.matched;
    applied_count_ += result.applied;
    if (result.registration_ms > 0.0) durations_.push_back(result.registration_ms);
    auto sorted = durations_;
    std::sort(sorted.begin(), sorted.end());
    const auto percentile = [&](double fraction) {
      return sorted.empty() ? 0.0 : sorted[static_cast<std::size_t>(std::ceil(fraction * sorted.size())) - 1];
    };
    const auto coarse_value = [&](std::size_t value) {
      return diagnostics_.coarse_ran ? std::to_string(value) : std::string("N/A");
    };
    const auto overlap = diagnostics_.fine_ran ? std::to_string(result.overlap_percent) : "N/A";
    const auto &d = diagnostics_;
    RCLCPP_INFO(get_logger(),
        "LOC_METRICS attempt=%llu mode=%s points=%zu->%zu global=%zu target_used=%zu "
        "features_local=%s features_global=%s matches=%s pruned=%s rot_inliers=%s "
        "coarse_inliers=%s fine_ran=%s overlap_percent=%s",
        static_cast<unsigned long long>(attempt_count_), attempt_mode_.c_str(), input_points_,
        voxel_points_, map_cloud_->size(), d.target_points_used,
        coarse_value(d.source_features).c_str(), coarse_value(d.target_features).c_str(),
        coarse_value(d.initial_matches).c_str(), coarse_value(d.pruned_matches).c_str(),
        coarse_value(d.rotation_inliers).c_str(), coarse_value(result.inliers).c_str(),
        d.fine_ran ? "true" : "false", overlap.c_str());
    RCLCPP_INFO(get_logger(),
        "LOC_RESULT attempt=%llu matched=%s applied=%s reason=%s registration_ms=%.1f "
        "total_ms=%.1f matched_total=%llu applied_total=%llu applied_rate=%.1f%% "
        "registration_p50_ms=%.1f registration_p95_ms=%.1f",
        static_cast<unsigned long long>(attempt_count_), result.matched ? "true" : "false",
        result.applied ? "true" : "false", result.reason.c_str(), result.registration_ms, total_ms,
        static_cast<unsigned long long>(matched_count_), static_cast<unsigned long long>(applied_count_),
        100.0 * applied_count_ / attempt_count_, percentile(.5), percentile(.95));
    if (metrics_.is_open()) {
      // CSV uses blank values for stages which did not run, rather than false zeros.
      const auto csv_coarse = [&](std::size_t value) {
        return d.coarse_ran ? std::to_string(value) : std::string();
      };
      std::string reason;
      for (char ch : result.reason) { if (ch == '"') reason += '"'; reason += ch; }
      const auto cloud_ns = rclcpp::Time(result.local_map_stamp).nanoseconds();
      metrics_ << std::setprecision(15) << attempt_count_ << ',' << completed.nanoseconds() << ','
               << cloud_ns << ',' << attempt_mode_ << ',' << input_points_ << ',' << voxel_points_
               << ',' << map_cloud_->size() << ',' << d.target_points_used << ','
               << csv_coarse(d.source_features) << ',' << csv_coarse(d.target_features) << ','
               << csv_coarse(d.initial_matches) << ',' << csv_coarse(d.pruned_matches) << ','
               << csv_coarse(d.rotation_inliers) << ',' << csv_coarse(result.inliers) << ','
               << d.coarse_ran << ',' << d.fine_ran << ',' << result.matched << ',' << result.applied
               << ",\"" << reason << "\",";
      if (d.fine_ran) metrics_ << result.overlap_percent;
      metrics_ << ',' << result.registration_ms << ',' << total_ms << ',';
      if (cloud_ns != 0) metrics_ << (completed.nanoseconds() - cloud_ns) / 1e9;
      metrics_ << ',' << matched_count_ << ',' << applied_count_ << ',';
      if (attempt_region_id_ >= 0) metrics_ << attempt_region_id_;
      std::string region_name_csv;
      for (char ch : attempt_region_name_) { if (ch == '"') region_name_csv += '"'; region_name_csv += ch; }
      metrics_ << ",\"" << region_name_csv << "\"," << region_target_points_ << ',' << region_ms_ << ',' << region_did_fallback_ << ',';
      if (region_diagnostics_.coarse_ran) metrics_ << region_diagnostics_.num_final_inliers;
      metrics_ << ',';
      if (region_diagnostics_.fine_ran) metrics_ << region_diagnostics_.overlap_percent;
      metrics_ << ',' << csvQuote(initialization_decision_.reason) << ',' << seed_candidates_ << ','
          << initialization_decision_.clusters << ',' << initialization_decision_.confirmations << ','
          << csvQuote(initialization_selected_seed_) << '\n';
      metrics_.flush();
      if (!metrics_) RCLCPP_ERROR(get_logger(), "Cannot flush matching metrics CSV; check disk space");
    }
    // Every completed attempt, including early rejection and service calls, starts the cooldown.
    last_completed_ = Clock::now();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (result.applied) consecutive_failures_ = 0;
      else if (!result.matched && consecutive_failures_ < permission_max_consecutive_failures_)
        ++consecutive_failures_;
    }
    return result;
  }

  void captureInput(const sensor_msgs::msg::PointCloud2 &cloud,
                    const std::string &stage, const std::string &topic) {
    if(keyframes_enabled_) selected_input_pub_->publish(cloud);
    if (!capture_inputs_) return;
    if (input_capture_pub_->get_subscription_count() == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
          "Attempt input capture enabled but no recorder subscribes to /localization/input_snapshot");
      return;
    }
    kiss_matcher_localization::msg::LocalizationInput capture;
    capture.attempt = attempt_count_;
    capture.stage = stage;
    capture.source_topic = topic;
    capture.selected_stamp = get_clock()->now();
    capture.cloud = cloud;
    input_capture_pub_->publish(capture);
  }

  void openRace() {
    if(!external_initialization_ || race_waiting_) return;
    race_waiting_=true;++race_epoch_;race_start_ns_=get_clock()->now().nanoseconds();
  }

  Result adoptRace() {
    auto request=*race_proposal_;race_proposal_.reset();
    Result result=newResult();attempt_mode_="race_"+request.source;
    auto reject=[&](const std::string &why){result.reason=why;return result;};
    if(!race_waiting_ || request.epoch!=race_epoch_) return reject("race_epoch_closed_or_changed");
    const auto expected=request.source=="bbs" ? race_bbs_sha_ : (request.source=="keyframes" ? race_keyframes_sha_ : "");
    if(expected.empty() || request.library_sha256!=expected || request.map_sha256!=race_map_sha_ || request.profile_sha256!=(request.source=="bbs" ? race_bbs_profile_sha_ : race_profile_sha_))
      return reject("race_version_or_source_mismatch");
    const auto &p=request.proposal;
    const auto stamp=rclcpp::Time(p.local_map_stamp).nanoseconds();
    const double age=(get_clock()->now().nanoseconds()-stamp)/1e9;
    if(stamp<race_start_ns_ || age<0 || age>max_local_map_age_sec_ || !p.matched || !p.applied || p.reason!="applied" || !std::isfinite(p.overlap_percent) || p.overlap_percent<80)
      return reject("race_proposal_unconfirmed_or_expired");
    const auto &t=p.candidate_map_to_odom;
    if(t.header.frame_id!=map_frame_ || t.child_frame_id!=odom_frame_)return reject("race_frame_mismatch");
    Eigen::Quaterniond q(t.transform.rotation.w,t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z);
    Eigen::Matrix4d candidate=Eigen::Matrix4d::Identity();
    candidate.block<3,1>(0,3)=Eigen::Vector3d(t.transform.translation.x,t.transform.translation.y,t.transform.translation.z);
    if(!q.coeffs().allFinite() || std::abs(q.norm()-1)>1e-5 || !candidate.allFinite())return reject("race_invalid_transform");
    candidate.block<3,3>(0,0)=q.normalized().toRotationMatrix();
    try {
      const auto ob=snapshotOdom(p.local_map_stamp);
      bool previous;Eigen::Matrix4d old;
      {std::lock_guard<std::mutex> lock(state_mutex_);previous=has_applied_;old=applied_map_T_odom_;}
      result=p;result.applied=false;result.has_previous=previous;result.candidate_id.clear();
      if(previous) {
        Eigen::Matrix4d delta=(old*ob).inverse()*(candidate*ob);
        result.translation_delta_m=delta.block<3,1>(0,3).norm();result.rotation_delta_deg=rotationDegrees(delta.block<3,3>(0,0));result.delta_valid=true;
        if(result.translation_delta_m>max_auto_translation_m_ || result.rotation_delta_deg>max_auto_rotation_deg_)return reject("race_jump_exceeds_limit");
      }
      if(apply(candidate,result)) {race_waiting_=false;result.reason="applied";}
      // Refinement ran in the private validator, not again in this owner.
      diagnostics_.fine_ran=true;result.completed_stamp=get_clock()->now();
      return result;
    } catch(const std::exception &e) {return reject(std::string("race_snapshot_unavailable: ")+e.what());}
  }

  Result runLocalizationImpl(bool apply_if_safe) {
    if(external_initialization_ && race_proposal_) return adoptRace();
    pending_.reset();
    if(multistart_enabled_) {
      bool reset=false;
      {std::lock_guard<std::mutex> lock(input_mutex_);reset=input_time_reset_;input_time_reset_=false;}
      if(reset) {
        initializer_->reset();initialization_reference_ns_=0;last_bbs_query_stamp_=0;
        std::lock_guard<std::mutex> lock(state_mutex_);
        has_applied_=false;recovering_candidates_=false;last_apply_wall_.reset();applied_map_T_odom_.setIdentity();
        force_global_next_=true;
        if(external_initialization_){race_waiting_=false;openRace();}
        RCLCPP_WARN(get_logger(),"REGION_INIT input time reset; discarded applied transform and confirmation history");
      }
    }
    Result result = newResult();
    sensor_msgs::msg::PointCloud2::ConstSharedPtr snapshot;
    std::string snapshot_topic = get_parameter("local_map_topic").as_string();
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      snapshot = latest_local_map_;
    }
    if (!snapshot) {
      result.reason = "no_local_map";
      return result;
    }
    // Without a valid recent prior, use the larger recovery/initialization window.
    bool try_prior = false;
    Eigen::Matrix4d prior = Eigen::Matrix4d::Identity();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      try_prior = active_region_id_ < 0 && !force_global_next_ && !recovering_candidates_ && use_prior_ && has_applied_ && last_apply_wall_ &&
        std::chrono::duration<double>(Clock::now() - *last_apply_wall_).count() < prior_max_age_;
      prior = applied_map_T_odom_;
    }
    if(external_initialization_ && !try_prior) {
      openRace();result.reason="external_initialization_pending";return result;
    }
    if (!try_prior && keyframes_enabled_ && has_applied_ && active_region_id_<0 && !recovering_candidates_) {
      recovering_candidates_=true;initializer_->reset();initialization_reference_ns_=0;
    }
    if (!try_prior) {
      std::lock_guard<std::mutex> lock(input_mutex_);
      if (latest_global_map_) { snapshot = latest_global_map_; snapshot_topic = global_topic_; }
    }
    keyframe_attempt_=false;keyframe_guesses_.clear();
    if(keyframes_enabled_ && !try_prior && (active_region_id_==keyframe_region_id_ || (active_region_id_<0 && recovering_candidates_))) {
      Keyframes::ConstSharedPtr m;{std::lock_guard<std::mutex> lock(input_mutex_);m=latest_keyframes_;}
      if(!m || !m->ready) {
        result.reason="keyframe_query_unavailable";
        const std::set<std::string> temporary={"no_short_cloud","short_cloud_warmup","query_odom_missing","estimated_gravity_missing_at_query_stamp","bbs_search_pending"};
        if(m && m->region_id==keyframe_region_id_ && m->map_sha256==keyframe_map_sha_ &&
            m->library_sha256==keyframe_library_sha_ && m->profile_sha256==keyframe_profile_sha_ &&
            m->cloud.header.frame_id==odom_frame_ && temporary.count(m->reason)) {
          initialization_decision_.confirmations=initializer_->waitForQuery(get_clock()->now().nanoseconds());
          initialization_decision_.reason="initialization_wait_query";
          RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,"INIT_WAIT query=%s confirmations=%d; no new match, time limit still enforced",m->reason.c_str(),initialization_decision_.confirmations);
        } else initializer_->resetConfirmation();
        return result;
      }
      if(m->map_sha256!=keyframe_map_sha_ || m->library_sha256!=keyframe_library_sha_ || m->profile_sha256!=keyframe_profile_sha_ || m->region_id!=keyframe_region_id_) {
        result.reason="keyframe_version_or_region_mismatch";initializer_->resetConfirmation();return result;
      }
      if((candidate_source_!="keyframes" ? (m->map_to_odom.empty() || m->map_to_odom.size()>static_cast<std::size_t>(keyframe_expected_) || m->reason!=(candidate_source_=="bbs" ? "bbs_competitors_complete" : "virtual_shortlist")) : m->map_to_odom.size()!=static_cast<std::size_t>(keyframe_expected_)) || m->window_start_ns==0 || rclcpp::Time(m->cloud.header.stamp).nanoseconds()<static_cast<std::int64_t>(m->window_start_ns) ||
          (rclcpp::Time(m->cloud.header.stamp).nanoseconds()-m->window_start_ns)/1e9<0.8 ||
          (rclcpp::Time(m->cloud.header.stamp).nanoseconds()-m->window_start_ns)/1e9>1.05 || m->ids.size()!=m->map_to_odom.size() || m->scores.size()!=m->ids.size()) {
        result.reason="keyframe_candidate_shape_invalid";initializer_->resetConfirmation();return result;
      }
      std::set<std::string> ids(m->ids.begin(),m->ids.end());
      if(ids.size()!=m->ids.size() || ids.count("")){result.reason="keyframe_candidate_ids_invalid";initializer_->resetConfirmation();return result;}
      for(std::size_t i=0;i<m->map_to_odom.size();++i) {
        const auto &v=m->map_to_odom[i];Eigen::Quaterniond q(v.rotation.w,v.rotation.x,v.rotation.y,v.rotation.z);
        Eigen::Matrix4d T=Eigen::Matrix4d::Identity();
        if(!q.coeffs().allFinite() || std::abs(q.norm()-1)>0.01 || !std::isfinite(m->scores[i])) {result.reason="keyframe_candidate_nonfinite";initializer_->resetConfirmation();return result;}
        T.block<3,3>(0,0)=q.normalized().toRotationMatrix();T.block<3,1>(0,3)=Eigen::Vector3d(v.translation.x,v.translation.y,v.translation.z);
        if(!T.allFinite()){result.reason="keyframe_candidate_nonfinite";initializer_->resetConfirmation();return result;}
        keyframe_guesses_.emplace_back(m->ids[i],T);
      }
      snapshot=std::make_shared<sensor_msgs::msg::PointCloud2>(m->cloud);
      snapshot_topic="/localization/keyframe_candidates";keyframe_attempt_=true;
    }
    if(!try_prior && !keyframe_attempt_ && (!multistart_enabled_ || !initializer_->hasRegion(active_region_id_))) {
      result.reason=has_applied_ ? "prior_expired_reinitialize_required" : "initialization_candidates_unavailable";
      return result;
    }
    result.local_map_stamp = snapshot->header.stamp;
    input_points_ = static_cast<std::size_t>(snapshot->width) * snapshot->height;
    if (snapshot->header.frame_id != odom_frame_) {
      result.reason = "local_map_frame_mismatch";
      return result;
    }
    const auto age = (get_clock()->now() - rclcpp::Time(snapshot->header.stamp)).seconds();
    if (max_local_map_age_sec_ > 0.0 &&
        (age > max_local_map_age_sec_ || age < -1.0)) {
      if(keyframe_attempt_) initializer_->resetConfirmation();
      result.reason = "local_map_stale_or_future_dated";
      return result;
    }

    if(keyframe_attempt_ && candidate_source_!="keyframes") {
      const auto stamp=rclcpp::Time(snapshot->header.stamp).nanoseconds();
      if(last_bbs_query_stamp_ && stamp<=last_bbs_query_stamp_) {
        result.reason=stamp==last_bbs_query_stamp_ ? "keyframe_query_already_processed" : "bbs_query_out_of_order";
        if(stamp==last_bbs_query_stamp_) {
          initialization_decision_.confirmations=initializer_->waitForQuery(get_clock()->now().nanoseconds());
          initialization_decision_.reason="initialization_wait_query";
        } else initializer_->resetConfirmation();
        return result;
      }
    }
    force_global_next_ = false;
    try {
      captureInput(*snapshot, keyframe_attempt_ ? "keyframe" : (try_prior ? "prior" : "global"), snapshot_topic);
      auto source = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>());
      pcl::fromROSMsg(*snapshot, *source);
      if (source->empty()) {
        result.reason = "empty_local_map";
        return result;
      }
      pcl::VoxelGrid<Point> filter;
      filter.setLeafSize(voxel_resolution_, voxel_resolution_, voxel_resolution_);
      filter.setInputCloud(source);
      pcl::PointCloud<Point> sampled;
      filter.filter(sampled);
      voxel_points_ = sampled.size();
      if (sampled.empty()) {
        result.reason = "empty_voxelized_local_map";
        return result;
      }
      RCLCPP_INFO(get_logger(),
                  "Local map prepared: frame=%s, message_points=%zu, "
                  "decoded_points=%zu, voxelized_points=%zu (voxel=%.3f m)",
                  snapshot->header.frame_id.c_str(),
                  static_cast<std::size_t>(snapshot->width) * snapshot->height,
                  source->size(), sampled.size(), voxel_resolution_);
      const auto start = Clock::now();
      kiss_matcher::LocalizationResult registration;
      if (try_prior) {
        attempt_mode_ = "prior_vgicp";
        registration = registration_->alignWithPrior(sampled, *map_cloud_, prior, prior_margin_);
      }
      if (try_prior && !registration.valid) {
        result.registration_ms=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
        diagnostics_=registration;result.overlap_percent=registration.overlap_percent;
        result.reason="prior_registration_rejected";pending_.reset();
        if(external_initialization_){force_global_next_=true;openRace();}
        if(candidate_source_!="keyframes") last_bbs_query_stamp_=rclcpp::Time(snapshot->header.stamp).nanoseconds();
        if(keyframes_enabled_){recovering_candidates_=true;force_global_next_=true;initializer_->reset();initialization_reference_ns_=0;}
        RCLCPP_WARN(get_logger(),"Prior VGICP rejected; no coarse fallback. Next fresh input uses %s",external_initialization_?"BBS + keyframes race with independent two-window confirmation":(keyframes_enabled_?"database candidates with two-window confirmation":"prior VGICP until expiry; then explicit reinitialization required"));
        return result;
      }
      if (!try_prior) {
        if (multistart_enabled_) {
          try {
            initialization_odom_=snapshotOdom(snapshot->header.stamp);
            initialization_stamp_=rclcpp::Time(snapshot->header.stamp).nanoseconds();
            initialization_tf_valid_=true;
          } catch(const std::exception &error) {
            initializer_->reset();initialization_reference_ns_=0;
            result.reason="initialization_tf_unavailable";
            initialization_decision_.reason=result.reason;
            RCLCPP_WARN(get_logger(),"REGION_INIT exact snapshot TF unavailable: %s",error.what());
            return result;
          }
        }
        if(keyframe_attempt_ && candidate_source_!="keyframes") last_bbs_query_stamp_=initialization_stamp_;
        registration = alignInitialized(sampled);
        RCLCPP_INFO(get_logger(), "Registration mode: %s", attempt_mode_.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "Registration mode: prior VGICP (coarse inliers not applicable)");
      }
      result.registration_ms =
          std::chrono::duration<double, std::milli>(Clock::now() - start).count();
      diagnostics_ = registration;
      result.inliers = static_cast<std::uint32_t>(registration.num_final_inliers);
      result.overlap_percent = registration.overlap_percent;
      if (multistart_enabled_) {
        const double completed_age = (get_clock()->now() - rclcpp::Time(result.local_map_stamp)).seconds();
        if (max_local_map_age_sec_ > 0 && (completed_age > max_local_map_age_sec_ || completed_age < -1.0)) {
          if (completed_age < -1.0) initializer_->reset();
          else initializer_->resetConfirmation();
          initialization_reference_ns_=initializer_->referenceStamp();
          result.reason="initialization_input_expired_during_registration";
          return result;
        }
      }
      if (multistart_enabled_ && initialization_tf_valid_ && !initialization_decision_.ready) {
        result.reason=initialization_decision_.reason;
        if(result.reason=="initialization_wait_confirmation" && initialization_decision_.selected>=0) {
          result.matched=true;
          result.candidate_map_to_odom=toTransform(registration.target_T_source,map_frame_,odom_frame_,
              rclcpp::Time(snapshot->header.stamp,get_clock()->get_clock_type()));
        }
        return result;  // No pending candidate: accept_candidate cannot bypass this gate.
      }
      if (!registration.valid) {
        result.reason = "registration_rejected";
        return result;
      }

      result.matched = true;
      const auto &candidate = registration.target_T_source;
      result.candidate_map_to_odom =
          toTransform(candidate, map_frame_, odom_frame_, get_clock()->now());
      Eigen::Matrix4d previous = Eigen::Matrix4d::Identity();
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (has_applied_) previous = applied_map_T_odom_;
      }
      if (result.has_previous) {
        try {
          Eigen::Matrix4d odom_T_base;
          if (multistart_enabled_) {
            odom_T_base = initialization_tf_valid_ ? initialization_odom_ : snapshotOdom(snapshot->header.stamp);
          } else {
          const auto odom_to_base = tf_buffer_->lookupTransform(
              odom_frame_, base_frame_, tf2::TimePointZero,
              tf2::durationFromSec(0.2));
          const auto &translation = odom_to_base.transform.translation;
          const auto &rotation = odom_to_base.transform.rotation;
          odom_T_base = Eigen::Matrix4d::Identity();
          odom_T_base.block<3, 3>(0, 0) =
              Eigen::Quaterniond(rotation.w, rotation.x, rotation.y, rotation.z)
                  .toRotationMatrix();
          odom_T_base.block<3, 1>(0, 3) =
              Eigen::Vector3d(translation.x, translation.y, translation.z);
          }
          const Eigen::Matrix4d old_map_T_base = previous * odom_T_base;
          const Eigen::Matrix4d new_map_T_base = candidate * odom_T_base;
          const Eigen::Matrix4d delta = old_map_T_base.inverse() * new_map_T_base;
          result.translation_delta_m = delta.block<3, 1>(0, 3).norm();
          result.rotation_delta_deg = rotationDegrees(delta.block<3, 3>(0, 0));
          result.delta_valid = true;
        } catch (const std::exception &error) {
          RCLCPP_WARN(get_logger(), "Cannot compare robot poses: %s", error.what());
        }
      }

      const bool jump_is_safe = !result.has_previous ||
          (result.delta_valid &&
          (result.translation_delta_m <= max_auto_translation_m_ &&
           result.rotation_delta_deg <= max_auto_rotation_deg_));
      if (apply_if_safe && jump_is_safe) {
        if (apply(candidate, result)) result.reason = "applied";
      } else {
        result.reason = !apply_if_safe ? "preview_only" :
            (result.has_previous && !result.delta_valid ?
             "base_transform_unavailable" : "jump_exceeds_limit");
        result.candidate_id = std::to_string(++candidate_sequence_);
        pending_ = PendingCandidate{result.candidate_id, candidate, result, Clock::now(), std::nullopt};
        if(multistart_enabled_) {
          try {pending_->snapshot_odom=snapshotOdom(result.local_map_stamp);}
          catch(const std::exception &) {pending_.reset();result.candidate_id.clear();}
        }
      }
      RCLCPP_INFO(get_logger(),
                  "Localization: %s; inliers=%u overlap=%.2f%% delta=%.3f m, %.2f deg",
                  result.reason.c_str(), result.inliers, result.overlap_percent,
                  result.translation_delta_m, result.rotation_delta_deg);
      return result;
    } catch (const std::exception &error) {
      result.reason = std::string("exception: ") + error.what();
      RCLCPP_ERROR(get_logger(), "%s", result.reason.c_str());
      return result;
    }
  }

  bool apply(const Eigen::Matrix4d &map_T_odom, Result &result) {
    {
      // Input reset and state update must be atomic with respect to input callbacks.
      std::scoped_lock lock(input_mutex_, state_mutex_);
      if (multistart_enabled_) {
        const double age = (get_clock()->now() - rclcpp::Time(result.local_map_stamp)).seconds();
        if (input_time_reset_ || (max_local_map_age_sec_ > 0 && (age > max_local_map_age_sec_ || age < -1.0))) {
          result.reason=input_time_reset_ ? "initialization_epoch_changed" : "initialization_input_expired_before_apply";
          result.applied=false;
          if (input_time_reset_ || age < -1.0) initializer_->reset();
          else initializer_->resetConfirmation();
          initialization_reference_ns_=initializer_->referenceStamp();
          return false;
        }
      }
      applied_map_T_odom_ = map_T_odom;
      has_applied_ = true;
      last_success_stamp_ = get_clock()->now();
      last_apply_wall_ = Clock::now();
      consecutive_failures_ = 0;
      result.last_success_stamp = last_success_stamp_;
    }
    result.applied = true;
    if (initializer_) initializer_->reset();
    initialization_reference_ns_=0;
    if (active_region_id_ >= 0) {
      RCLCPP_INFO(get_logger(), "Region hint %d consumed after applied result; subsequent tracking uses full map; recovery uses configured VGICP candidates", active_region_id_);
      active_region_id_ = -1;
    }
    force_global_next_ = false;recovering_candidates_=false;
    publishTf();
    return true;
  }

  Result acceptCandidate(const std::string &id) {
    Result result = newResult();
    if (!pending_ || id.empty() || id != pending_->id) {
      result.reason = "candidate_not_found";
      return result;
    }
    const auto age = std::chrono::duration<double>(Clock::now() - pending_->created).count();
    if (age > candidate_timeout_sec_) {
      pending_.reset();
      result.reason = "candidate_expired";
      return result;
    }
    if(multistart_enabled_) {
      try {
        bool time_reset=false;
        {std::lock_guard<std::mutex> lock(input_mutex_);time_reset=input_time_reset_;}
        if(time_reset || !pending_->snapshot_odom ||
            (snapshotOdom(pending_->result.local_map_stamp)-*pending_->snapshot_odom).norm()>1e-5)
          throw std::runtime_error("pending candidate odom epoch changed");
      } catch(const std::exception &) {
        pending_.reset();result.reason="candidate_snapshot_tf_unavailable_or_changed";return result;
      }
    }
    result = pending_->result;
    result.completed_stamp = get_clock()->now();
    result.candidate_id.clear();
    if (apply(pending_->map_T_odom, result)) result.reason = "candidate_accepted";
    pending_.reset();
    return result;
  }

  void triggerAutomatically() {
    if(race_validator_ && !race_generation_) return;
    if(external_initialization_ && race_waiting_) {
      std::lock_guard<std::mutex> lock(input_mutex_);
      if(!input_time_reset_) return; // Waiting is not a registration attempt; do not inflate metrics.
    }
    const auto now = Clock::now();
    bool initialized;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      initialized = has_applied_;
    }
    if (initialized && periodic_interval_sec_ == 0.0) return;
    const double interval = initialized ? periodic_interval_sec_ : retry_interval_sec_;
    if (last_completed_ &&
        std::chrono::duration<double>(now - *last_completed_).count() < interval) return;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      if (!latest_local_map_) return;
    }
    auto result = runLocalization(true);
    result.completed_stamp = get_clock()->now();
    status_pub_->publish(result);
  }

  void publishTf() {
    if(!publish_tf_)return;
    std::scoped_lock lock(input_mutex_, state_mutex_);
    if (!has_applied_ || (multistart_enabled_ && input_time_reset_)) return;
    const Eigen::Matrix4d pose = applied_map_T_odom_;
    tf_broadcaster_->sendTransform(toTransform(pose, map_frame_, odom_frame_,
                                             get_clock()->now()));
  }

  void publishNavigationPermission() {
    const auto wall_now = Clock::now();
    const auto ros_now = get_clock()->now();
    const auto fresh = [&ros_now](const rclcpp::Time &stamp, double limit) {
      const double age = (ros_now - stamp).seconds();
      return std::isfinite(age) && age >= 0.0 && age <= limit;
    };
    bool applied = false;
    bool success_fresh = false;
    bool failures_ok = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      applied = has_applied_;
      success_fresh = last_apply_wall_ &&
          std::chrono::duration<double>(wall_now - *last_apply_wall_).count() <= permission_max_success_age_sec_ &&
          fresh(last_success_stamp_, permission_max_success_age_sec_);
      failures_ok = consecutive_failures_ < permission_max_consecutive_failures_;
    }
    bool map_fresh = false;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      map_fresh = !input_time_reset_ && latest_local_map_ && last_local_map_wall_ &&
          latest_local_map_->width > 0 && latest_local_map_->height > 0 &&
          latest_local_map_->header.frame_id == odom_frame_ &&
          std::chrono::duration<double>(wall_now - *last_local_map_wall_).count() <= permission_max_local_map_age_sec_ &&
          fresh(rclcpp::Time(latest_local_map_->header.stamp), permission_max_local_map_age_sec_);
    }
    bool odom_fresh = false;
    if (applied && success_fresh && failures_ok && map_fresh) {
      try {
        const auto tf = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
        odom_fresh = fresh(rclcpp::Time(tf.header.stamp), permission_max_odom_age_sec_);
      } catch (const tf2::TransformException &) {
        odom_fresh = false;
      }
    }
    std_msgs::msg::Bool permission;
    permission.data = applied && success_fresh && failures_ok && map_fresh && odom_fresh;
    permission_pub_->publish(permission);
    if (!last_permission_ || *last_permission_ != permission.data) {
      RCLCPP_INFO(get_logger(), "Navigation permission %s: applied=%s success_fresh=%s failures_ok=%s map_fresh=%s odom_fresh=%s",
          permission.data ? "GRANTED" : "DENIED", applied ? "true" : "false",
          success_fresh ? "true" : "false", failures_ok ? "true" : "false",
          map_fresh ? "true" : "false", odom_fresh ? "true" : "false");
      last_permission_ = permission.data;
    }
  }

  bool publish_tf_=true,race_validator_=false,external_initialization_=false,race_waiting_=true;
  std::uint64_t race_epoch_=1,race_generation_=0;
  std::int64_t race_start_ns_=0;
  std::string race_map_sha_,race_keyframes_sha_,race_bbs_sha_,race_profile_sha_,race_bbs_profile_sha_;
  std::optional<kiss_matcher_localization::srv::RaceInitialization::Request> race_proposal_;
  rclcpp::Service<kiss_matcher_localization::srv::RaceInitialization>::SharedPtr race_service_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr race_control_sub_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double voxel_resolution_ = 0.1;
  double max_local_map_age_sec_ = 10.0;
  double retry_interval_sec_ = 1.0;
  double periodic_interval_sec_ = 1.0;
  double max_auto_translation_m_ = 1.0;
  double max_auto_rotation_deg_ = 15.0;
  double candidate_timeout_sec_ = 60.0;

  double permission_max_success_age_sec_ = 5.0;
  double permission_max_local_map_age_sec_ = 2.5;
  double permission_max_odom_age_sec_ = 0.5;
  int permission_max_consecutive_failures_ = 3;
  int consecutive_failures_ = 0;  // guarded by state_mutex_
  std::optional<bool> last_permission_;
  bool multistart_enabled_ = false, initialization_tf_valid_ = false;
  bool input_time_reset_ = false;  // guarded by input_mutex_
  double initialization_tf_timeout_ = 0.2;
  std::unique_ptr<kiss_matcher::RegionInitializer> initializer_;
  kiss_matcher::InitializationDecision initialization_decision_;
  Eigen::Matrix4d initialization_odom_ = Eigen::Matrix4d::Identity();
  std::int64_t initialization_stamp_ = 0, initialization_reference_ns_ = 0;
  std::size_t seed_candidates_ = 0;
  std::string initialization_selected_seed_;
  std::ofstream candidate_metrics_;
  bool use_prior_ = false;
  double prior_margin_ = 5.0;
  double prior_max_age_ = 30.0;
  std::string global_topic_;
  std::optional<Clock::time_point> last_apply_wall_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_global_map_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_sub_;
  pcl::PointCloud<Point>::Ptr map_cloud_;
  kiss_matcher::RegionMap region_map_;
  std::string region_labels_csv_;
  std::string candidate_source_;
  std::int64_t last_bbs_query_stamp_=0;
  bool keyframes_enabled_=false,keyframe_attempt_=false;
  std::string keyframe_map_sha_,keyframe_library_sha_,keyframe_profile_sha_;
  int keyframe_region_id_=2,keyframe_expected_=28;
  Keyframes::ConstSharedPtr latest_keyframes_;
  rclcpp::Subscription<Keyframes>::SharedPtr keyframe_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr selected_input_pub_;
  std::vector<std::pair<std::string,Eigen::Matrix4d>> keyframe_guesses_;
  int active_region_id_ = -1, attempt_region_id_ = -1;
  bool force_global_next_ = false, recovering_candidates_ = false, region_did_fallback_ = false;
  std::string attempt_region_name_;
  std::size_t region_target_points_ = 0;
  double region_ms_ = 0;
  kiss_matcher::LocalizationResult region_diagnostics_;
  bool hierarchy_enabled_=false;double hierarchy_confidence_=80;
  std::unique_ptr<kiss_matcher::LocalizationRegistration> screening_;
  std::unique_ptr<kiss_matcher::LocalizationRegistration> registration_;
  std::mutex input_mutex_;
  bool first_local_map_logged_ = false;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_local_map_;
  std::optional<Clock::time_point> last_local_map_wall_;  // guarded by input_mutex_
  std::mutex state_mutex_;
  bool has_applied_ = false;
  Eigen::Matrix4d applied_map_T_odom_ = Eigen::Matrix4d::Identity();
  rclcpp::Time last_success_stamp_;
  std::optional<Clock::time_point> last_completed_;
  std::optional<PendingCandidate> pending_;
  std::uint64_t candidate_sequence_ = 0;
  std::uint64_t attempt_count_ = 0, matched_count_ = 0, applied_count_ = 0;
  std::size_t input_points_ = 0, voxel_points_ = 0;
  std::string attempt_mode_ = "not_run";
  kiss_matcher::LocalizationResult diagnostics_;
  std::vector<double> durations_;
  std::ofstream metrics_;
  bool capture_inputs_ = false;
  rclcpp::Publisher<kiss_matcher_localization::msg::LocalizationInput>::SharedPtr input_capture_pub_;

  rclcpp::CallbackGroup::SharedPtr input_group_;
  rclcpp::CallbackGroup::SharedPtr registration_group_;
  rclcpp::CallbackGroup::SharedPtr tf_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_sub_;
  rclcpp::Publisher<Result>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr permission_pub_;
  rclcpp::Service<Relocalize>::SharedPtr relocalize_service_;
  rclcpp::Service<SetRegion>::SharedPtr region_service_;
  rclcpp::Service<AcceptCandidate>::SharedPtr accept_service_;
  rclcpp::TimerBase::SharedPtr trigger_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr permission_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationLocalizer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

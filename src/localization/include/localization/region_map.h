#pragma once

#include <cstddef>
#include <map>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace kiss_matcher {

struct MapRegion {
  std::string name;
  std::size_t raw_points = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
};

// Region labels remain categorical. Each region is selected BEFORE voxel filtering.
class RegionMap {
 public:
  void load(const std::string &pcd, const std::string &labels_csv, double voxel);
  int resolveHint(const std::string &hint) const;  // -1 explicitly clears the hint.
  const MapRegion &region(int id) const;
  const std::map<int, MapRegion> &regions() const { return regions_; }
  pcl::PointCloud<pcl::PointXYZI>::Ptr full() const { return full_; }
  std::size_t rawPoints() const { return raw_points_; }
  std::size_t skippedPoints() const { return skipped_points_; }
  bool hasLabels() const { return has_labels_; }
 private:
  std::map<int, MapRegion> regions_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr full_;
  std::size_t raw_points_ = 0, skipped_points_ = 0;
  bool has_labels_ = false;
};

}  // namespace kiss_matcher

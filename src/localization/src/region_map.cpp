#include "localization/region_map.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <vector>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace kiss_matcher {
namespace {
std::string trim(std::string value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  return value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1);
}
std::string replaceAll(std::string value, const std::string &from, const std::string &to) {
  std::size_t position = 0;
  while ((position = value.find(from, position)) != std::string::npos) {
    value.replace(position, from.size(), to); position += to.size();
  }
  return value;
}
std::string normalize(std::string value) {
  value = replaceAll(value, "五号楼", "5号楼");
  value = replaceAll(value, "　", "");
  value.erase(std::remove_if(value.begin(), value.end(), [](unsigned char c) {
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
  }), value.end());
  return value;
}
std::vector<std::string> csvRow(const std::string &line) {
  std::vector<std::string> fields; std::string field; bool quoted = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '"') {
      if (quoted && i + 1 < line.size() && line[i + 1] == '"') { field += '"'; ++i; }
      else quoted = !quoted;
    } else if (c == ',' && !quoted) { fields.push_back(trim(field)); field.clear(); }
    else field += c;
  }
  if (quoted) throw std::runtime_error("unclosed CSV quote");
  fields.push_back(trim(field)); return fields;
}
int integerId(const std::string &value) {
  std::size_t used = 0; long long id;
  try { id = std::stoll(value, &used); }
  catch (...) { throw std::invalid_argument("invalid integer region ID: " + value); }
  if (used != value.size() || id < 0 || id > std::numeric_limits<int>::max())
    throw std::invalid_argument("invalid integer region ID: " + value);
  return static_cast<int>(id);
}
std::map<int, std::string> readLabels(const std::string &path) {
  std::map<int, std::string> result;
  if (path.empty()) return result;
  std::ifstream file(path);
  if (!file) throw std::runtime_error("cannot open region labels CSV: " + path);
  std::string line;
  if (!std::getline(file, line)) throw std::runtime_error("empty region labels CSV");
  if (line.compare(0, 3, "\xef\xbb\xbf") == 0) line.erase(0, 3);
  const auto columns = csvRow(line);
  const auto id = std::find(columns.begin(), columns.end(), "region_id");
  const auto name = std::find(columns.begin(), columns.end(), "region_name");
  if (id == columns.end() || name == columns.end())
    throw std::runtime_error("region CSV requires region_id,region_name columns");
  const auto id_index = static_cast<std::size_t>(id - columns.begin());
  const auto name_index = static_cast<std::size_t>(name - columns.begin());
  std::map<std::string, int> names;
  while (std::getline(file, line)) {
    if (trim(line).empty()) continue;
    const auto fields = csvRow(line);
    if (fields.size() <= std::max(id_index, name_index)) throw std::runtime_error("incomplete region CSV row");
    const int key = integerId(fields[id_index]); const auto label = fields[name_index];
    if (label.empty() || result.count(key) || names.count(normalize(label)))
      throw std::runtime_error("empty or duplicate region ID/name in CSV");
    result.emplace(key, label); names.emplace(normalize(label), key);
  }
  return result;
}
template<typename T> double readScalar(const std::uint8_t *data, bool swap) {
  std::array<std::uint8_t, sizeof(T)> bytes;
  std::memcpy(bytes.data(), data, sizeof(T));
  if (swap) std::reverse(bytes.begin(), bytes.end());
  T value; std::memcpy(&value, bytes.data(), sizeof(T)); return static_cast<double>(value);
}
double number(const std::uint8_t *point, const pcl::PCLPointField &field, bool swap) {
  const auto *data = point + field.offset;
  switch (field.datatype) {
    case pcl::PCLPointField::INT8: return readScalar<std::int8_t>(data, false);
    case pcl::PCLPointField::UINT8: return readScalar<std::uint8_t>(data, false);
    case pcl::PCLPointField::INT16: return readScalar<std::int16_t>(data, swap);
    case pcl::PCLPointField::UINT16: return readScalar<std::uint16_t>(data, swap);
    case pcl::PCLPointField::INT32: return readScalar<std::int32_t>(data, swap);
    case pcl::PCLPointField::UINT32: return readScalar<std::uint32_t>(data, swap);
    case pcl::PCLPointField::FLOAT32: return readScalar<float>(data, swap);
    case pcl::PCLPointField::FLOAT64: return readScalar<double>(data, swap);
    default: throw std::runtime_error("unsupported numeric PCD field type");
  }
}
pcl::PointCloud<pcl::PointXYZI>::Ptr downsample(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, double voxel) {
  auto output = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> filter;
  filter.setLeafSize(voxel, voxel, voxel); filter.setInputCloud(cloud); filter.filter(*output);
  return output;
}
}  // namespace

void RegionMap::load(const std::string &pcd, const std::string &labels_csv, double voxel) {
  if (!std::isfinite(voxel) || voxel <= 0) throw std::invalid_argument("invalid map voxel size");
  auto labels = readLabels(labels_csv);
  pcl::PCLPointCloud2 raw;
  if (pcl::io::loadPCDFile(pcd, raw) < 0 || raw.data.empty())
    throw std::runtime_error("failed to load nonempty global map: " + pcd);
  const auto field = [&](const std::string &name) -> const pcl::PCLPointField * {
    for (const auto &f : raw.fields) if (f.name == name) return &f;
    return nullptr;
  };
  const auto *x = field("x"), *y = field("y"), *z = field("z"), *id_field = field("region_id");
  if (!x || !y || !z) throw std::runtime_error("PCD requires x,y,z fields");
  if (id_field && id_field->count != 1) throw std::runtime_error("region_id must have count=1");
  regions_.clear(); has_labels_ = id_field != nullptr;
  raw_points_ = static_cast<std::size_t>(raw.width) * raw.height; skipped_points_ = 0;
  if (raw.row_step < raw.width * raw.point_step || raw.data.size() < static_cast<std::size_t>(raw.row_step) * raw.height)
    throw std::runtime_error("incomplete PCD data layout");
  const std::uint16_t one = 1; const bool host_big = *reinterpret_cast<const std::uint8_t *>(&one) == 0;
  const bool swap = raw.is_bigendian != host_big;
  auto full = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  full->reserve(raw_points_);
  for (std::size_t row = 0; row < raw.height; ++row) {
    for (std::size_t col = 0; col < raw.width; ++col) {
      const auto *data = raw.data.data() + row * raw.row_step + col * raw.point_step;
      pcl::PointXYZI point;
      point.x = number(data, *x, swap); point.y = number(data, *y, swap); point.z = number(data, *z, swap); point.intensity = 0;
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) { ++skipped_points_; continue; }
      full->push_back(point);
      if (!id_field) continue;
      const double value = number(data, *id_field, swap);
      if (!std::isfinite(value) || value < 0 || value > std::numeric_limits<int>::max() || std::abs(value - std::round(value)) > 1e-4)
        throw std::runtime_error("region_id contains non-integer, negative or non-finite values");
      const int id = static_cast<int>(std::llround(value));
      auto &entry = regions_[id];
      if (!entry.cloud) entry.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
      entry.cloud->push_back(point); ++entry.raw_points;
    }
  }
  if (full->empty()) throw std::runtime_error("map has no finite points");
  full_ = downsample(full, voxel); full.reset();
  for (auto &[id, entry] : regions_) {
    entry.name = labels.count(id) ? labels.at(id) : std::to_string(id);
    entry.cloud = downsample(entry.cloud, voxel);
  }
}

const MapRegion &RegionMap::region(int id) const {
  const auto found = regions_.find(id);
  if (found == regions_.end() || found->second.cloud->empty())
    throw std::invalid_argument("region ID absent or empty in map: " + std::to_string(id));
  return found->second;
}

int RegionMap::resolveHint(const std::string &input) const {
  const auto hint = normalize(input);
  if (hint.empty() || hint == "全图" || hint == "全部" || hint == "清除" || hint == "取消区域" || hint == "all" || hint == "-1") return -1;
  if (!has_labels_) throw std::invalid_argument("map has no region_id field");
  // A destination is not evidence of the sensor's current location.
  for (const auto *word : {"我要去", "前往", "导航到", "目标", "不在", "不是", "离开"})
    if (hint.find(word) != std::string::npos) throw std::invalid_argument("provide CURRENT location, not destination/negation");
  if (hint.rfind("去", 0) == 0) throw std::invalid_argument("destination cannot select current region");
  std::string digits = hint;
  for (const auto *prefix : {"region_id=", "区域ID", "区域"})
    if (digits.rfind(prefix, 0) == 0) { digits.erase(0, std::strlen(prefix)); break; }
  if (!digits.empty() && std::all_of(digits.begin(), digits.end(), [](unsigned char c) { return c >= '0' && c <= '9'; })) {
    const int id = integerId(digits); region(id); return id;
  }
  std::vector<int> matches;
  // Exact region names win before substring matching (space_1 vs space_10).
  for (const auto &[id, entry] : regions_) {
    if (normalize(entry.name) == hint) return id;
  }
  const auto identifier = [](unsigned char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
           (c >= '0' && c <= '9') || c == '_';
  };
  for (const auto &[id, entry] : regions_) {
    const auto label = normalize(entry.name);
    if (label.empty()) continue;
    std::size_t position = hint.find(label);
    while (position != std::string::npos) {
      const auto end = position + label.size();
      const bool left_ok = !identifier(label.front()) || position == 0 || !identifier(hint[position-1]);
      const bool right_ok = !identifier(label.back()) || end == hint.size() || !identifier(hint[end]);
      if (left_ok && right_ok) { matches.push_back(id); break; }
      position = hint.find(label, position + 1);
    }
  }
  if (matches.size() != 1) throw std::invalid_argument("unknown or ambiguous current region; use CSV name or numeric ID");
  const bool exact = normalize(region(matches[0]).name) == hint;
  const bool current = hint.find("现在在") != std::string::npos || hint.find("目前在") != std::string::npos ||
      hint.find("当前在") != std::string::npos || hint.find("雷达在") != std::string::npos ||
      hint.find("当前位置") != std::string::npos || hint.find("现在位于") != std::string::npos || hint.find("目前位于") != std::string::npos;
  if (!exact && !current) throw std::invalid_argument("state current location explicitly, e.g. 我现在在5号楼入口楼梯");
  return matches[0];
}
}  // namespace kiss_matcher

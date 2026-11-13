#include <iostream>
#include "localization/region_map.h"
int main(int argc, char **argv) {
  if (argc < 3 || argc > 4) { std::cerr << "usage: region_map_inspect MAP.pcd LABELS.csv [HINT]\n"; return 2; }
  try {
    kiss_matcher::RegionMap map; map.load(argv[1], argv[2], 0.1);
    std::cout << "MAP raw=" << map.rawPoints() << " voxel=" << map.full()->size() << " labeled=" << map.hasLabels() << '\n';
    for (const auto &[id, r] : map.regions()) std::cout << "REGION id=" << id << " name=" << r.name << " raw=" << r.raw_points << " voxel=" << r.cloud->size() << '\n';
    if (argc == 4) {
      const int id = map.resolveHint(argv[3]); std::cout << "RESOLVED id=" << id << " target=" << (id < 0 ? map.full()->size() : map.region(id).cloud->size()) << '\n';
    }
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}

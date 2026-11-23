#include "localization/region_map.h"
#include <iostream>
int main(int argc,char **argv) {
  if(argc!=5){std::cerr<<"map labels voxel output\n";return 2;}
  try {kiss_matcher::RegionMap map;map.load(argv[1],argv[2],std::stod(argv[3]));map.saveCache(argv[4]);return 0;}
  catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}

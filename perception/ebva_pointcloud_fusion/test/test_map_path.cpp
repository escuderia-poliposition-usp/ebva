#include "ebva_pointcloud_fusion/map_path.hpp"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <unistd.h>

int main()
{
  char directory[] = "/tmp/ebva-map-path-XXXXXX";
  if (mkdtemp(directory) == nullptr) {
    return 1;
  }
  const std::string base = std::string(directory) + "/map.pcd";
  const auto first = ebva_pointcloud_fusion::reserve_map_path(base, 1700000000);
  {
    std::ofstream output(first);
    output << "preserved map";
  }
  const auto second = ebva_pointcloud_fusion::reserve_map_path(base, 1700000000);
  std::ifstream input(first);
  const std::string content((std::istreambuf_iterator<char>(input)), {});
  const bool correct = first != second && content == "preserved map";
  std::remove(first.c_str());
  std::remove(second.c_str());
  rmdir(directory);
  if (!correct) {
    std::cerr << "Saving twice at the same instant replaced the first map" << std::endl;
    return 1;
  }
  return 0;
}

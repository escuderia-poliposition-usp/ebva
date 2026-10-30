#ifndef EBVA_POINTCLOUD_FUSION_MAP_PATH_HPP
#define EBVA_POINTCLOUD_FUSION_MAP_PATH_HPP

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <ctime>
#include <fcntl.h>
#include <string>
#include <system_error>
#include <unistd.h>

namespace ebva_pointcloud_fusion
{
// Reserve a distinct file atomically. A fixed time still yields different paths.
inline std::string reserve_map_path(
  const std::string & path, std::time_t timestamp = std::time(nullptr))
{
  static std::atomic<std::uint64_t> sequence{0};
  std::tm local{};
  if (localtime_r(&timestamp, &local) == nullptr) {
    throw std::system_error(errno, std::generic_category(), "map timestamp");
  }
  char stamp[32];
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &local);
  const auto dot = path.find_last_of('.');
  const auto slash = path.find_last_of('/');
  const bool extension = dot != std::string::npos &&
    (slash == std::string::npos || dot > slash);
  const std::string stem = extension ? path.substr(0, dot) : path;
  const std::string suffix = extension ? path.substr(dot) : "";
  for (;;) {
    const auto candidate = stem + "_" + stamp + "_" +
      std::to_string(getpid()) + "_" + std::to_string(sequence++) + suffix;
    const int descriptor = open(candidate.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0666);
    if (descriptor >= 0) {
      if (close(descriptor) != 0) {
        throw std::system_error(errno, std::generic_category(), "close reserved map");
      }
      return candidate;
    }
    if (errno != EEXIST) {
      throw std::system_error(errno, std::generic_category(), "reserve map " + candidate);
    }
  }
}
}  // namespace ebva_pointcloud_fusion
#endif

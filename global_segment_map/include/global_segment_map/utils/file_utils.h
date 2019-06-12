#ifndef GLOBAL_SEGMENT_MAP_UTILS_FILE_UTILS_H_
#define GLOBAL_SEGMENT_MAP_UTILS_FILE_UTILS_H_

#include <errno.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <cstring>
#include <string>

#include <glog/logging.h>

namespace voxblox {
namespace file_utils {

// Creates the given path, unless the path already exists, in which case
// nothing is done and 0 (success) is returned.
inline int makePath(const std::string& path_in, mode_t mode) {
  size_t previous_slash_pos = 0u;
  size_t current_slash_pos;
  std::string dir;
  int make_dir_status = 0;

  std::string path = path_in;
  CHECK(!path.empty());
  if (path.back() != '/') {
    // Add a trailing '/' so we can handle everything in loop.
    path += '/';
  }

  while ((current_slash_pos = path.find_first_of('/', previous_slash_pos)) !=
         std::string::npos) {
    dir = path.substr(0, current_slash_pos++);
    previous_slash_pos = current_slash_pos;
    if (dir.empty()) continue;  // If leading / first time is 0 length.
    if (dir == ".") continue;

    const char* c_str = dir.c_str();
    for (size_t i = 0u; i < strlen(c_str); ++i) {
      if (c_str[i] < static_cast<char>(32) ||
          c_str[i] > static_cast<char>(126)) {
        return -1;
      }
    }

    if ((make_dir_status = mkdir(c_str, mode)) && errno != EEXIST) {
      return make_dir_status;
    }
  }
  // We return -1 on error above, so just return 0 here.
  return 0;
}

}  // namespace file_utils
}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_UTILS_FILE_UTILS_H_

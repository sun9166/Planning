/******************************************************************************
 * Copyright 2017 The  acu Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "common/util/file.h"

#include <dirent.h>
#include <errno.h>
#include <limits.h>

namespace  acu {
namespace common {
namespace util {

std::string GetAbsolutePath(const std::string &prefix,
                            const std::string &relative_path) {
  if (relative_path.empty()) {
    return prefix;
  }
  // If prefix is empty or relative_path is already absolute.
  if (prefix.empty() || relative_path[0] == '/') {
    return relative_path;
  }

  if (prefix.back() == '/') {
    return StrCat(prefix, relative_path);
  }
  return StrCat(prefix, "/", relative_path);
}

bool PathExists(const std::string &path) {
  struct stat info;
  return stat(path.c_str(), &info) == 0;
}

bool DirectoryExists(const std::string &directory_path) {
  struct stat info;
  if (stat(directory_path.c_str(), &info) != 0) {
    return false;
  }

  if (info.st_mode & S_IFDIR) {
    return true;
  }

  return false;
}

bool EnsureDirectory(const std::string &directory_path) {
  std::string path = directory_path;
  for (size_t i = 1; i < directory_path.size(); ++i) {
    if (directory_path[i] == '/') {
      // Whenever a '/' is encountered, create a temporary view from
      // the start of the path to the character right before this.
      path[i] = 0;

      if (mkdir(path.c_str(), S_IRWXU) != 0) {
        if (errno != EEXIST) {
          return false;
        }
      }

      // Revert the temporary view back to the original.
      path[i] = '/';
    }
  }

  // Make the final (full) directory.
  if (mkdir(path.c_str(), S_IRWXU) != 0) {
    if (errno != EEXIST) {
      return false;
    }
  }

  return true;
}

bool RemoveAllFiles(const std::string &directory_path) {
  DIR *directory = opendir(directory_path.c_str());
  struct dirent *file;
  while ((file = readdir(directory)) != NULL) {
    // skip directory_path/. and directory_path/..
    if (!strcmp(file->d_name, ".") || !strcmp(file->d_name, "..")) {
      continue;
    }
    // build the path for each file in the folder
    std::string file_path = directory_path + "/" + file->d_name;
    if (unlink(file_path.c_str()) < 0) {
      AERROR << "Fail to remove file " << file_path << ": " << strerror(errno);
      closedir(directory);
      return false;
    }
  }
  closedir(directory);
  return true;
}



bool GetFileContent(const std::string& path, std::string* content) {
  if (content == NULL) {
    return false;
  }

  int fd = ::open(path.c_str(), O_RDONLY);
  if (fd < 0) {
    AERROR << "failed to open file: " << path;
    return false;
  }
  struct stat buf;
  if (::fstat(fd, &buf) != 0) {
    AERROR << "failed to lstat file: " << path;
    ::close(fd);
    return false;
  }

  size_t fsize = buf.st_size;
  content->resize(fsize);
  char* data = const_cast<char*>(content->data());
  int size = 0;
  size_t has_read = 0;
  do {
    size = ::read(fd, data + has_read, fsize - has_read);
    if (size < 0) {
      AERROR << "failed to read file: " << path;
      ::close(fd);
      return false;
    }
    has_read += size;
  } while (size > 0);
  ::close(fd);
  return true;
}


}  // namespace util
}  // namespace common
}  // namespace  acu

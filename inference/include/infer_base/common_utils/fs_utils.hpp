#pragma once

#include <iostream>
#include <filesystem>
#include <vector>

namespace easy_deploy {

namespace fs = std::filesystem;

/**
 * @brief Get the absolute path of files in the directory
 *
 * @param directory
 * @return std::vector<fs::path>
 */
inline std::vector<fs::path> get_files_in_directory(const fs::path &directory)
{
  std::vector<fs::path> files;

  // Check whether directory exists
  if (!fs::exists(directory) || !fs::is_directory(directory))
  {
    throw std::runtime_error("Directory does not exist or is not a directory : " +
                             directory.string());
  }

  // Recursively traverse directory
  for (const auto &entry : fs::recursive_directory_iterator(directory))
  {
    if (entry.is_regular_file())
    {
      files.push_back(fs::absolute(entry.path()));
    }
  }

  return files;
}

} // namespace easy_deploy

#pragma once

#include "experimental/filesystem"

namespace mc {
namespace utils {

template <typename T>
void makeDirectoryIfNotExist(const T& path)
{
    namespace fs = std::experimental::filesystem;
    if (!fs::is_directory(path) || !fs::exists(path))
    {
        fs::create_directories(path);
    }
}

template <typename T>
void removeDirectoryIfExist(const T& path)
{
    namespace fs = std::experimental::filesystem;
    if (fs::is_directory(path))
    {
        fs::remove_all(path);
    }
}

}  // namespace utils
}  // namespace mc

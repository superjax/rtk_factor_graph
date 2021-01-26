#include "utils/config.h"

namespace mc {
namespace utils {

Config::Config(const std::string& config_path) : yaml_(YAML::LoadFile(config_path)) {}

Config::Config(const YAML::Node& config) : yaml_(config) {}

}  // namespace utils
}  // namespace mc

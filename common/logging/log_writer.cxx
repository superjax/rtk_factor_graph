#include "common/logging/log_writer.h"

namespace mc {
namespace logging {

Stream::~Stream()
{
    format_["num_records"] = num_records_;
    namespace fs = std::experimental::filesystem;
    auto manifest_path = filename_;
    manifest_path.replace_extension(".yml");
    std::ofstream yml(manifest_path);
    yml << YAML::Dump(format_);
    file_.close();
}

void Logger::open(const std::string& directory)
{
    namespace fs = std::experimental::filesystem;
    log_id_ = createLogId();
    directory_ = fs::path(directory) / fs::path(log_id_);
    utils::makeDirectoryIfNotExist(directory_);
    createManifest();
}

void Logger::close()
{
    writeManifest();
}

void Logger::createManifest()
{
    manifest_.reset();
    manifest_["comment"] = "Project Midnight Compass Log";
    manifest_["id"] = log_id_;

    const UTCTime now = UTCTime::now();
    manifest_["ver"] = fmt::format("{}.{}", VERSION_MAJOR, VERSION_MINOR);
    manifest_["start_time"]["stamp"] = now.str();
    manifest_["start_time"]["sec"] = now.sec;
    manifest_["start_time"]["nsec"] = now.nsec;
}

void Logger::writeManifest()
{
    const UTCTime now = UTCTime::now();
    manifest_["end_time"]["stamp"] = now.str();
    manifest_["end_time"]["sec"] = now.sec;
    manifest_["end_time"]["nsec"] = now.nsec;
    namespace fs = std::experimental::filesystem;
    const auto manifest_path = directory_ / fs::path("manifest.yml");
    std::ofstream os(manifest_path);
    os << YAML::Dump(manifest_);
}

std::string createLogId()
{
    const auto now = UTCTime::now();
    time_t time(now.sec);
    tm* date = gmtime(&time);

    // clang-format on
    return fmt::format("{:0>4}{:0>2}{:0>2}_{:0>2}{:0>2}{:0>2}", 1900 + date->tm_year,
                       1 + date->tm_mon, date->tm_mday, date->tm_hour, date->tm_min, date->tm_sec);
}

}  // namespace logging
}  // namespace mc

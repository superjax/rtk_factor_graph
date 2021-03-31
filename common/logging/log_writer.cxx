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
    error("writing stream {}", fmt(filename_));
    yml << YAML::Dump(format_);
    file_.close();
}

void Logger::open(const UTCTime& start_time, const std::string& directory)
{
    namespace fs = std::experimental::filesystem;
    log_id_ = createLogId(start_time);
    directory_ = fs::path(directory) / fs::path(log_id_);
    utils::removeDirectoryIfExist(directory_);
    utils::makeDirectoryIfNotExist(directory_);
    createManifest(start_time);
}

void Logger::close(const UTCTime& t_end)
{
    while (!streams_.empty())
    {
        streams_.erase(streams_.begin());
    }
    writeManifest(t_end);
}

void Logger::createManifest(const UTCTime& start_time)
{
    manifest_.reset();
    manifest_["comment"] = "Project Midnight Compass Log";
    manifest_["id"] = log_id_;

    manifest_["ver"] = fmt::format("{}.{}", VERSION_MAJOR, VERSION_MINOR);
    manifest_["start_time"]["stamp"] = start_time.str();
    manifest_["start_time"]["sec"] = start_time.sec;
    manifest_["start_time"]["nsec"] = start_time.nsec;
}

void Logger::writeManifest(const UTCTime& t_end)
{
    manifest_["end_time"]["stamp"] = t_end.str();
    manifest_["end_time"]["sec"] = t_end.sec;
    manifest_["end_time"]["nsec"] = t_end.nsec;
    namespace fs = std::experimental::filesystem;
    const auto manifest_path = directory_ / fs::path("manifest.yml");
    std::ofstream os(manifest_path);
    error("writing manifest");
    os << YAML::Dump(manifest_);
}

void Logger::amends(const std::string& other_log)
{
    namespace fs = std::experimental::filesystem;
    const auto other_log_path = fs::path(other_log);

    manifest_["amends"] = fs::canonical(other_log_path).string();
}

std::string createLogId(const UTCTime& start_time)
{
    time_t time(start_time.sec);
    tm* date = gmtime(&time);

    // clang-format on
    return fmt::format("{:0>4}{:0>2}{:0>2}_{:0>2}{:0>2}{:0>2}", 1900 + date->tm_year,
                       1 + date->tm_mon, date->tm_mday, date->tm_hour, date->tm_min, date->tm_sec);
}

}  // namespace logging
}  // namespace mc

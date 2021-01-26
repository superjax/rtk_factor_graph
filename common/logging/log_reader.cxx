#include "common/logging/log_reader.h"

#include <yaml-cpp/yaml.h>

#include "common/ephemeris/galileo.h"
#include "common/ephemeris/glonass.h"
#include "common/ephemeris/gps.h"
#include "common/logging/log_key.h"
#include "common/math/dquat.h"
#include "common/math/jet.h"
#include "common/math/quat.h"
#include "common/math/two_jet.h"
#include "common/measurements/gnss_observation.h"
#include "common/measurements/imu.h"
#include "common/utctime.h"

namespace mc {
namespace logging {

StreamReader::StreamReader(const std::string& filename)
{
    open(filename);
}

StreamReader::~StreamReader()
{
    if (file_.is_open())
    {
        file_.close();
    }
}

Error StreamReader::open(const std::string& filename)
{
    namespace fs = std::experimental::filesystem;
    filename_ = filename;
    if (!fs::exists(filename_))
    {
        error("Unable to find log file {} for reading", fmt(filename_));
        return Error::create("Unable to find file");
    }
    file_.open(filename_);
    if (!file_.is_open())
    {
        error("Unable to open log file {}", fmt(filename_));
        return Error::create("Unable to open file");
    }

    auto format_path = fs::path(filename_).replace_extension(".yml");
    YAML::Node yaml = YAML::LoadFile(format_path);

    key_ = yaml["key"].as<int>();

    readStamp();
    return Error::none();
}

bool StreamReader::done()
{
    return file_.eof();
}

LogReader::LogReader(const std::string& directory)
{
    open(directory);
}

void LogReader::open(const std::string& directory, bool quiet)
{
    namespace fs = std::experimental::filesystem;
    directory_ = fs::path(directory);
    const auto manifest_path = directory_ / "manifest.yml";
    YAML::Node node = YAML::LoadFile(manifest_path);

    log_id_ = node["id"].as<std::string>();

    start_time_ =
        UTCTime(node["start_time"]["sec"].as<int64_t>(), node["start_time"]["nsec"].as<int64_t>());
    end_time_ =
        UTCTime(node["end_time"]["sec"].as<int64_t>(), node["end_time"]["nsec"].as<int64_t>());

    const std::string version = node["ver"].as<std::string>();

    for (const auto key : node["keys"].as<std::vector<int>>())
    {
        const std::string stream_data_file = directory_ / fs::path(logKeyName(key) + ".log");
        streams_.emplace(key, stream_data_file);
    }

    if (!quiet)
    {
        fmt::print("Loading log      {}\n", directory_);
        fmt::print("    time:        ({}) - ({})\n", start_time_, end_time_);
        fmt::print("    version:     {}\n", version);
        fmt::print("    contents:\n");
        for (const auto& stream : streams_)
        {
            fmt::print("               - {}: {}\n", stream.first, logKeyName(stream.first));
        }
        fmt::print("-----------------------------------\n\n");
    }
}

void LogReader::setCallback(int key, const MsgCallback&& fn)
{
    check(callbacks_.find(key) == callbacks_.end(),
          "Cannot assign multiple callbacks to the same key");
    callbacks_[key] = std::move(fn);
}

void LogReader::read()
{
    // Remove unsubscribed streams
    for (auto it = streams_.begin(); it != streams_.end();)
    {
        if (callbacks_.find(it->first) == callbacks_.end())
        {
            it = streams_.erase(it);
        }
        else
        {
            ++it;
        }
    }

    // Go through all streams and call messages as they occur.
    while (true)
    {
        UTCTime min_time = MAX_TIME;
        auto lowest_it = streams_.end();
        for (auto stream = streams_.begin(); stream != streams_.end(); ++stream)
        {
            const auto& msg_time = stream->second.nextStamp();
            if (msg_time < min_time)
            {
                lowest_it = stream;
                min_time = msg_time;
            }
        }

        // Done
        if (min_time == MAX_TIME || lowest_it == streams_.end())
        {
            return;
        }

        const auto cb = callbacks_.find(lowest_it->second.key());
        if (cb != callbacks_.end())
        {
            cb->second(min_time, lowest_it->second.key(), lowest_it->second);
        }
    }
}

}  // namespace logging
}  // namespace mc

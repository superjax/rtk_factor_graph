#include "common/logging/log_reader.h"

#include <yaml-cpp/yaml.h>

#include <regex>

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
#include "common/quantized_time.h"
#include "common/utctime.h"
#include "utils/split_string.h"

namespace fs = std::experimental::filesystem;

namespace mc {
namespace logging {

bool isLogId(const std::string& maybe_log_id)
{
    std::regex re("\\d{8}_\\d{6}");
    return std::regex_match(maybe_log_id, re);
}

fs::path getLatestLog(const fs::path& path)
{
    if (isLogId(path.stem()))
    {
        return path;
    }

    std::vector<std::string> options;
    for (const auto& entry : fs::directory_iterator(path))
    {
        const auto split = utils::split_string(entry.path().string(), '/');
        if (isLogId(entry.path().stem()))
        {
            options.push_back(entry.path());
        }
    }
    std::sort(options.begin(), options.end());
    return fs::path(options.back());
}

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
    num_records_ = yaml["num_records"].as<int>();

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

void LogReader::openLog(const fs::path& directory)
{
    namespace fs = std::experimental::filesystem;

    if (!fs::exists(directory))
    {
        error("unable to find base log {}", fmt(directory));
        check(fs::exists(directory), "Unable to find base log {}", fmt(directory));
    }

    const auto manifest_path = directory / "manifest.yml";
    if (!fs::exists(manifest_path))
    {
        error("Unable to find {}", fmt(manifest_path));
        check(fs::exists(manifest_path), "Unable to find {}", fmt(manifest_path));
    }

    YAML::Node node = YAML::LoadFile(manifest_path);
    const auto log_start_time =
        UTCTime(node["start_time"]["sec"].as<int64_t>(), node["start_time"]["nsec"].as<int64_t>());
    const auto log_end_time =
        UTCTime(node["end_time"]["sec"].as<int64_t>(), node["end_time"]["nsec"].as<int64_t>());

    start_time_ = std::min(start_time_, log_start_time);
    end_time_ = std::max(end_time_, log_end_time);

    for (const auto key : node["keys"].as<std::vector<int>>())
    {
        const std::string stream_data_file = directory / fs::path(logKeyName(key) + ".log");
        streams_.emplace(key, stream_data_file);
    }

    version_ = node["ver"].as<std::string>();

    if (node["amends"])
    {
        amends_.push_back(node["amends"].as<std::string>());
        openLog(fs::path(node["amends"].as<std::string>()));
    }
}

void LogReader::open(const std::string& directory, bool quiet)
{
    namespace fs = std::experimental::filesystem;
    directory_ = getLatestLog(fs::path(directory));

    openLog(directory_);

    if (!quiet)
    {
        fmt::print("Loading log      {}\n", directory_);
        for (const auto& id : amends_)
        {
            fmt::print("Amends           {}\n", id);
        }
        fmt::print("    time:        ({}) - ({})\n", start_time_, end_time_);
        fmt::print("    version:     {}\n", version_);
        fmt::print("    contents:\n");
        for (const auto& stream : streams_)
        {
            fmt::print("               - {}: {} #{}\n", stream.first, logKeyName(stream.first),
                       stream.second.numRecords());
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

        // Call maintenance callbacks, if any
        for (auto& maint_cb : maintenance_callbacks_)
        {
            if (min_time >= maint_cb.next_call.quantized())
            {
                Error exit_early = maint_cb.cb(min_time);
                maint_cb.next_call += maint_cb.period_s;

                if (!exit_early.ok())
                {
                    info("exiting read early due to maintenance cb return {}",
                         fmt(exit_early.what()));
                    return;
                }
            }
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

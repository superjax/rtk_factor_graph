
#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <experimental/filesystem>
#include <fstream>
#include <string>
#include <type_traits>
#include <unordered_map>

#include "common/logging/log_format.h"
#include "common/logging/log_key.h"
#include "common/logging/log_reader.h"
#include "common/logging/serialize.h"
#include "common/print.h"
#include "common/utctime.h"
#include "utils/file.h"
#include "utils/has_data.h"
#include "utils/non_copyable.h"

namespace mc {

namespace logging {

static constexpr int VERSION_MAJOR = 3;
static constexpr int VERSION_MINOR = 0;

std::string createLogId(const UTCTime& start_time);

class Stream : public utils::NonCopyable
{
 public:
    ~Stream();

    template <typename... T>
    void init(const std::experimental::filesystem::path& directory,
              const std::string& log_id,
              int key,
              const std::array<std::string, sizeof...(T)>& names)
    {
        namespace fs = std::experimental::filesystem;
        format_["comment"] = "Project Midnight Compass Log Stream";
        format_["id"] = log_id;
        format_["key"] = key;
        format_["key_string"] = logKeyName(key);

        filename_ = directory / fs::path(logKeyName(key) + ".log");
        file_.open(filename_.string());
        format_["format"] = makeFormat<T...>(names);
    }

 private:
    int num_records_ = 0;
    std::experimental::filesystem::path filename_;
    std::ofstream file_;
    YAML::Node format_;

    friend class Logger;
};

class Logger
{
 public:
    Logger(const std::string directory) { open(UTCTime::now(), directory); }
    Logger(const UTCTime& start_time, const std::string directory) { open(start_time, directory); }

    ~Logger() { close(UTCTime::now()); }

    void open(const std::string& directory) { open(UTCTime::now(), directory); }
    void open(const UTCTime& start_time, const std::string& directory);

    void close(const UTCTime& t_end);

    void amends(const std::string& other_log);

    void createManifest(const UTCTime& start_time);
    void writeManifest(const UTCTime& t_end);

    std::string logId() { return log_id_; }

    template <typename... T>
    void initStream(int key, const std::array<std::string, sizeof...(T)>& names)
    {
        auto topic_it = streams_.find(key);
        check(topic_it == streams_.end(), "Cannot re-initialize log stream");
        auto it =
            streams_.emplace(std::piecewise_construct, std::make_tuple(key), std::make_tuple());
        it.first->second.init<T...>(directory_, log_id_, key, names);
        manifest_["keys"].push_back(key);
        manifest_["key_names"].push_back(logKeyName(key));
    }

    template <typename... T>
    void logStamped(int key, T... data)
    {
        const UTCTime& now = UTCTime::now();
        log(key, now, data...);
    }

    template <typename... T>
    void log(int key, const UTCTime& t, T... data)
    {
        auto topic_it = streams_.find(key);
        check(topic_it != streams_.end(), "must initialize stream {} before logging",
              fmt(logKeyName(key)));

        std::ostream& file = topic_it->second.file_;
        serialize(file, t);
        int dummy[sizeof...(data)] = {(serialize(file, data), 1)...};
        (void)dummy;
        ++topic_it->second.num_records_;
    }

 private:
    std::experimental::filesystem::path directory_;
    YAML::Node manifest_;
    std::string log_id_;
    std::unordered_map<int, Stream> streams_;
};

}  // namespace logging
}  // namespace mc

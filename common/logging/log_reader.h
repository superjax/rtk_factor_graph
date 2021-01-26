#pragma once

#include <experimental/filesystem>
#include <fstream>
#include <set>
#include <string>
#include <unordered_map>

#include "common/error.h"
#include "common/logging/serialize.h"
#include "common/out.h"
#include "common/print.h"
#include "common/utctime.h"
#include "utils/has_data.h"
#include "utils/non_copyable.h"

namespace mc {
namespace logging {

class StreamReader : public utils::NonCopyable
{
 public:
    StreamReader(const std::string& filename);
    ~StreamReader();

    const UTCTime& nextStamp() const { return next_stamp_; }

    Error open(const std::string& filename);

    template <typename... T>
    void get(T&... x)
    {
        read(file_, x...);
        readStamp();
    }

    bool done();

    int key() { return key_; }

 private:
    void readStamp()
    {
        deserialize(file_, next_stamp_.sec);
        deserialize(file_, next_stamp_.nsec);
        if (file_.eof())
        {
            next_stamp_ = MAX_TIME;
        }
    }

    std::string filename_ = "";
    std::ifstream file_;
    UTCTime next_stamp_;
    int key_;
};

class LogReader
{
 public:
    LogReader(const std::string& directory);
    void open(const std::string& directory, bool quiet = false);

    void logId();
    std::set<int> keys();

    using MsgCallback = std::function<void(const UTCTime&, int, StreamReader&)>;

    void read();

    void setCallback(int key, const MsgCallback&& fn);

    UTCTime startTime() { return start_time_; }
    UTCTime endTime() { return end_time_; }

 private:
    std::unordered_map<int, MsgCallback> callbacks_;
    UTCTime start_time_;
    UTCTime end_time_;
    std::string log_id_;
    std::experimental::filesystem::path directory_;
    std::unordered_map<int, StreamReader> streams_;
};

}  // namespace logging
}  // namespace mc

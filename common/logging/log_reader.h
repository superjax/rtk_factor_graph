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

    int key() const { return key_; }

    int numRecords() const { return num_records_; }

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
    int num_records_;
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

    template <typename... T>
    struct Entry
    {
        UTCTime t;
        std::tuple<T...> data;
    };

    template <typename... T>
    Entry<T...> getNext(int key)
    {
        auto& stream = streams_.at(key);
        Entry<T...> out;
        out.t = stream.nextStamp();
        std::apply([&](auto&... args) { stream.get(args...); }, out.data);
        return out;
    }

    UTCTime startTime() { return start_time_; }
    UTCTime endTime() { return end_time_; }

    const std::string logPath() const { return directory_.string(); }

 private:
    void openLog(const std::experimental::filesystem::path& directory);

    std::unordered_map<int, MsgCallback> callbacks_;

    struct MaintenanceCallbackState
    {
        const double period_s;
        std::function<Error(const UTCTime&)> cb;
        UTCTime next_call;
    };

 public:
    void setMaintenanceCallback(const double period_s, std::function<Error(const UTCTime&)> cb)
    {
        check(start_time_ != UTCTime::min());
        maintenance_callbacks_.push_back(MaintenanceCallbackState{period_s, cb, start_time_});
    }

 private:
    std::vector<MaintenanceCallbackState> maintenance_callbacks_;
    UTCTime start_time_ = UTCTime::max();
    UTCTime end_time_ = UTCTime::min();
    std::string log_id_;
    std::experimental::filesystem::path directory_;
    std::unordered_map<int, StreamReader> streams_;
    std::string version_;
    std::vector<std::string> amends_;
};

}  // namespace logging
}  // namespace mc

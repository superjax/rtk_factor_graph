
#pragma once
#include <Eigen/Core>
#include <experimental/filesystem>
#include <fstream>
#include <string>
#include <type_traits>

#include "common/print.h"
#include "common/utctime.h"
#include "utils/file.h"

namespace mc {

// Forward declaration for special serialize types
class UTCTime;
namespace ephemeris {
class GPSEphemeris;
class GlonassEphemeris;
class GalileoEphemeris;
}  // namespace ephemeris
namespace meas {
class ImuSample;
class GnssObservation;
}  // namespace meas
namespace math {
template <typename T>
class Quat;
template <typename T>
class DQuat;
template <typename T>
class Jet;
}  // namespace math

namespace logging {

namespace detail {

template <typename T>
struct has_data
{
 private:
    template <typename U>
    static auto test(int) -> decltype(std::declval<U>().data(), std::true_type());

    template <typename>
    static std::false_type test(...);

 public:
    static constexpr bool value = std::is_same<decltype(test<T>(0)), std::true_type>::value;
};

}  // namespace detail

class Logger
{
 public:
    Logger(const std::string filename) { open(filename); }

    void open(const std::string& filename)
    {
        namespace fs = std::experimental::filesystem;
        filename_ = filename;
        fs::path directory = fs::path(filename).parent_path();
        utils::makeDirectoryIfNotExist(directory);
        file_.open(filename);
    }
    void close() { file_.close(); }

    void addHeader(const std::string& header)
    {
        namespace fs = std::experimental::filesystem;
        const auto log_path = fs::path(filename_);
        const std::string header_filename =
            (log_path.parent_path() / log_path.stem()).string() + ".hdr";
        if (fs::exists(fs::path(header_filename)))
        {
            fs::remove(fs::path(header_filename));
        }
        std::ofstream header_file(header_filename);
        const auto start_time = UTCTime::now();

        header_file << "{\n";
        header_file << "\"_comment\": \"Project Midnight Compass Header: v1.0\",\n";
        header_file << "\"_start_time\": \"" << start_time << "\",\n";
        header_file << "\"file\": \"" << filename_ << "\",\n";
        header_file << "\"time\": { \n";
        header_file << "  \"sec\": " << start_time.sec << ",\n";
        header_file << "  \"nsec\": " << start_time.nsec << "\n";
        header_file << "},\n";
        header_file << "\"format\": {\n";
        header_file << header;
        header_file << "}\n";
        header_file << "}\n";
    }

    ~Logger() { file_.close(); }

    template <typename... T>
    void log(T... data)
    {
        int dummy[sizeof...(data)] = {(serialize(data), 1)...};
        (void)dummy;
    }

 private:
    template <typename T>
    void serialize(const T& data)
    {
        if constexpr (detail::has_data<T>::value)
        {
            file_.write((const char*)data.data(), sizeof(decltype(*data.data())) * data.size());
        }
        else
        {
            file_.write((const char*)&data, sizeof(T));
        }
    }

    void serialize(const UTCTime& t);
    void serialize(const ephemeris::GPSEphemeris& eph);
    void serialize(const ephemeris::GlonassEphemeris& eph);
    void serialize(const ephemeris::GalileoEphemeris& eph);
    void serialize(const meas::ImuSample& imu);
    void serialize(const meas::GnssObservation& obs);
    void serialize(const math::Quat<double>& q);
    void serialize(const math::DQuat<double>& dq);
    void serialize(const math::Jet<double>& x);

    std::string filename_ = "";
    std::ofstream file_;
};

}  // namespace logging
}  // namespace mc

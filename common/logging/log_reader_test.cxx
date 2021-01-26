#include "common/logging/log_reader.h"

#include <gtest/gtest.h>

#include "common/logging/log_writer.h"
#include "common/measurements/gnss_observation.h"
#include "common/measurements/imu.h"
#include "common/test_helpers.h"

namespace mc {
namespace logging {

namespace fs = std::experimental::filesystem;

static const std::string log_directory = "/tmp/log_reader_test";

TEST(LogReaderTest, OneTopic)
{
    UTCTime start_time;
    std::string log_id;
    {
        Logger lw(log_directory);
        lw.initStream<meas::ImuSample>(LogKey::IMU_SAMPLE, {"imu"});

        for (int i = 0; i < 10; i++)
        {
            meas::ImuSample imu;
            imu.t = UTCTime(i);
            imu.accel.setConstant(i);
            imu.gyro.setConstant(i + 1);

            lw.log(LogKey::IMU_SAMPLE, UTCTime(10 + i), imu);
        }

        log_id = lw.logId();
    }

    UTCTime last_stamp = MIN_TIME;
    int i = 0;
    auto imu_cb = [&](const UTCTime& t, int key, StreamReader& reader) {
        meas::ImuSample imu;
        reader.get(imu);
        EXPECT_EQ(t, UTCTime(i + 10));
        EXPECT_EQ(key, LogKey::IMU_SAMPLE);
        EXPECT_EQ(imu.t, UTCTime(i));
        MATRIX_CLOSE(imu.accel, Vec3::Constant(i), 1e-15);
        MATRIX_CLOSE(imu.gyro, Vec3::Constant(i + 1), 1e-15);
        EXPECT_LE(last_stamp, t);
        last_stamp = t;
        i++;
    };
    LogReader lr(log_directory + "/" + log_id);

    lr.setCallback(LogKey::IMU_SAMPLE, std::move(imu_cb));

    lr.read();

    EXPECT_EQ(i, 10);
}

TEST(LogReaderTest, TwoTopics)
{
    UTCTime start_time;
    std::string log_id;
    {
        Logger lw(log_directory);
        lw.initStream<meas::ImuSample>(LogKey::IMU_SAMPLE, {"imu"});
        lw.initStream<meas::GnssObservation>(LogKey::GNSS_OBS, {"obs"});

        for (int i = 0; i < 10; i++)
        {
            meas::ImuSample imu;
            imu.t = UTCTime(i);
            imu.accel.setConstant(i);
            imu.gyro.setConstant(i + 1);

            lw.log(LogKey::IMU_SAMPLE, UTCTime(10 + i), imu);

            meas::GnssObservation obs;
            obs.t = UTCTime(i + 20);
            obs.gnss_id = 3 * i;
            obs.pseudorange = i * 1000;
            obs.doppler = i * 92;
            obs.carrier_phase = i * 129;

            lw.log(LogKey::GNSS_OBS, UTCTime(10.2 + i), obs);
        }

        log_id = lw.logId();
    }

    UTCTime last_stamp = MIN_TIME;
    int imu_cb_calls = 0;
    int obs_cb_calls = 0;
    int i = 0;
    auto imu_cb = [&](const UTCTime& t, int key, StreamReader& reader) {
        meas::ImuSample imu;
        reader.get(imu);
        EXPECT_EQ(t, UTCTime(i + 10));
        EXPECT_EQ(key, LogKey::IMU_SAMPLE);
        EXPECT_EQ(imu.t, UTCTime(i));
        MATRIX_CLOSE(imu.accel, Vec3::Constant(i), 1e-15);
        MATRIX_CLOSE(imu.gyro, Vec3::Constant(i + 1), 1e-15);
        EXPECT_LE(last_stamp, t);
        last_stamp = t;
        imu_cb_calls++;
    };

    auto obs_cb = [&](const UTCTime& t, int key, StreamReader& reader) {
        meas::GnssObservation obs;
        reader.get(obs);

        EXPECT_EQ(t, UTCTime(i + 10.2));
        EXPECT_EQ(key, LogKey::GNSS_OBS);
        EXPECT_EQ(obs.t, UTCTime(i + 20));
        EXPECT_LE(last_stamp, t);
        last_stamp = t;
        obs_cb_calls++;
        ++i;
    };

    LogReader lr(log_directory + "/" + log_id);

    lr.setCallback(LogKey::IMU_SAMPLE, std::move(imu_cb));
    lr.setCallback(LogKey::GNSS_OBS, std::move(obs_cb));

    lr.read();

    EXPECT_EQ(i, 10);
    EXPECT_EQ(imu_cb_calls, 10);
    EXPECT_EQ(obs_cb_calls, 10);
}

TEST(LogReaderTest, EmptyTopic)
{
    UTCTime start_time;
    std::string log_id;
    {
        Logger lw(log_directory);
        lw.initStream<meas::ImuSample>(LogKey::IMU_SAMPLE, {"imu"});
        lw.initStream<meas::GnssObservation>(LogKey::GNSS_OBS, {"obs"});

        for (int i = 0; i < 10; i++)
        {
            meas::ImuSample imu;
            imu.t = UTCTime(i);
            imu.accel.setConstant(i);
            imu.gyro.setConstant(i + 1);

            lw.log(LogKey::IMU_SAMPLE, UTCTime(10 + i), imu);
        }

        log_id = lw.logId();
    }

    UTCTime last_stamp = MIN_TIME;
    int i = 0;
    auto imu_cb = [&](const UTCTime& t, int key, StreamReader& reader) {
        meas::ImuSample imu;
        reader.get(imu);
        EXPECT_EQ(t, UTCTime(i + 10));
        EXPECT_EQ(key, LogKey::IMU_SAMPLE);
        EXPECT_EQ(imu.t, UTCTime(i));
        MATRIX_CLOSE(imu.accel, Vec3::Constant(i), 1e-15);
        MATRIX_CLOSE(imu.gyro, Vec3::Constant(i + 1), 1e-15);
        EXPECT_LE(last_stamp, t);
        last_stamp = t;
        i++;
    };

    LogReader lr(log_directory + "/" + log_id);

    lr.setCallback(LogKey::IMU_SAMPLE, std::move(imu_cb));

    lr.read();

    EXPECT_EQ(i, 10);
}

TEST(LogReaderTest, UnsubscribedTopics)
{
    UTCTime start_time;
    std::string log_id;
    {
        Logger lw(log_directory);
        lw.initStream<meas::ImuSample>(LogKey::IMU_SAMPLE, {"imu"});
        lw.initStream<meas::GnssObservation>(LogKey::GNSS_OBS, {"obs"});

        for (int i = 0; i < 10; i++)
        {
            meas::ImuSample imu;
            imu.t = UTCTime(i);
            imu.accel.setConstant(i);
            imu.gyro.setConstant(i + 1);

            lw.log(LogKey::IMU_SAMPLE, UTCTime(10 + i), imu);

            meas::GnssObservation obs;
            obs.t = UTCTime(i + 20);
            obs.gnss_id = 3 * i;
            obs.pseudorange = i * 1000;
            obs.doppler = i * 92;
            obs.carrier_phase = i * 129;

            lw.log(LogKey::GNSS_OBS, UTCTime(10.2 + i), obs);
        }

        log_id = lw.logId();
    }

    UTCTime last_stamp = MIN_TIME;
    int i = 0;
    auto imu_cb = [&](const UTCTime& t, int key, StreamReader& reader) {
        meas::ImuSample imu;
        reader.get(imu);
        EXPECT_EQ(t, UTCTime(i + 10));
        EXPECT_EQ(key, LogKey::IMU_SAMPLE);
        EXPECT_EQ(imu.t, UTCTime(i));
        MATRIX_CLOSE(imu.accel, Vec3::Constant(i), 1e-15);
        MATRIX_CLOSE(imu.gyro, Vec3::Constant(i + 1), 1e-15);
        EXPECT_LE(last_stamp, t);
        last_stamp = t;
        i++;
    };

    LogReader lr(log_directory + "/" + log_id);

    lr.setCallback(LogKey::IMU_SAMPLE, std::move(imu_cb));

    lr.read();

    EXPECT_EQ(i, 10);
}

}  // namespace logging
}  // namespace mc

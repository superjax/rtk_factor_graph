#include "common/logging/serialize.h"

#include <gtest/gtest.h>

#include <experimental/filesystem>
#include <fstream>

#include "common/ephemeris/galileo.h"
#include "common/ephemeris/glonass.h"
#include "common/ephemeris/gps.h"
#include "common/math/dquat.h"
#include "common/math/jet.h"
#include "common/math/quat.h"
#include "common/math/two_jet.h"
#include "common/measurements/gnss_observation.h"
#include "common/measurements/imu.h"
#include "common/utctime.h"

namespace mc {

namespace meas {
bool operator==(const GnssObservation& a, const GnssObservation& b)
{
    return (a.t == b.t) && (a.gnss_id == b.gnss_id) && (a.sat_num == b.sat_num) &&
           (a.freq == b.freq) && (a.pseudorange == b.pseudorange) && (a.doppler == b.doppler) &&
           (a.carrier_phase == b.carrier_phase);
}

bool operator==(const ImuSample& a, const ImuSample& b)
{
    return (a.t == b.t) && (a.accel == b.accel) && (a.gyro == b.gyro);
}
}  // namespace meas

namespace ephemeris {

bool operator==(const GPSEphemeris& a, const GPSEphemeris& b)
{
    return (a.gnssID == b.gnssID) && (a.sat == b.sat) && (a.toe == b.toe) && (a.toc == b.toc) &&
           (a.tow == b.tow) && (a.iodc == b.iodc) && (a.iode == b.iode) && (a.week == b.week) &&
           (a.toes == b.toes) && (a.tocs == b.tocs) && (a.af2 == b.af2) && (a.af1 == b.af1) &&
           (a.af0 == b.af0) && (a.m0 == b.m0) && (a.delta_n == b.delta_n) && (a.ecc == b.ecc) &&
           (a.sqrta == b.sqrta) && (a.omega0 == b.omega0) && (a.i0 == b.i0) && (a.w == b.w) &&
           (a.omegadot == b.omegadot) && (a.idot == b.idot) && (a.cuc == b.cuc) &&
           (a.cus == b.cus) && (a.crc == b.crc) && (a.crs == b.crs) && (a.cic == b.cic) &&
           (a.cis == b.cis) && (a.health == b.health) && (a.alert_flag == b.alert_flag) &&
           (a.anti_spoof == b.anti_spoof) && (a.code_on_L2 == b.code_on_L2) && (a.ura == b.ura) &&
           (a.L2_P_data_flag == b.L2_P_data_flag) && (a.fit_interval_flag == b.fit_interval_flag) &&
           (a.age_of_data_offset == b.age_of_data_offset) && (a.tgd == b.tgd);
}
bool operator==(const GlonassEphemeris& a, const GlonassEphemeris& b)
{
    return (a.gnssID == b.gnssID) && (a.sat == b.sat) && (a.toe == b.toe) && (a.iode == b.iode) &&
           (a.slot == b.slot) && (a.svh == b.svh) && (a.sva == b.sva) && (a.age == b.age) &&
           (a.tof == b.tof) && (a.pos == b.pos) && (a.vel == b.vel) && (a.acc == b.acc) &&
           (a.taun == b.taun) && (a.gamn == b.gamn) && (a.dtaun == b.dtaun);
}
bool operator==(const GalileoEphemeris& a, const GalileoEphemeris& b)
{
    return (a.gnssID == b.gnssID) && (a.sat == b.sat) && (a.toe == b.toe) && (a.toc == b.toc) &&
           (a.tow == b.tow) && (a.iodc == b.iodc) && (a.iode == b.iode) && (a.week == b.week) &&
           (a.toes == b.toes) && (a.tocs == b.tocs) && (a.af2 == b.af2) && (a.af1 == b.af1) &&
           (a.af0 == b.af0) && (a.m0 == b.m0) && (a.delta_n == b.delta_n) && (a.ecc == b.ecc) &&
           (a.sqrta == b.sqrta) && (a.omega0 == b.omega0) && (a.i0 == b.i0) && (a.w == b.w) &&
           (a.omegadot == b.omegadot) && (a.idot == b.idot) && (a.cuc == b.cuc) &&
           (a.cus == b.cus) && (a.crc == b.crc) && (a.crs == b.crs) && (a.cic == b.cic) &&
           (a.cis == b.cis) && (a.ai0 == b.ai0) && (a.ai1 == b.ai1) && (a.ai2 == b.ai2) &&
           (a.bgd_e1_e5a == b.bgd_e1_e5a) && (a.bgd_e1_e5b == b.bgd_e1_e5b) &&
           (a.e5b_hs == b.e5b_hs) && (a.e1b_hs == b.e1b_hs) && (a.e5b_dvs == b.e5b_dvs) &&
           (a.e1b_dvs == b.e1b_dvs);
}

}  // namespace ephemeris

namespace logging {

namespace fs = std::experimental::filesystem;

class TempFile
{
 public:
    TempFile()
    {
        filename_ = fs::path(testing::TempDir()) / (std::to_string(rand()) + "_config_test.yaml");
    }
    ~TempFile()
    {
        // Delete the temporary file
        if (fs::exists(filename_))
        {
            fs::remove(filename_);
        }
    }
    std::string filename_;
};

template <typename T>
class SerializeTest : public ::testing::Test
{
};

// The list of types we want to test.
typedef ::testing::Types<UTCTime,
                         Vec3,
                         Vec3f,
                         Mat96,
                         Mat10f,
                         meas::ImuSample,
                         meas::GnssObservation,
                         ephemeris::GPSEphemeris,
                         ephemeris::GlonassEphemeris,
                         ephemeris::GalileoEphemeris>
    TestTypes;

TYPED_TEST_CASE(SerializeTest, TestTypes);

TYPED_TEST(SerializeTest, ToAndFrom)
{
    TempFile tmp;
    // typename TypeParam::MyA someA;  // if you need access to the subtypes in the test itself
    TypeParam x = TypeParam::Random();
    {
        std::ofstream file(tmp.filename_);
        serialize(file, x);
    }

    std::ifstream file(tmp.filename_);
    TypeParam out;
    deserialize(file, out);

    EXPECT_EQ(x, out);
}

}  // namespace logging
}  // namespace mc

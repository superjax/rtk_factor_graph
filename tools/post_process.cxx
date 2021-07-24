#include "common/defs.h"
#include "common/logging/log_key.h"
#include "common/logging/log_reader.h"
#include "common/logging/log_writer.h"
#include "common/math/two_jet.h"
#include "common/measurements/gnss_observation.h"
#include "common/measurements/imu.h"
#include "core/ekf/rtk_ekf.h"
#include "core/ekf/rtk_ekf_estimator.h"
#include "third_party/argparse/argparse.hpp"
#include "utils/config.h"
#include "utils/wgs84.h"

using namespace mc;
using namespace ekf;
using namespace third_party::argparse;

ProcessCovariance getStateCovariance(const std::string& key, utils::Config& cfg)
{
    ProcessCovariance stdev;
    // Vec3 trans, rot, vel, acc_bias, gyro_bias, p_b2g, rot_I2e, trans_I2e;
    // Vec2 gps_clk, gal_clk, glo_clk;
    // double sd;
    cfg.get(key + "/rotation", make_out(stdev.pose.head<3>()), true);
    cfg.get(key + "/translation", make_out(stdev.pose.tail<3>()), true);
    cfg.get(key + "/vel", make_out(stdev.vel), true);
    cfg.get(key + "/acc_bias", make_out(stdev.accBias), true);
    cfg.get(key + "/gyro_bias", make_out(stdev.gyroBias), true);
    cfg.get(key + "/gps_clk", make_out(stdev.gpsClk), true);
    cfg.get(key + "/gal_clk", make_out(stdev.galClk), true);
    cfg.get(key + "/glo_clk", make_out(stdev.gloClk), true);
    cfg.get(key + "/p_b2g", make_out(stdev.pB2g), true);
    cfg.get(key + "/T_I2e", make_out(stdev.TI2e), true);
    cfg.get(key + "/sd", make_out(stdev.sd), true);

    ProcessCovariance cov;
    cov.pose = stdev.pose.cwiseProduct(stdev.pose);
    cov.vel = stdev.vel.cwiseProduct(stdev.vel);
    cov.accBias = stdev.accBias.cwiseProduct(stdev.accBias);
    cov.gyroBias = stdev.gyroBias.cwiseProduct(stdev.gyroBias);
    cov.gpsClk = stdev.gpsClk.cwiseProduct(stdev.gpsClk);
    cov.galClk = stdev.galClk.cwiseProduct(stdev.galClk);
    cov.gloClk = stdev.gloClk.cwiseProduct(stdev.gloClk);
    cov.pB2g = stdev.pB2g.cwiseProduct(stdev.pB2g);
    cov.TI2e = stdev.TI2e.cwiseProduct(stdev.TI2e);
    cov.sd = stdev.sd.cwiseProduct(stdev.sd);

    return cov;
}

Input getInput(utils::Config& cfg)
{
    Input u;
    u.accel << 0, 0, -9.80665;
    u.gyro.setZero();
    return u;
}

std::pair<State, Input> getState(utils::Config& cfg, logging::LogReader& log)
{
    State x;
    Input u;
    UTCTime t;

    Vec3 translation = Vec3::Zero();
    math::Quat<double> q = math::Quat<double>::Identity();
    Vec3 lla_deg = Vec3::Zero();
    double heading_deg = 0.0;

    bool use_init_sim_state = false;
    cfg.get("use_init_sim_state", make_out(use_init_sim_state), true);

    if (use_init_sim_state)
    {
        const auto& entry =
            log.getNext<math::TwoJet<double>, math::DQuat<double>>(logging::TRUTH_POSE);
        t = entry.t;
        const math::TwoJet<double>& two_jet = std::get<0>(entry.data);
        const math::DQuat<double>& T_e2g = std::get<1>(entry.data);
        x.pose = two_jet.x;
        x.vel = two_jet.dx.linear();
        u.accel = two_jet.d2x.linear() - Vec3(0, 0, 9.80665);
        u.gyro = two_jet.dx.angular();

        x.T_I2e = T_e2g.inverse();
    }
    else
    {
        t = log.startTime();
        cfg.get("translation0", make_out(translation), true);
        cfg.get("rotation0", make_out(q.arr_), true);
        cfg.get("vel0", make_out(x.vel), true);
        x.pose = math::DQuat<double>(q, translation);
        cfg.get("lla0_deg", make_out(lla_deg), true);
        cfg.get("heading0_deg", make_out(heading_deg), true);
        const Vec3 lla_rad(deg2Rad(lla_deg(0)), deg2Rad(lla_deg(1)), lla_deg(2));
        const double heading_rad = deg2Rad(heading_deg);
        math::Quatd q_hdg = math::Quatd::from_euler(0, 0, heading_rad);

        x.T_I2e = utils::WGS84::dq_ecef2ned(utils::WGS84::lla2ecef(lla_rad)).inverse();
        x.T_I2e.real() = q_hdg * x.T_I2e.real();
    }
    cfg.get("p_b2g0", make_out(x.p_b2g), true);

    x.gps_clk.setZero();
    x.gal_clk.setZero();
    x.glo_clk.setZero();
    x.acc_bias.setZero();
    x.gyro_bias.setZero();

    return std::make_pair(x, u);
}

void loadEphemeris(const std::string& log_dir, ekf::RtkEkfEstimator& ekf)
{
    logging::LogReader log_reader(log_dir);

    log_reader.setCallback(logging::LogKey::GPS_EPH, [&](const UTCTime& t, int key, auto& reader) {
        ephemeris::GPSEphemeris eph;
        reader.get(eph);
        const double dt = (t - eph.toe).toSec();
        if (std::abs(dt) > 7200)
        {
            info("Stale ephemeris: dt = {}.  eph.toe = {}, t = {}", mc::fmt(dt, eph.toe, t));
            return;
        }
        ekf.ephCb(eph);
        info("gps_eph, {}", mc::fmt(t));
    });

    log_reader.setCallback(logging::LogKey::GAL_EPH, [&](const UTCTime& t, int key, auto& reader) {
        ephemeris::GalileoEphemeris eph;
        reader.get(eph);
        const double dt = (t - eph.toe).toSec();
        if (std::abs(dt) > 7200)
        {
            info("Stale ephemeris: dt = {}.  eph.toe = {}, t = {}", mc::fmt(dt, eph.toe, t));
            return;
        }
        ekf.ephCb(eph);
        info("gal_eph, {}", mc::fmt(t));
    });

    log_reader.setCallback(logging::LogKey::GLO_EPH, [&](const UTCTime& t, int key, auto& reader) {
        ephemeris::GlonassEphemeris eph;
        reader.get(eph);
        const double dt = (t - eph.toe).toSec();
        if (std::abs(dt) > 7200)
        {
            info("Stale ephemeris: dt = {}.  eph.toe = {}, t = {}", mc::fmt(dt, eph.toe, t));
            return;
        }
        ekf.ephCb(eph);
        info("glo_eph, {}", mc::fmt(t));
    });
    log_reader.read();
}

template <typename T>
T getMeasurementCovariance(const std::string& key, utils::Config& cfg)
{
    T vec;
    cfg.get(key, make_out(vec), true);
    return vec.cwiseProduct(vec);
}

int main(int argc, char** argv)
{
    std::string config_file = "";
    std::string log_dir = "";

    ArgumentParser parser("Post-process a data log");
    parser.add_argument("--config_file")
        .help("config file to read (yaml)")
        .action([&](const std::string& value) { config_file = value; })
        .required();
    parser.add_argument("--log")
        .help("log_to_read")
        .action([&](const std::string& value) { log_dir = value; })
        .required();
    parser.parse_args(argc, argv);

    {
        std::ifstream f(config_file);
        std::cout << f.rdbuf() << std::endl;
    }
    utils::Config cfg(config_file);
    logging::LogReader log_reader(log_dir);
    ekf::RtkEkfEstimator ekf;

    loadEphemeris(log_dir, ekf);

    const auto x_and_u = getState(cfg, log_reader);
    State x0 = x_and_u.first;
    Input u = x_and_u.second;
    Covariance P0 = getStateCovariance("P0_stdev", cfg);
    ProcessCovariance process_cov = getStateCovariance("process_stdev", cfg);

    Vec6 point_pos_cov = getMeasurementCovariance<Vec6>("point_pos_stdev", cfg);
    Vec2 gps_obs_cov = getMeasurementCovariance<Vec2>("gps_obs_stdev", cfg);
    Vec2 gal_obs_cov = getMeasurementCovariance<Vec2>("gal_obs_stdev", cfg);
    Vec2 glo_obs_cov = getMeasurementCovariance<Vec2>("glo_obs_stdev", cfg);
    InputCovariance imu_cov;
    imu_cov.accel = getMeasurementCovariance<Vec3>("imu_stdev/accel", cfg);
    imu_cov.gyro = getMeasurementCovariance<Vec3>("imu_stdev/gyro", cfg);
    double fix_and_hold_stdev;
    cfg.get("fix_and_hold_stdev", make_out(fix_and_hold_stdev), true);
    Vec1 fix_and_hold_cov(fix_and_hold_stdev * fix_and_hold_stdev);

    logging::Logger log_writer(x0.t, "../logs/post_process");
    log_writer.initStream<math::DQuat<double>, Vec3, Vec3, Vec3, Vec3, Vec2, Vec2, Vec2>(
        logging::ESTIMATE,
        {"pose", "vel", "omg", "acc", "p_ecef", "gps_clk", "gal_clk", "glo_clk"});
    log_writer.amends(log_reader.logPath());

    ekf.init(x0.t, x0, P0, point_pos_cov, gps_obs_cov, gal_obs_cov, glo_obs_cov, fix_and_hold_cov,
             imu_cov, process_cov, &log_writer);
    ekf.u() = u;

    log_reader.setMaintenanceCallback(0.01, [&](const UTCTime& t) {
        if ((t - x0.t).toSec() > 45.0)
        {
            return Error::create("exit early");
        }
        log_writer.log(logging::ESTIMATE, ekf.t(), ekf.x().pose, ekf.x().vel, ekf.u().gyro,
                       ekf.u().accel, ekf.p_ecef(), ekf.x().gps_clk, ekf.x().gal_clk,
                       ekf.x().glo_clk, ekf.cov());

        return Error::none();
    });

    log_reader.setCallback(logging::LogKey::IMU_SAMPLE,
                           [&](const UTCTime& t, int key, auto& reader) {
                               meas::ImuSample imu;
                               reader.get(imu);
                               ImuMeas meas;
                               meas.z.accel = imu.accel;
                               meas.z.gyro = imu.gyro;
                               ekf.addMeasurement(t, meas);
                           });

    log_reader.setCallback(logging::LogKey::GNSS_OBS, [&](const UTCTime& t, int key, auto& reader) {
        meas::GnssObservation obs;
        reader.get(obs);
        switch (obs.gnss_id)
        {
        case GnssID::GPS: {
            gpsObsMeas meas;
            meas.z << obs.pseudorange, obs.doppler;
            meas.sat_id = obs.sat_num;
            ekf.addMeasurement(t, meas);
            info("gps_obs, {}", mc::fmt(t));
            break;
        }
        case GnssID::Galileo: {
            galObsMeas meas;
            meas.z << obs.pseudorange, obs.doppler;
            meas.sat_id = obs.sat_num;
            info("gal_obs, {}", mc::fmt(t));
            ekf.addMeasurement(t, meas);
            break;
        }
        case GnssID::Glonass: {
            gloObsMeas meas;
            meas.z << obs.pseudorange, obs.doppler;
            meas.sat_id = obs.sat_num;
            info("glo_obs, {}", mc::fmt(t));
            ekf.addMeasurement(t, meas);
            break;
        }
        }
    });

    log_reader.read();
}

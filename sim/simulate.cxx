#include <experimental/filesystem>
#include <fstream>
#include <regex>
#include <streambuf>
#include <string>

#include "common/logging/log_reader.h"
#include "common/logging/log_writer.h"
#include "sim/sim.h"
#include "third_party/argparse/argparse.hpp"
#include "utils/config.h"
#include "utils/file.h"
#include "utils/progress_bar.h"
#include "utils/wgs84.h"

using namespace mc;
using namespace third_party::argparse;

bool stop = false;
void inthand(int signum)
{
    stop = true;
}

UTCTime get_start_time(const std::string& log_file)
{
    logging::LogReader log_reader(log_file);
    return log_reader.startTime();
}

int main(int argc, char** argv)
{
    ArgumentParser parser("Run a Simulation Experiment");
    sim::Sim::Options options;

    std::string config_file = "";
    double dt = 0.01;
    double tmax = 300.0;
    std::string output_dir;
    std::string eph_directory;
    Vec3 ref_lla{deg2Rad(40.246184), -deg2Rad(111.647769), 1387.998309};

    parser.add_argument("--config_file")
        .help("config file to read (yaml)")
        .action([&](const std::string& value) { config_file = value; })
        .required();
    parser.parse_args(argc, argv);

    mc::utils::Config cfg(config_file);

    cfg.get("dt", make_out(dt));
    cfg.get("tmax", make_out(tmax));
    cfg.get("eph_log", make_out(options.gnss_options.ephemeris_log), true);
    cfg.get("output_dir", make_out(output_dir), true);

    cfg.get("max_speed", make_out(options.wp_options.max_speed));
    cfg.get("max_omega", make_out(options.wp_options.max_omega));
    cfg.get("gnss_update_rate", make_out(options.gnss_options.update_rate_hz_));
    cfg.get("prange_stdev", make_out(options.gnss_options.pseudorange_stdev_));
    cfg.get("cphase_stdev", make_out(options.gnss_options.carrier_phase_stdev_));
    cfg.get("doppler_stdev", make_out(options.gnss_options.doppler_stdev_));
    cfg.get("multipath_prob", make_out(options.gnss_options.multipath_probability_));
    cfg.get("lol_probability", make_out(options.gnss_options.loss_of_lock_probability_));
    cfg.get("clock_walk_stdev", make_out(options.gnss_options.clock_walk_stdev_));
    cfg.get("clock_init_stdev", make_out(options.gnss_options.clock_init_stdev_));
    cfg.get("ref_lla", make_out(ref_lla));
    cfg.get("p_b2g", make_out(options.gnss_options.p_b2g));
    cfg.get("update_rate", make_out(options.imu_options.update_rate_hz));
    cfg.get("accel_noise_stdev", make_out(options.imu_options.accel_noise_stdev));
    cfg.get("accel_walk_stdev", make_out(options.imu_options.accel_walk_stdev));
    cfg.get("gyro_noise_stdev", make_out(options.imu_options.gyro_noise_stdev));
    cfg.get("gyro_walk_stdev", make_out(options.imu_options.gyro_walk_stdev));
    cfg.get("waypoints", make_out(options.wp_options.waypoints));

    options.gnss_options.T_e2n = utils::WGS84::dq_ecef2ned(utils::WGS84::lla2ecef(ref_lla));

    const UTCTime t0 = get_start_time(options.gnss_options.ephemeris_log);
    std::cout << "Simulation beginning at " << t0 << std::endl;
    options.car_options.t0 = t0;
    mc::UTCTime t_end = t0 + tmax;

    sim::Sim sim(options);

    logging::Logger log(t0, output_dir);

    log.initStream<math::TwoJet<double>>(logging::TRUTH_POSE, {"pose"});
    log.initStream<mc::meas::ImuSample>(logging::IMU_SAMPLE, {"imu"});
    log.initStream<mc::meas::GnssObservation>(logging::GNSS_OBS, {"obs"});
    log.amends(sim.logPath());

    const auto obs_cb = [&](const std::vector<mc::meas::GnssObservation>& obs) {
        for (const auto o : obs)
        {
            log.log(logging::LogKey::GNSS_OBS, sim.t(), o);
        }
    };
    sim.registerGnssCB(obs_cb);

    const auto imu_cb = [&](const mc::meas::ImuSample& imu) {
        log.log(logging::LogKey::IMU_SAMPLE, sim.t(), imu);
    };
    sim.registerImuCB(imu_cb);

    mc::utils::ProgressBar prog(std::round(tmax / dt), 100);
    int i = 0;

    while (!stop && sim.t() < t_end)
    {
        sim.step(dt);
        log.log(logging::LogKey::TRUTH_POSE, sim.t(), sim.x());
        const double rel_t = (sim.t() - t0).toSec();
        prog.print(++i, rel_t);
    }
    info("wrote log to {}", log.logId());
    log.close(sim.t());
    exit(EXIT_SUCCESS);
}

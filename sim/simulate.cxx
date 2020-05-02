#include <experimental/filesystem>
#include <fstream>
#include <regex>
#include <streambuf>
#include <string>

#include "common/logging/header.h"
#include "common/logging/logger.h"
#include "sim/sim.h"
#include "third_party/argparse/argparse.hpp"
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

std::vector<Vec3> default_waypoints = {{0, 0, 0},
                                       {100, 100, 0},
                                       {-100, 100, 0},
                                       {-100, -100, 0},
                                       {100, -100, 0}};

int main(int argc, char** argv)
{
    ArgumentParser parser("Run a Simulation Experiment");
    sim::Sim::Options options;
    parser.add_argument("--input_noise").help("/s").set(options.input_noise);
    parser.add_argument("--max_speed").help("m/s").set(options.wp_options.max_speed);
    parser.add_argument("--max_omega").help("rad/s").set(options.wp_options.max_omega);

    // clang-format off
    double dt = 0.01;
    double tmax = 300.0;
    std::string output_dir;
    std::string eph_directory;
    std::vector<double> ref_lla{deg2Rad(40.246184), -deg2Rad(111.647769), 1387.998309};
    std::vector<double> p_b2g{0, 0, 0};
    parser.add_argument("--eph_directory").help("location of gps.log, galileo.log, and/or glonass.log").required()
                .action([&](const std::string& value){eph_directory = value;});
    parser.add_argument("--gnss_update_rate").help("Hz").set(options.gnss_options.update_rate_hz_);
    parser.add_argument("--prange_stdev").help("m").set(options.gnss_options.pseudorange_stdev_);
    parser.add_argument("--cphase_stdev").help("cycles").set(options.gnss_options.carrier_phase_stdev_);
    parser.add_argument("--doppler_stdev").help("m/s").set(options.gnss_options.doppler_stdev_);
    parser.add_argument("--multipath_prob").help("[0-1]").set(options.gnss_options.multipath_probability_);
    parser.add_argument("--lol_probability").help("[0-1]").set(options.gnss_options.loss_of_lock_probability_);
    parser.add_argument("--clock_walk_stdev").help("nsec").set(options.gnss_options.clock_walk_stdev_);
    parser.add_argument("--clock_init_stdev").help("nsec/s").set(options.gnss_options.clock_init_stdev_);
    parser.add_argument("--ref_lla").help("rad, rad, m")
                .default_value(ref_lla).nargs(3).action([](const std::string& value) { return std::stod(value); });
    parser.add_argument("--p_b2g").help("position of GNSS receiver wrt body frame (m)")
                .default_value(p_b2g).nargs(3).action([](const std::string& value) { return std::stod(value); });

    parser.add_argument("--update_rate").help("Hz").set(options.imu_options.update_rate_hz);
    parser.add_argument("--accel_noise_stdev").help("m/s²").set(options.imu_options.accel_noise_stdev);
    parser.add_argument("--accel_walk_stdev").help("m/s³").set(options.imu_options.accel_walk_stdev);
    parser.add_argument("--gyro_noise_stdev").help("rad/s").set(options.imu_options.gyro_noise_stdev);
    parser.add_argument("--gyro_walk_stdev").help("rad/s²").set(options.imu_options.gyro_walk_stdev);

    parser.add_argument("--dt").help("simuation step size (s)").set(dt);
    parser.add_argument("--tmax").help("max simulation length (s)").set(tmax);
    parser.add_argument("--output_dir").help("output directory for simulation logs").required()
                .action([&](const std::string& value) {output_dir = value;});
    // clang-format on

    try
    {
        parser.parse_args(argc, argv);
    }
    catch (const std::runtime_error& err)
    {
        std::cout << err.what() << std::endl;
        std::cout << parser;
        exit(EXIT_FAILURE);
    }

    // Convert std::vector<double> into relevant Eigen types
    p_b2g = parser.get<std::vector<double>>("--p_b2g");
    ref_lla = parser.get<std::vector<double>>("--ref_lla");
    Vec3 ref_lla_eigen;
    for (int i = 0; i < 3; ++i)
    {
        options.gnss_options.p_b2g[i] = p_b2g[i];
        ref_lla_eigen[i] = ref_lla[i];
    }
    options.gnss_options.T_e2n = utils::WGS84::dq_ecef2ned(utils::WGS84::lla2ecef(ref_lla_eigen));

    // Read the ephemeris directory
    namespace fs = std::experimental::filesystem;
    for (const auto& entry : fs::directory_iterator(fs::path(eph_directory)))
    {
        const std::string name = fs::path(entry).stem();
        if (name.compare("gps") == 0)
        {
            options.gnss_options.ephemeris_files_[GnssID::GPS] = fs::path(entry).string();
        }
        if (name.compare("galileo") == 0 || name.compare("gal") == 0)
        {
            options.gnss_options.ephemeris_files_[GnssID::Galileo] = fs::path(entry).string();
        }
        if (name.compare("glonass") == 0 || name.compare("glo") == 0)
        {
            options.gnss_options.ephemeris_files_[GnssID::Glonass] = fs::path(entry).string();
        }
    }
    // Get the start time from the GPS ephemeris log header
    mc::UTCTime t0;
    if (!mc::logging::getHeaderTime((fs::path(eph_directory) / "gps.hdr").string(), Out(t0)).ok())
    {
        mc::fatal("Unable to retrieve header time for ephemeris data");
        exit(EXIT_FAILURE);
    }
    std::cout << "Simulation beginning at " << t0 << std::endl;
    options.car_options.t0 = t0;

    options.wp_options.waypoints = default_waypoints;

    sim::Sim sim(options);

    mc::UTCTime t_end = t0 + tmax;

    mc::logging::Logger truth_log(output_dir + "/truth.log");
    mc::logging::Logger obs_log(output_dir + "/obs.log");
    mc::logging::Logger imu_log(output_dir + "/imu.log");

    const auto obs_cb = [&](const std::vector<mc::meas::GnssObservation>& obs) {
        const double rel_t = (sim.t() - t0).toSec();
        for (const auto o : obs)
        {
            obs_log.log(rel_t, o);
        }
    };
    sim.registerGnssCB(obs_cb);

    const auto imu_cb = [&](const mc::meas::ImuSample& imu) {
        const double rel_t = (sim.t() - t0).toSec();
        imu_log.log(rel_t, imu);
    };
    sim.registerImuCB(imu_cb);

    mc::utils::ProgressBar prog(std::round(tmax / dt), 100);
    int i = 0;

    while (!stop && sim.t() < t_end)
    {
        sim.step(dt);
        const double rel_t = (sim.t() - t0).toSec();
        truth_log.log(rel_t, sim.x());
        prog.print(++i, rel_t);
    }
    std::cout << std::endl;
    exit(EXIT_SUCCESS);
}

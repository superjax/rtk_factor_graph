#include <map>
#include <vector>

#include "async_comm/serial.h"
#include "common/defs.h"
#include "common/ephemeris/galileo.h"
#include "common/ephemeris/glonass.h"
#include "common/ephemeris/gps.h"
#include "common/logging/header.h"
#include "common/logging/logger.h"
#include "common/measurements/gnss_observation.h"
#include "common/print.h"
#include "parsers/ubx/ubx.h"
#include "third_party/argparse/argparse.hpp"

using namespace mc;
using namespace third_party::argparse;
using namespace parsers;
using namespace ephemeris;

bool stop = false;
void inthand(int signum)
{
    stop = true;
}

template <typename EphType>
struct Accumulator
{
    std::map<int, EphType*> working;
    std::map<int, std::vector<EphType*>> finished;

    void handle(const ublox::RXM_SFRBX_t& msg)
    {
        const int sat = msg.sv_id;
        // if we have not seen ephemeris from this satellite before, start a new one
        if (working.find(sat) == working.end() || working[sat] == nullptr)
        {
            working[sat] = new EphType(sat);
        }

        if (working[sat]->parse(reinterpret_cast<const uint8_t*>(&msg.dwrd), msg.num_words * 4))
        {
            warn("finished {}: {}", mc::fmt(working[sat]->Type(), working[sat]->sat));

            if ((working[sat]->toe - UTCTime::now()).toSec() > 2 * UTCTime::SEC_IN_HOUR)
            {
                warn("really stale ephemeris: toe: {}, now: {}",
                     mc::fmt(working[sat]->toe, UTCTime::now()));
            }

            // move the ephemeris into the finished bucket
            finished[sat].push_back(working[sat]);
            working[sat] = nullptr;
        }
    }

    void dumpToFile(const std::string& output_file) const
    {
        logging::Logger log(output_file);
        log.addHeader(logging::makeHeader({"eph"}, EphType(0)));
        for (const auto& sat_vec : finished)
        {
            for (const auto& eph : sat_vec.second)
            {
                log.log(*eph);
            }
        }
    }

    std::string type_string;
};

struct ObsListener : public ublox::UbxListener
{
    ObsListener() { subscribe(ublox::CLASS_RXM, ublox::RXM_RAWX); }

    static double get_freq(int gnss_id, int sig_id, int slot)
    {
        double freq = 0;
        switch (gnss_id)
        {
        // clang-format off
        case GnssID::GPS:
                using gps = ephemeris::GPSEphemeris;
                freq = sig_id == 0 ? gps::FREQUENCY_L1 :
                       sig_id == 3 ? gps::FREQUENCY_L2 :
                       sig_id == 4 ? gps::FREQUENCY_L2 :
                       -1;
            break;
        case GnssID::Glonass:
                using glo = ephemeris::GlonassEphemeris;
                freq = sig_id == 0 ? glo::FREQ1_GLO + glo::DFRQ1_GLO * slot :
                       sig_id == 2 ? glo::FREQ2_GLO + glo::DFRQ2_GLO * slot :
                       -1;
            break;
        case GnssID::Galileo:
                using gal = ephemeris::GalileoEphemeris;
                freq = sig_id == 0 ? gal::FREQUENCY_E1 :
                       sig_id == 1 ? gal::FREQUENCY_E1 :
                       sig_id == 5 ? gal::FREQUENCY_E5b :
                       sig_id == 6 ? gal::FREQUENCY_E5b :
                       -1;
            break;
        default:
            dbg("Unsupported satellite Observation");
            break;
        }
        if (freq < 0) {
            error("unsupported sig_id {} for {}", mc::fmt(sig_id, gnss_id));
        }
        // clang-format on
        return freq;
    }

    void gotUbx(const uint8_t cls, const uint8_t id, const ublox::UbxMessage& msg)
    {
        const ublox::RXM_RAWX_t& data = msg.RXM_RAWX;
        fmt::print("{}, Got {} observations\n", UTCTime::now(), data.num_meas);
        for (int i = 0; i < data.num_meas; ++i)
        {
            const ublox::RXM_RAWX_t::RawxMeas& meas = data.meas[i];

            meas::GnssObservation o;
            o.gnss_id = meas.gnss_id;
            o.sat_num = meas.sv_id;
            o.freq = get_freq(meas.gnss_id, meas.sig_id, meas.freq_id);
            o.pseudorange = meas.pr_meas;
            o.doppler = meas.do_meas;
            o.carrier_phase = meas.cp_meas;
            if (o.freq > 0)
            {
                obs.emplace_back(o, UTCTime::now());
            }
        }
    }
    std::vector<std::pair<meas::GnssObservation, UTCTime>> obs;

    void dumpToFile(const std::string& output_file)
    {
        logging::Logger log(output_file);
        log.addHeader(logging::makeHeader({"t", "x"}, UTCTime(), meas::GnssObservation()));
        for (const auto& o_t : obs)
        {
            log.log(o_t.second, o_t.first);
        }
    }
};

struct Listener : public ublox::UbxListener
{
    Listener() { subscribe(ublox::CLASS_RXM, ublox::RXM_SFRBX); }

    void gotUbx(const uint8_t cls, const uint8_t id, const ublox::UbxMessage& msg)
    {
        const ublox::RXM_SFRBX_t& subframe = msg.RXM_SFRBX;

        switch (subframe.gnss_id)
        {
        case ublox::GnssID_GPS:
            gps.handle(subframe);
            break;
        case ublox::GnssID_Glonass:
            glonass.handle(subframe);
            break;
        case ublox::GnssID_Galileo:
            galileo.handle(subframe);
            break;
        default:
            // error("unhandled ephemeris {}", fmt(subframe.gnss_id));
            break;
        }
    }

    Accumulator<GPSEphemeris> gps;
    Accumulator<GalileoEphemeris> galileo;
    Accumulator<GlonassEphemeris> glonass;
};

int main(int argc, char** argv)
{
    ArgumentParser parser("Gather Ephemeris Data from UBX Receiver");
    parser.add_argument("--port").help("Serial Port").required();
    parser.add_argument("--baudrate")
        .help("baudrate")
        .required()
        .action([](const std::string& value) { return std::stoi(value); });
    parser.add_argument("--output_directory").help("Output Directory").required();

    try
    {
        parser.parse_args(argc, argv);
    }
    catch (const std::runtime_error& err)
    {
        std::cout << err.what() << std::endl;
        std::cout << parser;
        exit(0);
    }

    const std::string port = parser.get<std::string>("--port");
    const int baudrate = parser.get<int>("--baudrate");
    const std::string output_directory = parser.get<std::string>("--output_directory");

    fmt::print("port = {}\n", port);
    fmt::print("baudrate = {}\n", baudrate);
    fmt::print("output_directory = {}\n", output_directory);

    // Initialize Serial Port
    async_comm::Serial serial(port, baudrate);
    if (!serial.init())
    {
        error("Failed to initialize serial port");
        return 2;
    }

    ublox::Ubx ubx(serial);

    ubx.disableNmea();
    ubx.getVersion();
    ubx.setNavRate(200);
    ubx.enableMessage(ublox::CLASS_RXM, ublox::RXM_SFRBX, 1);
    ubx.enableMessage(ublox::CLASS_RXM, ublox::RXM_RAWX, 1);

    Listener listener;
    ObsListener obs_listener;
    ubx.registerListener(&listener);
    ubx.registerListener(&obs_listener);

    // quit if Ctrl+c
    signal(SIGINT, inthand);
    while (!stop)
    {
        sleep(1);
    }

    warn("Saving files, do not close");
    listener.gps.dumpToFile(output_directory + "/gps.log");
    listener.glonass.dumpToFile(output_directory + "/glonass.log");
    listener.galileo.dumpToFile(output_directory + "/galileo.log");
    obs_listener.dumpToFile(output_directory + "/obs.log");
    warn("Done saving files to {}", mc::fmt(output_directory));

    serial.close();

    return EXIT_SUCCESS;
}

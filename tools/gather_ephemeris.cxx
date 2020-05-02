#include <map>
#include <vector>

#include "async_comm/serial.h"
#include "common/ephemeris/galileo.h"
#include "common/ephemeris/glonass.h"
#include "common/ephemeris/gps.h"
#include "common/logging/header.h"
#include "common/logging/logger.h"
#include "common/print.h"
#include "parsers/ubx/ubx.h"
#include "third_party/argparse/argparse.hpp"

using namespace third_party::argparse;
using namespace mc::parsers;
using namespace mc::ephemeris;

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
            mc::warn("finished {}: {}", mc::fmt(working[sat]->Type(), working[sat]->sat));

            // move the ephemeris into the finished bucket
            finished[sat].push_back(working[sat]);
            working[sat] = nullptr;
        }
    }

    void dumpToFile(const std::string& output_file) const
    {
        mc::logging::Logger log(output_file);
        log.addHeader(mc::logging::makeHeader({"eph"}, EphType(0)));
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
            mc::error("unhandled ephemeris {}", mc::fmt(subframe.gnss_id));
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
        mc::error("Failed to initialize serial port");
        return 2;
    }

    ublox::Ubx ubx(serial);

    ubx.disableNmea();
    ubx.getVersion();
    ubx.setNavRate(200);
    ubx.enableMessage(ublox::CLASS_RXM, ublox::RXM_SFRBX, 1);

    Listener listener;
    ubx.registerListener(&listener);

    // quit if Ctrl+c
    signal(SIGINT, inthand);
    while (!stop)
    {
        sleep(1);
    }

    mc::warn("Saving files, do not close");
    listener.gps.dumpToFile(output_directory + "/gps.log");
    listener.glonass.dumpToFile(output_directory + "/glonass.log");
    listener.galileo.dumpToFile(output_directory + "/galileo.log");
    mc::warn("Done saving files to {}", mc::fmt(output_directory));

    serial.close();

    return EXIT_SUCCESS;
}

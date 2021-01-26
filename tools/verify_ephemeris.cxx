#include <fstream>
#include <map>
#include <vector>

#include "common/ephemeris/galileo.h"
#include "common/ephemeris/glonass.h"
#include "common/ephemeris/gps.h"
#include "common/error.h"
#include "common/logging/log_reader.h"
#include "common/logging/log_writer.h"
#include "common/print.h"
#include "common/utctime.h"
#include "third_party/argparse/argparse.hpp"

using namespace third_party::argparse;
using namespace mc::ephemeris;
using namespace mc::logging;
using mc::UTCTime;

template <typename EphType>
mc::Error verify(const EphType& eph)
{
    fmt::print("Type: {}, sat: {}, TOE: {}\n", eph.Type(), eph.sat, eph.toe);
    return mc::Error::none();
}

int main(int argc, char** argv)
{
    ArgumentParser parser("Verify Ephemeris Data gathered from UBX Receiver");
    parser.add_argument("--log").help("Log to verify").required();
    parser.parse_args(argc, argv);

    const std::string file = parser.get<std::string>("--log");

    auto gps_cb = [&](const UTCTime& t, int key, mc::logging::StreamReader& reader) {
        GPSEphemeris eph;
        reader.get(eph);
        verify(eph);
    };
    auto gal_cb = [&](const UTCTime& t, int key, mc::logging::StreamReader& reader) {
        GalileoEphemeris eph;
        reader.get(eph);
        verify(eph);
    };
    auto glo_cb = [&](const UTCTime& t, int key, mc::logging::StreamReader& reader) {
        GlonassEphemeris eph;
        reader.get(eph);
        verify(eph);
    };

    LogReader log(file);
    log.setCallback(LogKey::GPS_EPH, std::move(gps_cb));
    log.setCallback(LogKey::GAL_EPH, std::move(gal_cb));
    log.setCallback(LogKey::GLO_EPH, std::move(glo_cb));

    log.read();

    return EXIT_SUCCESS;
}

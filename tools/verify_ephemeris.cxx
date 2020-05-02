#include <fstream>
#include <map>
#include <vector>

#include "common/ephemeris/galileo.h"
#include "common/ephemeris/glonass.h"
#include "common/ephemeris/gps.h"
#include "common/error.h"
#include "common/logging/log_reader.h"
#include "common/print.h"
#include "third_party/argparse/argparse.hpp"

using namespace third_party::argparse;
using namespace mc::ephemeris;

template <typename EphType>
mc::Error verify(const std::string& file)
{
    std::vector<EphType> eph;
    mc::logging::LogReader log_reader;
    const auto result = log_reader.open(file);
    if (!result.ok())
    {
        return result;
    }

    while (true)
    {
        EphType new_eph(0);
        log_reader.read(new_eph);
        if (log_reader.done())
        {
            break;
        }
        eph.push_back(new_eph);
        fmt::print("Type: {}, sat: {}, TOE: {}\n", new_eph.Type(), new_eph.sat, new_eph.toe);
    }
    return mc::Error::none();
}

int main(int argc, char** argv)
{
    ArgumentParser parser("Verify Ephemeris Data gathered from UBX Receiver");
    parser.add_argument("--type").help("Type of ephemeris [gps galileo glonass]").required();
    parser.add_argument("--file").help("File to verify").required();

    try
    {
        parser.parse_args(argc, argv);
    }
    catch (const std::runtime_error& err)
    {
        std::cout << err.what() << std::endl;
        std::cout << parser;
        return EXIT_FAILURE;
    }

    const std::string type = parser.get<std::string>("--type");
    const std::string file = parser.get<std::string>("--file");

    if (type.compare("gps") == 0)
    {
        if (!verify<GPSEphemeris>(file).ok())
        {
            return EXIT_FAILURE;
        }
    }
    else if (type.compare("glonass") == 0)
    {
        if (!verify<GlonassEphemeris>(file).ok())
        {
            return EXIT_FAILURE;
        }
    }
    else if (type.compare("galileo") == 0)
    {
        if (!verify<GalileoEphemeris>(file).ok())
        {
            return EXIT_FAILURE;
        }
    }
    else
    {
        fmt::print("unsupported ephemeris type {}.  Choose from [gps, glonass, galileo]\n", type);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

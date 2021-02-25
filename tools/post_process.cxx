#include "common/logging/log_reader.h"
#include "third_party/argparse/argparse.hpp"
#include "utils/config.h"

using namespace mc;
using namespace third_party::argparse;

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

    utils::Config cfg(config_file);

    logging::LogReader log_reader(log_dir);
}

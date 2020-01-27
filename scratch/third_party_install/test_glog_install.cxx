#include "glog/logging.h"

namespace mc {
namespace scratch {
namespace third_party_install {

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);

    int num_cookies = 1000;
    LOG(INFO) << "Found " << num_cookies << " cookies";
}

}  // namespace third_party_install
}  // namespace scratch
}  // namespace mc

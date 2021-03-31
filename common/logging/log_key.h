#pragma once

#include <string>

#include "common/print.h"

namespace mc {
namespace logging {

enum LogKey
{
    IMU_SAMPLE,
    GNSS_OBS,
    GPS_EPH,
    GAL_EPH,
    GLO_EPH,
    TRUTH_POSE
};

inline std::string logKeyName(int key)
{
    switch (key)
    {
    case IMU_SAMPLE:
        return "IMU_SAMPLE";
    case GNSS_OBS:
        return "GNSS_OBS";
    case GPS_EPH:
        return "GPS_EPH";
    case GAL_EPH:
        return "GAL_EPH";
    case GLO_EPH:
        return "GLO_EPH";
    case TRUTH_POSE:
        return "TRUTH_POSE";
    default:
        return fmt::format("LOG_KEY_{:0^4}", key);
    }
}

}  // namespace logging
}  // namespace mc

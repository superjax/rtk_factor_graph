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
    TRUTH_POSE,
    GPS_OBS_RESIDUAL,
    GAL_OBS_RESIDUAL,
    GLO_OBS_RESIDUAL,
    POINT_POS_RESIDUAL,
    FIX_AND_HOLD_RESIDUAL,
    ESTIMATE
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
    case GPS_OBS_RESIDUAL:
        return "GPS_OBS_RESIDUAL";
    case GLO_OBS_RESIDUAL:
        return "GLO_OBS_RESIDUAL";
    case GAL_OBS_RESIDUAL:
        return "GAL_OBS_RESIDUAL";
    case POINT_POS_RESIDUAL:
        return "POINT_POS_RESIDUAL";
    case FIX_AND_HOLD_RESIDUAL:
        return "FIX_AND_HOLD_RESIDUAL";
    case ESTIMATE:
        return "ESTIMATE";
    default:
        return fmt::format("LOG_KEY_{:0^4}", key);
    }
}

}  // namespace logging
}  // namespace mc

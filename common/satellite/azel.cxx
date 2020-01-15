#include "common/satellite/azel.h"

#include <cmath>

void los2AzEl(const Vec3& receiver_pos_ecef, const Vec3& los_ecef, AzimuthElevation* az_el)
{
    // Compute the transform from ECEF -> NED
    const DQuat<double> dq_e2n = WGS84::dq_ecef2ned(receiver_pos_ecef);

    // Rotate the line-of-sight vector into the NED frame
    const Vec3 los_ned = dq_e2n.real().rotp(los_ecef.normalized());

    // Compute Azimuth
    az_el->az = atan2(los_ned.y(), los_ned.x());
    az_el->el = atan2(-los_ned.z(), sqrt(los_ned.y() * los_ned.y() + los_ned.x() * los_ned.x()));
}

AzimuthElevation los2AzEl(const Vec3& receiver_pos_ecef, const Vec3& los_ecef)
{
    AzimuthElevation az_el;
    los2AzEl(receiver_pos_ecef, los_ecef, &az_el);
    return az_el;
}

AzimuthElevation getAzEl(const Vec3& receiver_pos_ecef, const SatelliteState& sat)
{
    Vec3 los_ecef = sat.pos - receiver_pos_ecef;
    return los2AzEl(receiver_pos_ecef, los_ecef);
}

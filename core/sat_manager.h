#pragma once

#include "common/circular_buffer.h"
#include "common/ephemeris/galileo.h"
#include "common/ephemeris/glonass.h"
#include "common/ephemeris/gps.h"
#include "common/satellite/satellite.h"

namespace mc {

class SatelliteManager
{
 private:
    using GpsEph = ephemeris::GPSEphemeris;
    using GloEph = ephemeris::GlonassEphemeris;
    using GalEph = ephemeris::GalileoEphemeris;

 public:
    Error getSat(int gnss_id, int sat_num, Out<const satellite::SatelliteBase*> sat) const;

    void ephCb(const ephemeris::GPSEphemeris& eph);
    void ephCb(const ephemeris::GlonassEphemeris& eph);
    void ephCb(const ephemeris::GalileoEphemeris& eph);

 private:
    static constexpr int MAX_SAT = 30;

    CircularBuffer<satellite::Satellite<GpsEph>, MAX_SAT> gps_;
    CircularBuffer<satellite::Satellite<GloEph>, MAX_SAT> glo_;
    CircularBuffer<satellite::Satellite<GalEph>, MAX_SAT> gal_;
};

}  // namespace mc

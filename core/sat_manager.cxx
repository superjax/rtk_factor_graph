#include "core/sat_manager.h"

namespace mc {
namespace core {

template <typename T, int SIZE>
static const T* find(int sat_num, const CircularBuffer<T, SIZE>& sat_vec)
{
    for (const auto& sat : sat_vec)
    {
        if (sat.sat_num() == sat_num)
        {
            return &sat;
        }
    }
    return nullptr;
}

template <typename T, int SIZE>
static T* find(int sat_num, CircularBuffer<T, SIZE>& sat_vec)
{
    for (auto& sat : sat_vec)
    {
        if (sat.sat_num() == sat_num)
        {
            return &sat;
        }
    }
    return nullptr;
}

template <typename EphType, int SIZE>
static void addEph(const EphType& eph, CircularBuffer<satellite::Satellite<EphType>, SIZE>& sat_vec)
{
    satellite::Satellite<EphType>* sat = find(eph.sat, sat_vec);

    // This is a new satellite, so add it to the queue
    if (sat == nullptr)
    {
        mc::dbg("Discovered new {} satellite #{}", fmt(eph.Type(), eph.sat));
        if (sat_vec.full())
        {
            mc::fatal("{} satellite buffer is full, removing satellite {}",
                      fmt(eph.Type(), sat_vec.front().sat_num()));
            sat_vec.pop_front();
        }
        sat_vec.emplace_back(eph.gnssID, eph.sat);
        sat = &sat_vec.back();
    }

    mc::dbg("Adding ephemeris to {} satellite {}", fmt(eph.Type(), eph.sat));
    sat->addEph(eph);
}

void SatelliteManager::ephCb(const ephemeris::GPSEphemeris& eph)
{
    addEph(eph, gps_);
}

void SatelliteManager::ephCb(const ephemeris::GlonassEphemeris& eph)
{
    addEph(eph, glo_);
}

void SatelliteManager::ephCb(const ephemeris::GalileoEphemeris& eph)
{
    addEph(eph, gal_);
}

Error SatelliteManager::getSat(int gnss_id,
                               int sat_num,
                               Out<const satellite::SatelliteBase*> sat) const
{
    switch (gnss_id)
    {
    case GnssID::GPS:
        *sat = find(sat_num, gps_);
        break;
    case GnssID::Glonass:
        *sat = find(sat_num, glo_);
        break;
    case GnssID::Galileo:
        *sat = find(sat_num, gal_);
        break;
    default:
        return Error::create("Unsuppored Gnss Type");
    }
    if (*sat == nullptr)
    {
        return Error::create("Requested unknown satellite");
    }
    return Error::none();
}

}  // namespace core
}  // namespace mc

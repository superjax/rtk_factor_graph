#pragma once

#include <algorithm>
#include <vector>

#include "common/check.h"
#include "common/out.h"
#include "common/print.h"
#include "common/satellite/satellite_state.h"

namespace mc {
namespace satellite {

class SatelliteBase
{
 public:
    SatelliteBase(uint8_t gnss_id, uint8_t sat_num) : gnss_id_{gnss_id}, sat_{sat_num} {}
    virtual Error getState(const UTCTime& t, Out<SatelliteState> sat_state) const = 0;

    uint8_t gnssId() const { return gnss_id_; }
    uint8_t sat_num() const { return sat_; }

    virtual int almanacSize() const = 0;

 private:
    uint8_t gnss_id_;  // GNSS system
    uint8_t sat_;      // Satellite ID (within GNSS system)
};

template <typename EphType>
class Satellite : public SatelliteBase
{
 public:
    Satellite(uint8_t gnss_id, uint8_t sat_num);

    Error addEph(const EphType& eph)
    {
        check(eph.gnssID == gnssId(), "Mismatched satellite GNSS system");
        check(eph.sat == sat_num(), "Mismatched satellite number");

        auto it = almanac_.begin();
        while (it != almanac_.end())
        {
            if (it->toe == eph.toe)
            {
                // This ephemeris is already in our almanac
                // Update our local copy
                *it = eph;
                return Error::none();
            }
            else if (it->toe > eph.toe)
            {
                // The new ephemeris is coming out of order, insert it into the correct place
                almanac_.insert(it, eph);
                return Error::none();
            }
            ++it;
        }

        // New ephemeris is in the correct order
        almanac_.push_back(eph);
        return Error::none();
    }

    const EphType& findClosestEphemeris(const UTCTime& t) const
    {
        check(almanac_.size() > 0, "Tried to get Ephemeris from an empty almanac");
        double dt = std::numeric_limits<double>::max();
        auto min_it = almanac_.rend();
        for (auto it = almanac_.rbegin(); it != almanac_.rend(); ++it)
        {
            const double dt_i = std::abs((t - it->toe).toSec());
            if (dt_i < dt)
            {
                dt = dt_i;
                min_it = it;
            }
            else
            {
                // If the dt is getting bigger, then no ephemeris left in the almanac will be closer
                // (almanac is sorted in ascending order)
                break;
            }
        }

        return *min_it;
    }

    int almanacSize() const { return almanac_.size(); }

    Error getState(const UTCTime& t, Out<SatelliteState> sat_state) const
    {
        return eph2Sat(t, findClosestEphemeris(t), sat_state);
    }

 private:
    std::vector<EphType> almanac_;

    friend class Satellite_AlmanacIsSorted_Test;
    friend class Satellite_GloAlmanacIsSorted_Test;
};

}  // namespace satellite
}  // namespace mc

#include "sim/sensors/gnss.h"

#include <fstream>

#include "common/ephemeris/galileo.h"
#include "common/ephemeris/glonass.h"
#include "common/ephemeris/gps.h"
#include "common/logging/log_key.h"
#include "common/logging/log_reader.h"
#include "common/random.h"
#include "common/satellite/atm_correction.h"

namespace mc {
namespace sim {
namespace sensors {

GnssSim::GnssSim(const Options& options, const UTCTime& t0) : options_(options)
{
    prev_t_ = t0;

    clock_bias_ << options_.clock_init_stdev_, options_.clock_walk_stdev_;
    clock_bias_ = clock_bias_.cwiseProduct(randomNormal<Vec2>());

    load();

    // initialize the carrier phase offset
    carrier_phase_offsets_.resize(satellites_.size());
    for (auto& offset : carrier_phase_offsets_)
    {
        offset = rand() % 1000 - 500;
    }
}

template <typename T>
void ephCb(const T& new_eph, Out<std::map<int, satellite::Satellite<T>>> sats)
{
    if (sats->find(new_eph.sat) == sats->end())
    {
        sats->emplace(new_eph.sat, satellite::Satellite<T>(new_eph.gnssID, new_eph.sat));
    }
    sats->at(new_eph.sat).addEph(new_eph);

    fmt::print("Type: {}, sat: {}, TOE: {}\n", new_eph.Type(), new_eph.sat, new_eph.toe);
}

template <typename T>
void getSatPointers(const std::map<int, satellite::Satellite<T>>& sat_map,
                    Out<std::vector<const satellite::SatelliteBase*>> base_ptrs)
{
    for (const auto& sat : sat_map)
    {
        base_ptrs->push_back(dynamic_cast<const satellite::SatelliteBase*>(&(sat.second)));
    }
}

void GnssSim::load()
{
    logging::LogReader log(options_.ephemeris_log);

    log.setCallback(logging::GPS_EPH,
                    [&](const UTCTime& t, int key, logging::StreamReader& reader) {
                        ephemeris::GPSEphemeris eph;
                        reader.get(eph);
                        ephCb(eph, make_out(gps_));
                        gps_ephemerides_.push_back(std::make_tuple(t, eph));
                    });
    log.setCallback(logging::GLO_EPH,
                    [&](const UTCTime& t, int key, logging::StreamReader& reader) {
                        ephemeris::GlonassEphemeris eph;
                        reader.get(eph);
                        ephCb(eph, make_out(glo_));
                        glo_ephemerides_.push_back(std::make_tuple(t, eph));
                    });
    log.setCallback(logging::GAL_EPH,
                    [&](const UTCTime& t, int key, logging::StreamReader& reader) {
                        ephemeris::GalileoEphemeris eph;
                        reader.get(eph);
                        ephCb(eph, make_out(gal_));
                        gal_ephemerides_.push_back(std::make_tuple(t, eph));
                    });
    log_path_ = log.logPath();
    log.read();

    getSatPointers(gps_, make_out(satellites_));
    getSatPointers(glo_, make_out(satellites_));
    getSatPointers(gal_, make_out(satellites_));

    check(satellites_.size() > 0ul, "Unable to find any satellites in ephemeris data");
}

bool GnssSim::sample(const UTCTime& t,
                     const math::TwoJet<double>& x,
                     Out<std::vector<meas::GnssObservation>> obs)
{
    const double dt = (t - prev_t_).toSec();
    if (dt >= 1.0 / options_.update_rate_hz_)
    {
        prev_t_ = t;

        // Integrate clock bias
        const double prev_rate = clock_bias_(1);
        clock_bias_(1) += options_.clock_walk_stdev_ * randomNormal<Vec1>()(0) * dt;
        const double avg_rate = (prev_rate + clock_bias_(1)) / 2.0;
        clock_bias_(0) += avg_rate * dt;

        // Compute position and velocity of the GPS recevier
        Vec3 vel_ecef = options_.T_e2n.rota(x.dx.linear() + x.dx.angular().cross(x.dx.linear()));
        const Vec3 p_ecef = options_.T_e2n.transforma(x.x.translation() + x.x.rota(options_.p_b2g));

        obs->clear();
        int i = 0;
        for (auto* sat : satellites_)
        {
            double pseudorange;
            double doppler;
            range(t, sat, p_ecef, vel_ecef, make_out(pseudorange), make_out(doppler));
            double carrier_phase = pseudorange / sat->carrierWavelength(0) +
                                   normal_(gen_) * options_.carrier_phase_stdev_ +
                                   carrier_phase_offsets_[i++];
            pseudorange += normal_(gen_) * options_.pseudorange_stdev_;
            doppler += normal_(gen_) * options_.doppler_stdev_;

            obs->emplace_back();
            obs->back().pseudorange = pseudorange;
            obs->back().doppler = doppler;
            obs->back().carrier_phase = carrier_phase;
            obs->back().gnss_id = sat->gnssId();
            obs->back().sat_num = sat->sat_num();
            obs->back().freq = 0;  // TODO: add L1/L2 to simulation
        }
        return true;
    }

    return false;
}

bool GnssSim::sampleGpsEph(const UTCTime& t, Out<ephemeris::GPSEphemeris> eph)
{
    if (std::get<UTCTime>(gps_ephemerides_.front()) >= t)
    {
        (*eph) = std::get<ephemeris::GPSEphemeris>(gps_ephemerides_.front());
        gps_ephemerides_.erase(gps_ephemerides_.begin());
        return true;
    }
    else
        return false;
}
bool GnssSim::sampleGloEph(const UTCTime& t, Out<ephemeris::GlonassEphemeris> eph)
{
    if (std::get<UTCTime>(glo_ephemerides_.front()) >= t)
    {
        (*eph) = std::get<ephemeris::GlonassEphemeris>(glo_ephemerides_.front());
        glo_ephemerides_.erase(glo_ephemerides_.begin());
        return true;
    }
    else
        return false;
}
bool GnssSim::sampleGalEph(const UTCTime& t, Out<ephemeris::GalileoEphemeris> eph)
{
    if (std::get<UTCTime>(gal_ephemerides_.front()) >= t)
    {
        (*eph) = std::get<ephemeris::GalileoEphemeris>(gal_ephemerides_.front());
        gal_ephemerides_.erase(gal_ephemerides_.begin());
        return true;
    }
    else
        return false;
}

double computeSagnac(const satellite::SatelliteState& sat_state, const Vec3& rec_pos_ecef)
{
    return OMEGA_EARTH *
           (sat_state.pos.x() * rec_pos_ecef.y() - sat_state.pos.y() * rec_pos_ecef.x()) / C_LIGHT;
}

void GnssSim::range(const UTCTime& t,
                    const satellite::SatelliteBase* sat,
                    const Vec3& p_ecef,
                    const Vec3& vel_ecef,
                    Out<double> pseudorange,
                    Out<double> doppler)
{
    satellite::SatelliteState sat_state;
    sat->getState(t, make_out(sat_state));

    double dt;
    Vec3 los_to_sat;
    double range;
    do
    {
        // Compute how long it took for the signal to get here (including Sagnac Correction)
        los_to_sat = sat_state.pos - p_ecef;
        const double sagnac = computeSagnac(sat_state, p_ecef);
        range = los_to_sat.norm() + sagnac;

        double tau = range / C_LIGHT;
        dt = tau - (t - sat_state.t).toSec();
        sat->getState(sat_state.t - dt, make_out(sat_state));
    } while (abs(dt) * sat_state.vel.norm() > 1e-3);  // get millimeter-accurate

    satellite::AtmosphericCorrection atm;
    satellite::computeAtmCorrection(t, p_ecef, sat_state, make_out(atm));

    const double dclock = clock_bias_(0) - sat_state.clk(0);
    const double dclock_rate = clock_bias_(1) - sat_state.clk(1);

    *pseudorange = range + atm.ionospheric_delay_m + atm.tropospheric_delay_m + C_LIGHT * dclock;
    *doppler = (sat_state.vel - vel_ecef).dot(los_to_sat / range) +
               OMEGA_EARTH / C_LIGHT *
                   (sat_state.vel.y() * p_ecef.x() + sat_state.pos.y() * vel_ecef.x() -
                    sat_state.vel.x() * p_ecef.y() - sat_state.pos.x() * vel_ecef.y()) +
               C_LIGHT * dclock_rate;
}

}  // namespace sensors
}  // namespace sim
}  // namespace mc

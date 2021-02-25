
#include <vector>

#include "common/ephemeris/galileo.h"
#include "common/ephemeris/glonass.h"
#include "common/ephemeris/gps.h"
#include "common/measurements/gnss_observation.h"
#include "common/measurements/imu.h"
#include "common/satellite/satellite.h"
#include "core/factor_graph.h"
#include "core/solver/solver.h"
#include "core/states/antenna_position.h"
#include "core/states/clock_bias.h"
#include "core/states/imu_bias.h"
#include "core/states/pose.h"
#include "core/states/vel.h"

namespace mc {
namespace core {

class KalmanFilter
{
 public:
    struct Options
    {
        // Number of nodes in the slding Window
        int num_nodes = 10;

        FactorGraph::Options fg_options;
    };

    explicit KalmanFilter(const Options& options);

    // Return current estimates
    void currentState(Out<UTCTime> t, Out<math::DQuat<double>> pose, Out<Vec3> vel) const;
    // const states::AntennaPosition& antennaPosition() const;
    // const states::ImuBias& imuBias() const;
    // const states::ClockBias& clockBias() const;

    // Sensor Inputs
    void imuCb(const meas::ImuSample& imu);
    // void obsCb(const std::vector<meas::GnssObservation>& obs);
    void ephCb(const ephemeris::GPSEphemeris& eph);
    void ephCb(const ephemeris::GlonassEphemeris& eph);
    void ephCb(const ephemeris::GalileoEphemeris& eph);

 private:
    Options options_;
    FactorGraph graph_;

    // Solver solver_;
};

}  // namespace core
}  // namespace mc

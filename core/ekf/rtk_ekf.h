#include <unordered_map>

#include "common/satellite/atm_correction.h"
#include "common/satellite/satellite_cache.h"
#include "core/ekf/rtk_ekf_base.h"

namespace mc {
namespace ekf {

class RtkEkf : public RtkEkfBase<RtkEkf>
{
 public:
    RtkEkf() = default;
    template <typename MeasType, typename... Args>
    typename MeasType::Residual h(const typename MeasType::ZType& z,
                                  const State& x,
                                  typename MeasType::Jac* jac,
                                  const Args&... args) const;

    static ErrorState errorStateDynamics(const ErrorState& dx,
                                         const State& x,
                                         const Input& u,
                                         const Input& eta);

    static ErrorState dynamics(const State& x,
                               const Input& u,
                               RtkEkf::StateJac* dxdx,
                               RtkEkf::InputJac* dxdu);

    Vec3 p_e_g2e() const { return x().T_I2e.transformp(x().pose.transforma(x().p_b2g)); }
};

}  // namespace ekf
}  // namespace mc

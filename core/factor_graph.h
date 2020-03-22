#include <common/error.h>
#include <core/factors/factors.h>
#include <core/solver/solver.h>
#include <core/states/states.h>

namespace mc {

class FactorGraph
{
 private:
    FactorGraph();

    std::vector<std::unique_ptr<factors::Factor>> factors;
    std::vector<std::unique_ptr<states::State>> states;
    std::unique_ptr<Solver> solver;

 public:
    static FactorGraph build();

    std::unique_ptr<factors::Factor> const& addFactor(std::unique_ptr<factors::Factor> f);
    std::unique_ptr<factors::Factor> const& addState(std::unique_ptr<states::State> s);
    Error removeFactor(factors::Factor& f);
    Error removeState(states::State& s);
    Error setSolver(Solver&& s);
    Error solve();
};

}  // namespace mc

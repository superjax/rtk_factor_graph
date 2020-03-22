#pragma once

#include <memory>
#include "common/hetero_collection.h"
#include "common/math/dquat.h"
#include "utils/non_copyable.h"

namespace mc {

namespace factors {
class Factor;
}

namespace states {

enum
{
    STATE_ID_ANTENNA_POSITION,
    STATE_ID_CLOCK_BIAS,
    STATE_ID_GNSS_SWITCH,
    STATE_ID_IMU_BIAS,
    STATE_ID_POSE,
    STATE_ID_VELOCITY

};

class State : public utils::NonCopyable
{
 public:
    inline State(const int _dim, const int _id) : DIM(_dim), ID(_id) {}
    virtual ~State(){};
    // States don't usually need access to individual factors with their correct type so it makes
    // sense to use traditional inheritance, but we'll keep the same interface for consistency
    std::vector<factors::Factor*> factors;
    std::vector<factors::Factor*> base_items() { return factors; };

    // Generic Interface
    const int DIM;
    const int ID;
    virtual double* const data() = 0;
    virtual const double* const data() const = 0;
};

template <typename StateType, typename... CollectionTypeList>
class StateTemplate : public State
{
 public:
    StateTemplate() : State(StateType::DIM, StateType::ID) {}
    ~StateTemplate(){};

    // typedef std::shared_ptr<StateType> shared_ptr;

    inline double* const data() { return static_cast<StateType*>(this)->getData(); }
    inline const double* const data() const
    {
        return static_cast<const StateType*>(this)->getData();
    }
};

}  // namespace states
}  // namespace mc

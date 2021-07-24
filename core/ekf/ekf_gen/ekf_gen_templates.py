STATE_HEADER_TEMPLATE = """
#pragma once

#include "common/matrix_defs.h"
#include "common/quantized_time.h"
#include "common/random.h"
{INCLUDES}

// Forward Declarations
{BEGIN_NAMESPACES}
class {ErrorState};
class {State};
class {Input};
class Covariance;
class ProcessCovariance;
class InputCovariance;
{END_NAMESPACES}

namespace mc {{
template <>
{namespaces}::{Input} randomNormal<{namespaces}::{Input}>();
bool isFinite(const ::{namespaces}::{State}& x);
bool isFinite(const ::{namespaces}::{Input}& u);
bool isFinite(const ::{namespaces}::{ErrorState}& x);
bool isFinite(const ::{namespaces}::Covariance& x);
bool isFinite(const ::{namespaces}::ProcessCovariance& x);
bool isFinite(const ::{namespaces}::InputCovariance& x);
}}

{BEGIN_NAMESPACES}

class {ErrorState}
{{
 private:
    {ERROR_STATE_BUFFERS}

 public:
    static constexpr int DOF = {DOF};

    // Members of the error state.
    {ERROR_STATE_MEMBERS}

    // Access a full buffer segment
    {ERROR_STATE_VECTOR_GETTERS}

    {ErrorState} operator*(const double s) const;
    {ErrorState} operator/(const double s) const;
    {ErrorState}& operator+=(const {ErrorState}& dx);
    {ErrorState} operator+(const {ErrorState}& dx) const;
    {ErrorState} operator-(const {ErrorState}& dx) const;

    // helpers
    static {ErrorState} Random();
    static {ErrorState} Zero();
    static {ErrorState} Constant(double s);
    void setRandom();
    void setZero();

    static {ErrorState} fromDense(const Eigen::Matrix<double, {DOF}, 1>& x);

    friend bool mc::isFinite(const {ErrorState}& x);

    {ERROR_STATE_DENSE_DECL};
}};

class {State} {{
 private:
    {USING_STATEMENTS}

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr int DOF = {DOF};

    // Time
    QuantizedTime t;

    // States
    {STATE_MEMBERS}

    // Constructors
    {State}();
    {State}(const {State}& other);
    {State}& operator=(const {State}& obj);

    // Manipulators
    {State} operator+(const {ErrorState} &delta) const;
    {State}& operator+=(const {ErrorState} &delta);
    {ErrorState} operator-(const {State} &x2) const;

    // helpers
    static {State} Random();
    static {State} Identity();
    void setRandom();
    void setIdentity();
    inline int size() const {{ return {STATE_SIZE}; }}
}};

class Covariance {{
 private:
    {COVARIANCE_BUFFERS}

 public:
    Covariance() = default;
    Covariance(const ProcessCovariance& cov);

    {COVARIANCE_MEMBERS}

    friend bool mc::isFinite(const Covariance& x);

    {COVARIANCE_DENSE_DECL}

    // helpers
    static Covariance Random();
    static Covariance Identity();
    void setRandom();
    void setIdentity();
}};

class {Input} {{
 private:
    {INPUT_USING_STATEMENTS}
 public:
    static constexpr int DOF = {INPUT_DOF};

    {INPUT_MEMBERS}

    // Constructors
    {Input}() = default;

    static {Input} Random();
    static {Input} Zero();
    void setRandom();
    void setZero();

    {Input} operator*(double s);
    {Input} operator+=(const {Input}& u);
    {Input} operator+(const {Input}& dx) const;

    {INPUT_DENSE_DECL}
    static {Input} fromDense(const Eigen::Matrix<double, {INPUT_DOF}, 1>& x);
}};


class ProcessCovariance {{
 public:
    {PROCESS_COVARIANCE_MEMBERS}

    static ProcessCovariance Identity();
    static ProcessCovariance Zero();
    static ProcessCovariance Random();

    friend bool mc::isFinite(const ProcessCovariance& x);

    {COVARIANCE_DENSE_DECL}
    void setRandom();
    void setIdentity();
    void setZero();
}};

class InputCovariance {{
 public:
    {INPUT_COVARIANCE_MEMBERS}

    friend bool mc::isFinite(const InputCovariance& x);

    {INPUT_COVARIANCE_DENSE_DECL}
    static InputCovariance Random();
    static InputCovariance Zero();
    void setRandom();
    void setZero();
}};

struct Snapshot
{{
    {State} x;
    Covariance cov;
    {Input} u;
}};


{END_NAMESPACES}
"""

STATE_IMPL_TEMPLATE = """
#include "{DESTINATION_DIR}/{EKF_FILENAME}_state.h"

{BEGIN_NAMESPACES}

using namespace Eigen;

void {ErrorState}::setRandom()
{{
    {ERROR_STATE_SET_RANDOM}
}}

void {ErrorState}::setZero()
{{
    {ERROR_STATE_SET_ZERO}
}}

{ErrorState}& {ErrorState}::operator+=(const {ErrorState}& dx) {{
    {ERROR_STATE_SELF_PLUS}
    return *this;
}}

{ErrorState} {ErrorState}::operator+(const {ErrorState}& dx) const {{
    {ErrorState} out;
    {ERROR_STATE_PLUS}
    return out;
}}

{ErrorState} {ErrorState}::operator*(const double s) const {{
    {ErrorState} out;
    {SCALAR_MULTIPLY}
    return out;
}}

{ErrorState} {ErrorState}::operator/(const double s) const {{
    return operator*(1.0/s);
}}

{ErrorState} {ErrorState}::operator-(const {ErrorState}& dx) const {{
    {ErrorState} out;
    {ERROR_STATE_MINUS}
    return out;
}}

{ErrorState} {ErrorState}::Random()
{{
    {ErrorState} out;
    out.setRandom();
    return out;
}}

{ErrorState} {ErrorState}::Zero()
{{
    {ErrorState} out;
    out.setZero();
    return out;
}}

{ErrorState} {ErrorState}::fromDense(const Eigen::Matrix<double, {DOF}, 1>& x)
{{
    {ErrorState} out;
    {FROM_DENSE}
    return out;
}}

{State}::{State}() {{}}


{State}::{State}(const {State} &other)
{{
    *this = other;
}}

{State}& {State}::operator= (const {State}& other)
{{
    {COPY_STATE}
    return *this;
}}

{State} {State}::operator+(const {ErrorState}& dx) const
{{
    {State} xp;
    {BOXPLUS_IMPL}
    return xp;
}}

{State}& {State}::operator+=(const {ErrorState}& dx)
{{
    {SELF_PLUS_IMPL}
    return *this;
}}

{ErrorState} {State}::operator-(const {State}& x2) const
{{
    {ErrorState} dx;
    {BOXMINUS_IMPL}
    return dx;
}}

{State} {State}::Random()
{{
    {State} out;
    out.setRandom();
    return out;
}}

{State} {State}::Identity()
{{
    {State} out;
    out.setIdentity();
    return out;
}}

void {State}::setRandom()
{{
    {STATE_SET_RANDOM}
}}

void {State}::setIdentity()
{{
    {STATE_SET_IDENTITY}
}}

{ErrorState} {ErrorState}::Constant(double s){{
    {ErrorState} out;
    {ERROR_STATE_CONSTANT}
    return out;
}}

{ERROR_STATE_DENSE}

{INPUT_DENSE}

{Input} {Input}::fromDense(const Eigen::Matrix<double, {INPUT_DOF}, 1>& x)
{{
    {Input} out;
    {INPUT_FROM_DENSE}
    return out;
}}

{Input} {Input}::Random() {{
    {Input} out;
    out.setRandom();
    return out;
}}

{Input} {Input}::Zero() {{
    {Input} out;
    out.setZero();
    return out;
}}

void {Input}::setRandom(){{
    {INPUT_RANDOM}
}}

{Input} {Input}::operator*(double s)
{{
    {Input} out;
    {MULTIPLY_CONSTANT}
    return out;
}}

{Input} {Input}::operator+=(const {Input}& dx)
{{
    {SELF_ADD_INPUT}
    return *this;
}}

{Input} {Input}::operator+(const {Input}& dx) const {{
    {Input} out;
    {INPUT_PLUS}
    return out;
}}

void {Input}::setZero(){{
    {INPUT_ZERO}
}}

Covariance::Covariance(const ProcessCovariance& cov)
{{
    {COVARIANCE_FROM_PROCESS_COV}
}}

{COVARIANCE_DENSE}
Covariance Covariance::Random()
{{
    Covariance out;
    out.setRandom();
    return out;
}}

Covariance Covariance::Identity()
{{
    Covariance out;
    out.setIdentity();
    return out;
}}

void Covariance::setRandom(){{
    {COVARIANCE_SET_RANDOM}
}}

void Covariance::setIdentity(){{
    {COVARIANCE_IDENTITY}
}}

{PROCESS_COV_DENSE}
ProcessCovariance ProcessCovariance::Random()
{{
    ProcessCovariance out;
    out.setRandom();
    return out;
}}

ProcessCovariance ProcessCovariance::Zero()
{{
    ProcessCovariance out;
    out.setZero();
    return out;
}}

ProcessCovariance ProcessCovariance::Identity()
{{
    ProcessCovariance out;
    out.setIdentity();
    return out;
}}

void ProcessCovariance::setRandom(){{
    {PROCESS_COV_SET_RANDOM}
}}

void ProcessCovariance::setZero(){{
    {PROCESS_COV_SET_ZERO}
}}

void ProcessCovariance::setIdentity(){{
    {PROCESS_COV_SET_IDENTITY}
}}


{INPUT_COV_DENSE}
InputCovariance InputCovariance::Random()
{{
    InputCovariance out;
    out.setRandom();
    return out;
}}

InputCovariance InputCovariance::Zero()
{{
    InputCovariance out;
    out.setZero();
    return out;
}}

void InputCovariance::setRandom(){{
    {INPUT_COV_SET_RANDOM}
}}

void InputCovariance::setZero(){{
    {INPUT_COV_SET_ZERO}
}}


{END_NAMESPACES}

namespace mc {{

template <>
{namespaces}::{Input} randomNormal<{namespaces}::{Input}>()
{{
    {namespaces}::{Input} out;
    {INPUT_RANDOM_NORMAL}
    return out;
}}

bool isFinite(const ::{namespaces}::{State}& x) {{
    {FINITE_STATE}
}}

bool isFinite(const ::{namespaces}::{Input}& x) {{
    {FINITE_INPUT}
}}

bool isFinite(const ::{namespaces}::{ErrorState}& x) {{
    {FINITE_ERROR_STATE}
}}

bool isFinite(const ::{namespaces}::Covariance& x) {{
    {FINITE_COVARIANCE}
}}

bool isFinite(const ::{namespaces}::ProcessCovariance& x) {{
    {FINITE_PROCESS_COVARIANCE}
}}

bool isFinite(const ::{namespaces}::InputCovariance& x) {{
    {FINITE_INPUT_COVARIANCE}
}}

}}

"""

TYPE_HEADER_TEMPLATE = """
#pragma once

#include "{DESTINATION_DIR}/{EKF_FILENAME}_state.h"

{BEGIN_NAMESPACES}

{DYNAMICS_JACOBIAN}

{INPUT_JACOBIAN}

{MEAS_TYPES}

{END_NAMESPACES}

namespace mc {{
{IS_FINITE_DECLS}
}}

"""

TYPE_IMPL_TEMPLATE = """
#include "{DESTINATION_DIR}/{EKF_FILENAME}_types.h"

{BEGIN_NAMESPACES}

{DYNAMICS_JACOBIAN_HELPERS}

{INPUT_JACOBIAN_HELPERS}

{MEAS_TYPES_HELPERS}

{END_NAMESPACES}

namespace mc {{
{IS_FINITE_IMPLS}
}}

"""

EKF_HEADER_TEMPLATE = """
#pragma once

#include "common/check.h"
#include "common/error.h"
#include "common/error_result.h"
#include "common/matrix_defs.h"
#include "common/out.h"

#include "{DESTINATION_DIR}/{EKF_FILENAME}_state.h"
#include "{DESTINATION_DIR}/{EKF_FILENAME}_types.h"

{BEGIN_NAMESPACES}

// forward declare dynamics function
{ErrorState} dynamics(const {State}& x, const {Input}& u, StateJac* dxdx, InputJac* dxdu);


// Forward declare error state dynamics function
{ErrorState} errorStateDynamics(const {ErrorState}& dx,
                                const {State}& x,
                                const {Input}& u,
                                const {Input}& eta);

Error predict(const Snapshot& snap,
                const UTCTime& t_new,
                const {Input}& u,
                const ProcessCovariance& Qx,
                const InputCovariance& Qu,
                Out<Snapshot> out);

// Fordward declare the measurement model function
template<typename Meas, typename...Args>
typename Meas::Residual h(const typename Meas::ZType& z,
                          const {State}& x,
                          typename Meas::Jac* dzdx,
                          const Args&...args);

// Forward declare the measurement-specific update implementation
template<typename Meas>
Error doUpdate(Out<Snapshot> snap,
                const typename Meas::ZType& z,
                const typename Meas::Jac& dzdx,
                const typename Meas::Covariance& R,
                const typename Meas::Residual& res);

template<typename Meas, typename...Args>
ErrorResult<typename Meas::Residual> update(Out<Snapshot> snap,
                                            const typename Meas::ZType& z,
                                            const typename Meas::Covariance& R,
                                            const Args&... args)
{{
    check(isFinite(z), "numerical issues in measurement");
    check(isFinite(R), "numerical issues in measurement covariance");

    typename Meas::Jac dzdx;
    const typename Meas::Residual res = h<Meas, Args...>(z, snap->x, &dzdx, args...);
    check(isFinite(dzdx), "numerical issues in measurement jacobian");
    check(isFinite(res), "numerical issues in measurement residual");

    const Error err = doUpdate<Meas>(snap, z, dzdx, R, res);
    if (!err.ok())
    {{
        return err;
    }}

    check(isFinite(snap->x), "numerical issues in state post-update");
    check(isFinite(snap->cov), "numerical issues in covariance post-update");

    return res;
}}



{END_NAMESPACES}

"""

EKF_IMPL_TEMPLATE = """
#include "{DESTINATION_DIR}/{EKF_FILENAME}.h"

{BEGIN_NAMESPACES}

Error predict(const Snapshot& snap,
                const UTCTime& t_new,
                const {Input}& u,
                const ProcessCovariance& Qx,
                const InputCovariance& Qu,
                Out<Snapshot> out)
{{
    const auto& x = snap.x;
    const auto& cov = snap.cov;
    auto& x_out = out->x;
    auto& cov_out = out->cov;
    check(x.t != INVALID_TIME, "tried to run ekf without initializing");
    check(t_new > x.t, "Cannot propagate backwards");
    check(isFinite(x), "Numerical issues in state vector");
    check(isFinite(cov), "Numerical issues in covariance matrix");
    check(isFinite(u), "Numerical issues in input vector");
    check(isFinite(Qx), "Numerical problems in process noise");
    check(isFinite(Qu), "Numerical problems in input noise");

    {GET_VARIABLE_STATES}

    const double dt = (t_new - x.t).toSec();

    StateJac dxdx;
    InputJac dxdu;
    const {ErrorState} xdot = dynamics(x, u, &dxdx, &dxdu);

    check(isFinite(xdot), "Numerical Issues in derivative");
    check(isFinite(dxdx), "Numerical Issues in state jacobian");
    check(isFinite(dxdu), "Numerical Issues in input jacobian");

    // Propagate State
    x_out = x + (xdot * dt);
    x_out.t = x.t + dt;

    // Propagate Covariance
    // Abar = I + A*dt + 0.5*A^2*dt^2 ≈ exp(A*dt)
    {ABAR}

    // Bbar = B*dt + 0.5*AB*dt^2
    {BBAR}

    // P = Abar ⋅ P ⋅ Abarᵀ + Bbar ⋅ Qu ⋅ Bbarᵀ + Qx
    {PROPAGATE_COVARIANCE}
    out->u = u;

    return Error::none();
}}

{DO_UPDATES}

{END_NAMESPACES}

"""

DO_UPDATE_TEMPLATE = """
template<>
Error doUpdate<{Meas}>(Out<Snapshot> snap,
                      const {Meas}::ZType& z,
                      const {Meas}::Jac& dzdx,
                      const {Meas}::Covariance& R,
                      const {Meas}::Residual& res)
{{
    auto& cov = snap->cov;
    auto& x = snap->x;
    (void)x;
    {GET_VARIABLE_MEAS}
    {BAIL_IF_VARIABLE_MEAS_EMPTY}
    {GET_VARIABLE_STATES}


    // PHT = P ⋅ Hᵀ
    {SPARSE_PHT}

    // S⁻¹ = (HPHᵀ + R)⁻¹
    {SPARSE_INNOVATION}
    check(mc::isFinite(Sinv), "numerical issues in innovation");

    const double mahal = res.transpose() * Sinv * res;
    // Gating values with prob less than MAX_PROB (using χ² distribution with DOF = SIZE-1)
    if (mahal > {Meas}::MAX_MAHAL)
    {{
        return Error::create("Measurement Gated");
    }}


    if constexpr (!{Meas}::DISABLED)
    {{
        // K = PHᵀ ⋅ S⁻¹
        {SPARSE_KALMAN_GAIN}

        // dx = K ⋅ res
        {ErrorState} dx;
        {SPARSE_COMPUTE_ERROR_STATE}

        // x⁺ = x + dx
        snap->x += dx;

        // P⁺ = P - PHᵀ ⋅ S⁻¹ ⋅ HP
        //    = P - K ⋅ (PHᵀ)ᵀ
        {SPARSE_COVARIANCE_UPDATE}
    }}

    return Error::none();
}}

"""

TEST_TEMPLATE = """

#include "{DESTINATION_DIR}/{EKF_FILENAME}.h"

#include "common/test_helpers.h"

#include "gtest/gtest.h"

{BEGIN_NAMESPACES}

{SPARSE_PROPAGATE_TEST}

{SPARSE_UPDATE_TEST}

TEST(State, BoxplusRules)
{{
    {State} x = {State}::Random();
    {State} x2 = {State}::Random();
    {MAX_OUT_SIZE}
    const {ErrorState} zeros = {ErrorState}::Zero();
    const {ErrorState} dx = {ErrorState}::Random();

    {{
        // x + 0 == x
        {ZERO_ADD_TEST}
    }}

    {{
        // x + (x2 - x) == x2
        {ADD_SUBTRACT_TEST}
    }}

    {{
        // (x + dx) - x == dx
        {SUBTRACT_ADD_TEST}
    }}
}}


{END_NAMESPACES}

"""

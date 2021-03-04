STATE_HEADER_TEMPLATE = """
#pragma once

#include "common/matrix_defs.h"
#include "common/math/dquat.h"
#include "common/quantized_time.h"

{BEGIN_NAMESPACES}


class {ErrorState} : public  Eigen::Matrix<double, {DOF}, 1> {{
 public:
    static constexpr int DOF = {DOF};

    // Index of states in the state vector
    enum {{
        {ERROR_STATE_INDEXES},
    }};

    // Convenient accessors.  These are implemented so they behave like python @property
    // getters/setters.  They are actually just maps to blocks of `arr` above.
    {ERROR_STATE_ACCESSORS}

    // Constructors
    {ErrorState}();
    // {ErrorState}(const {ErrorState}& obj);
    template<typename Derived>
    {ErrorState}(const Eigen::MatrixBase<Derived>& obj) :
        {ErrorState}()
    {{
        Eigen::Matrix<double, DOF, 1>::operator=(obj);
    }}

    // Manipulators
    template<typename Derived>
    {ErrorState}& operator=(const Eigen::MatrixBase<Derived>& other)
    {{
        Eigen::Matrix<double, {ErrorState}::SIZE, 1>::operator=(other);
        return *this;
    }}

    static ErrorState Random();
    static ErrorState Zero();
}};

class {State} {{
 private:
    {USING_STATEMENTS}

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr int DOF = {DOF};

    // Index of states in the state vector
    enum {{
        {STATE_INDEXES},
    }};

    // These are the only memory-holding variable.  The rest of the class is just syntactical sugar
    Eigen::Matrix<double, SIZE, 1> arr;
    QuantizedTime t;

    // Convenient accessors.  These are implemented so they behave like python @property
    // getters/setters
    {STATE_ACCESSORS}

    // Constructors
    {State}();
    {State}(const {State}& other);
    {State}& operator=(const {State}& obj);

    // Manipulators
    // {State} operator+(const {ErrorState} &delta) const;
    {State} operator+(const Eigen::Matrix<double, {ErrorState}::SIZE, 1> &delta) const;
    // {State}& operator+=(const {ErrorState} &delta);
    {State}& operator+=(const Eigen::Matrix<double, {ErrorState}::SIZE, 1>& dx);
    {ErrorState} operator-(const {State} &x2) const;

    static State Random();
    static State Identity();
}};

class {Input} : public Eigen::Matrix<double, {INPUT_SIZE}, 1> {{
 public:
    static constexpr int DOF = {INPUT_SIZE};

    // Index of fields in the input
    enum {{
        {INPUT_INDEXES},
    }};

    // getters/setters
    {INPUT_ACCESSORS}

    // Constructors
    {Input}();
    template <typename Derived>
    {Input}(const Eigen::MatrixBase<Derived>& obj) : {Input} ()
    {{
        Eigen::Matrix<double, DOF, 1>::operator=(obj);
    }}

    template<typename Derived>
    {Input}& operator=(const Eigen::MatrixBase<Derived>& other)
    {{
        Eigen::Matrix<double, DOF, 1>::operator=(other);
        return *this;
    }}

    static {Input} Random();
    static {Input} Zero();
}};

inline bool isFinite(const {State}& x) {{ return mc::isFinite(x.arr); }}

typedef Eigen::Matrix<double, {ErrorState}::SIZE, {ErrorState}::SIZE> dxMat;
typedef Eigen::Matrix<double, {ErrorState}::SIZE, 1> dxVec;

{EXTRA_TYPEDEFS}

{END_NAMESPACES}

"""

STATE_IMPL_TEMPLATE = """
#include "{DESTINATION_DIR}/state.h"

{BEGIN_NAMESPACES}

using namespace Eigen;

{ErrorState}::{ErrorState}() :
    {ERROR_STATE_INIT_LIST}
{{
#ifdef INIT_NAN
    // This is to help track down init-time bugs
    this->setConstant(NAN);
#endif
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

constexpr int {ErrorState}::DOF;

{State}::{State}() :
    {STATE_INIT_LIST}
{{
#ifdef INIT_NAN
    // This is to help track down init-time bugs
    arr.setConstant(NAN);
    t = INVALID_TIME;
#endif
}}

{State}::{State}(const {State} &other) :
    {State}()
{{
    t = other.t;
    arr = other.arr;
}}

{State}& {State}::operator= (const {State}& other)
{{
    t = other.t;
    arr = other.arr;
    return *this;
}}

//{State} {State}::operator+(const ErrorState& dx) const
//{{
//    {{BOXPLUS_IMPL}}
//}}

{State} {State}::operator+(const Matrix<double, ErrorState::SIZE, 1>& dx) const
{{
    {BOXPLUS_VECTOR_IMPL}
}}

//{State}& {State}::operator+=(const ErrorState& dx)
//{{
//    {{SELF_PLUS_IMPL}}
//}}

{State}& {State}::operator+=(const Matrix<double, ErrorState::SIZE, 1>& dx)
{{
    {SELF_PLUS_VECTOR_IMPL}
}}

{ErrorState} {State}:: operator-(const {State}& x2) const
{{
    {BOXMINUS_IMPL}
}}

{State} {State}::Random()
{{
    {STATE_RANDOM_IMPL}
}}

{State} {State}::Identity()
{{
    {IDENTITY_IMPL}
}}

constexpr int {State}::DOF;


{Input}::{Input}() :
    {INPUT_INIT_LIST}
{{
#ifdef INIT_NAN
    // This is to help track down init-time bugs
    this->setConstant(NAN);
#endif
}}


{Input} {Input}::Random()
{{
    {Input} out;
    out.setRandom();
    return out;
}}

{Input} {Input}::Zero()
{{
    {Input} out;
    out.setZero();
    return out;
}}

constexpr int {Input}::DOF;

{END_NAMESPACES}


"""

EKF_HEADER_TEMPLATE = """
#pragma once

#include "common/matrix_defs.h"
#include "{DESTINATION_DIR}/state.h"
#include "common/error.h"
#include "common/check.h"

{BEGIN_NAMESPACES}

template<typename ChildType>
class {EkfType} {{
 private:
    template<int Rows, int Cols>
    using Mat = typename Eigen::Matrix<double, Rows, Cols>;

    template<int Rows>
    using Vec = typename Eigen::Matrix<double, Rows, 1>;

    template<typename T, int R, int C>
    using BlkMat = Eigen::Block<T, R, C>;
    {USING_STATEMENTS}

 public:
    {EkfType}()
    {{
    #ifdef INIT_NAN
        // This is to help track down init-time bugs
        cov_.setConstant(NAN);
    #endif
    }}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using ProcessCov = Eigen::DiagonalMatrix<double, {ErrorState}::SIZE>;
    using InputCov = Eigen::DiagonalMatrix<double, {InputSize}>;

    const UTCTime& t() const {{ return x_.t; }}

    {JAC_TYPES}

    State& x() {{ return x_; }} // The current state object
    const State& x() const {{ return x_; }}
    dxMat& P() {{ return cov_; }} // The current covariance
    const dxMat& P() const {{ return cov_; }}
    const {Input}& u() const {{ return u_; }} // The current input
    {Input}& u() {{ return u_; }}

    Error predict(const UTCTime& t_new,
                         const {Input}& u,
                         const ProcessCov& Qx,
                         const InputCov& Qu)
    {{
        check(x_.t != INVALID_TIME, "tried to run ekf without initializing");
        check(t_new > x_.t, "Cannot propagate backwards");
        check(isFinite(x_), "Numerical issues in state vector");
        check(mc::isFinite(cov_), "Numerical issues in covariance matrix");
        check(mc::isFinite(u), "Numerical issues in input vector");
        check(mc::isFinite(Qx), "Numerical problems in process noise");
        check(mc::isFinite(Qu), "Numerical problems in input noise");

        const double dt = (t_new - x_.t).toSec();

        StateJac dxdx;
        InputJac dxdu;
        const {ErrorState} xdot = static_cast<ChildType*>(this)->dynamics(x_, u, &dxdx, &dxdu);

        check(mc::isFinite(xdot), "Numerical Issues in derivative");
        check(mc::isFinite(dxdx), "Numerical Issues in state jacobian");
        check(mc::isFinite(dxdu), "Numerical Issues in input jacobian");

        // Propagate State
        x_ += (xdot * dt);
        x_.t += dt;

        // Propagate Covariance
        // P = A ⋅ P ⋅ Aᵀ + B ⋅ Qu ⋅ Bᵀ + Qx
        cov_ = dxdx * cov_ * dxdx.transpose() + dxdu * Qu*dt*dt * dxdu.transpose();
        cov_ += Qx*dt*dt;

        u_ = u;

        return Error::none();
    }}

    template<typename Meas, typename...Args>
    Error update(const typename Meas::ZType& z, const typename Meas::Covariance& R, const Args&... args)
    {{
        using mc::isFinite;

        check(mc::isFinite(z), "numerical issues in measurement");
        check(mc::isFinite(R), "numerical issues in measurement covariance");

        typename Meas::Jac dzdx;
        const typename Meas::Residual res = static_cast<ChildType*>(this)->template h<Meas, Args...>(z, x_, &dzdx, args...);
        check(mc::isFinite(dzdx), "numerical issues in measurement jacobian");
        check(mc::isFinite(res), "numerical issues in measurement residual");

        // innov = (H ⋅ P ⋅ H.T + R)⁻¹
        Mat<Meas::SIZE, Meas::SIZE> HPHT = (dzdx * cov_ * dzdx.transpose());
        HPHT += R;
        Mat<Meas::SIZE, Meas::SIZE> innov = HPHT.inverse();
        check(mc::isFinite(innov), "numerical issues in innovation");

        const double mahal = res.transpose() * innov * res;
        // Gating values with probability less than MAX_PROB (using χ² distribution with DOF = SIZE-1)
        if (mahal > Meas::MAX_MAHAL)
        {{
            return Error::create("Measurement Gated");
        }}

        // K = P ⋅ H.T * innov
        const Mat<ErrorState::SIZE, Meas::SIZE> K = cov_ * dzdx.transpose() * innov;
        check(mc::isFinite(K), "numerical issues in Kalman gain");

        x_ += K * res;

        // A = I - K ⋅ H
        // P = A ⋅ P ⋅ A.T + K ⋅ R ⋅ K.T
        const dxMat A = dxMat::Identity() - K * dzdx;
        cov_ = A * cov_ * A.transpose() + K * R * K.transpose();

        check(isFinite(x_), "numerical issues in state post-update");
        check(mc::isFinite(cov_), "numerical issues in covariance post-update");

        return Error::none();
    }}

 protected:
    {State} x_;
    dxMat cov_;
    {Input} u_;


}};

template<int Rows, int Cols>
using Mat = typename Eigen::Matrix<double, Rows, Cols>;
template<int Rows>
using Vec = typename Eigen::Matrix<double, Rows, 1>;
template<typename T, int R, int C>
using BlkMat = Eigen::Block<T, R, C>;

{MEAS_TYPES}

{END_NAMESPACES}

"""

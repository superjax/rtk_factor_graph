#include "core/ekf/rtk_ekf_estimator.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <variant>

#include "common/satellite/satellite_cache.h"
#include "common/test_helpers.h"
#include "core/ekf/rtk_ekf.h"

namespace mc {
namespace ekf {

using ::testing::_;
using ::testing::Eq;
using ::testing::InSequence;

class MockCallbacks
{
 public:
    MockCallbacks() = default;

    MOCK_CONST_METHOD1(pointPosCb, void(const UTCTime& t));
    MOCK_CONST_METHOD1(obsCb, void(const UTCTime& t));
    MOCK_CONST_METHOD1(fixAndHoldCb, void(const UTCTime& t));

    MOCK_CONST_METHOD0(errorStateDynCb, void());
    MOCK_CONST_METHOD1(dynamicsCb, void(const UTCTime& t));
};

MockCallbacks* cb;

ErrorState dynamics(const State& x, const Input& u, StateJac* dxdx, InputJac* dxdu)
{
    dxdx->setZero();
    dxdu->setZero();
    cb->dynamicsCb(x.t);
    return ErrorState::Zero();
}

ErrorState errorStateDynamics(const ErrorState& dx,
                              const State& x,
                              const Input& u,
                              const Input& eta)
{
    cb->errorStateDynCb();
    return ErrorState::Zero();
}

template <>
pointPosMeas::Residual h<pointPosMeas>(const pointPosMeas::ZType& z,
                                       const State& x,
                                       pointPosMeas::Jac* jac,
                                       const Input& u)
{
    jac->setZero();
    cb->pointPosCb(x.t);
    return Vec6::Zero();
}

template <>
obsMeas::Residual h<obsMeas>(const obsMeas::ZType& z,
                             const State& x,
                             obsMeas::Jac* jac,
                             const Input& u)
{
    jac->setZero();
    cb->obsCb(x.t);
    return obsMeas::Residual::Zero();
}

template <>
fixAndHoldMeas::Residual h<fixAndHoldMeas>(const fixAndHoldMeas::ZType& z,
                                           const State& x,
                                           fixAndHoldMeas::Jac* jac)
{
    jac->setZero();
    const int num_sd = x.num_sd;
    const int num_fix_and_hold = z.size();

    check(num_sd == num_fix_and_hold,
          "Number of fix-and-hold measurements must equal the size of the single-differences "
          "estimate.  num_sd = {}, num_fix_and_hold = {}",
          fmt(num_sd, num_fix_and_hold));

    cb->fixAndHoldCb(x.t);
    fixAndHoldMeas::Residual res;
    return res;
}

class RtkEkfEstimatorTest : public ::testing::Test
{
 public:
    void SetUp() override
    {
        UTCTime t0 = UTCTime(0);
        State x0 = State::Identity();
        Covariance P0 = Covariance::Identity();
        Vec6 point_pos_cov = Vec6::Ones();
        Vec2 obs_cov = Vec2::Ones();
        Vec1 fix_and_hold_cov = Vec1::Ones();
        InputCovariance imu_cov = InputCovariance::Zero();
        ProcessCovariance process_cov = ProcessCovariance::Identity();

        cb = &cb_;

        est.init(t0, x0, P0, point_pos_cov, obs_cov, fix_and_hold_cov, imu_cov, process_cov);
    }
    RtkEkfEstimator est;
    MockCallbacks cb_;
};

TEST_F(RtkEkfEstimatorTest, IntegrateNegative)
{
    est.addMeasurement(UTCTime(-0.1), ImuMeas::Zero());
}

TEST_F(RtkEkfEstimatorTest, InOrderIMU)
{
    // t:       .1  .2  .3  .4  .5
    // IMU      x   x   x   x   x
    {
        InSequence seq;

        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.4).quantized())));
    }

    est.addMeasurement(UTCTime(0.1), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.2), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.3), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.4), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.5), ImuMeas::Zero());

    UTCTime expected_time = UTCTime(0);
    double dt = 0.1;
    EXPECT_EQ(est.measurements_.size(), 5ul);
    for (const auto& m : est.measurements_)
    {
        expected_time += dt;
        EXPECT_EQ(m.t, expected_time);
    }
    EXPECT_EQ(est.t(), UTCTime(0.5));
    EXPECT_EQ(est.states_.size(), 6ul);
}

TEST_F(RtkEkfEstimatorTest, InOrderMeasurements)
{
    // t:       .1  .2  .3  .4  .5
    // IMU      x   x   x-> x   x
    //                    ->
    // PntPos             x
    //
    {
        InSequence seq;

        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(cb_, pointPosCb(Eq(UTCTime(0.35).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.35).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.4).quantized())));
    }

    pointPosMeas m;
    m.z.setZero();

    est.addMeasurement(UTCTime(0.1), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.2), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.3), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.35), m);
    est.addMeasurement(UTCTime(0.4), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.5), ImuMeas::Zero());

    auto it = est.measurements_.begin();

    EXPECT_EQ((it++)->t, UTCTime(0.1));
    EXPECT_EQ((it++)->t, UTCTime(0.2));
    EXPECT_EQ((it++)->t, UTCTime(0.3));
    EXPECT_EQ((it++)->t, UTCTime(0.35));
    EXPECT_EQ((it++)->t, UTCTime(0.4));
    EXPECT_EQ((it++)->t, UTCTime(0.5));
    EXPECT_EQ(it, est.measurements_.end());
    EXPECT_EQ(est.t(), UTCTime(0.5));
    EXPECT_EQ(est.states_.size(), 7ul);
}

TEST_F(RtkEkfEstimatorTest, OnTopOfMeasurements)
{
    // t:       .1  .2  .3  .4  .5
    // IMU      x   x   x   x   x
    // PntPos           x
    {
        InSequence seq;

        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(cb_, pointPosCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.4).quantized())));
    }

    pointPosMeas m;
    m.z.setZero();

    est.addMeasurement(UTCTime(0.1), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.2), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.3), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.3), m);
    est.addMeasurement(UTCTime(0.4), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.5), ImuMeas::Zero());

    auto it = est.measurements_.begin();

    EXPECT_EQ((it++)->t, UTCTime(0.1));
    EXPECT_EQ((it++)->t, UTCTime(0.2));
    EXPECT_EQ((it++)->t, UTCTime(0.3));
    EXPECT_EQ((it++)->t, UTCTime(0.3));
    EXPECT_EQ((it++)->t, UTCTime(0.4));
    EXPECT_EQ((it++)->t, UTCTime(0.5));
    EXPECT_EQ(it, est.measurements_.end());
    EXPECT_EQ(est.t(), UTCTime(0.5));
    EXPECT_EQ(est.states_.size(), 6ul);
}

TEST_F(RtkEkfEstimatorTest, SmallRewind)
{
    // t:       .1  .2  .3  .4  .5
    // IMU      x   x   x   x   x
    // PntPos         x<-----
    //                ------>

    {
        InSequence seq;

        // normal progression
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.3).quantized())));

        // catchup
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(cb_, pointPosCb(Eq(UTCTime(0.25).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.25).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.4).quantized())));
    }

    pointPosMeas m;
    m.z.setZero();

    est.addMeasurement(UTCTime(0.1), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.2), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.3), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.4), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.25), m);
    est.addMeasurement(UTCTime(0.5), ImuMeas::Zero());

    auto it = est.measurements_.begin();
    EXPECT_EQ((it++)->t, UTCTime(0.1));
    EXPECT_EQ((it++)->t, UTCTime(0.2));
    EXPECT_EQ((it++)->t, UTCTime(0.25));
    EXPECT_EQ((it++)->t, UTCTime(0.3));
    EXPECT_EQ((it++)->t, UTCTime(0.4));
    EXPECT_EQ((it++)->t, UTCTime(0.5));
    EXPECT_EQ(it, est.measurements_.end());
    EXPECT_EQ(est.t(), UTCTime(0.5));
    EXPECT_EQ(est.states_.size(), 7ul);
}

TEST_F(RtkEkfEstimatorTest, DoubleRewind)
{
    // t:       .1  .2  .3  .4  .5
    // IMU      x   x   x   x   x
    // PntPos         x<----- x<-
    //                ------> -->

    {
        InSequence seq;

        // normal progression
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.3).quantized())));

        // catchup
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(cb_, pointPosCb(Eq(UTCTime(0.25).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.25).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.4).quantized())));

        // catchup #2
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(cb_, pointPosCb(Eq(UTCTime(0.35).quantized())));

        // finish prop
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.35).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.4).quantized())));
    }

    pointPosMeas m;
    m.z.setZero();

    est.addMeasurement(UTCTime(0.1), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.2), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.3), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.4), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.25), m);
    est.addMeasurement(UTCTime(0.5), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.35), m);

    auto it = est.measurements_.begin();
    EXPECT_EQ((it++)->t, UTCTime(0.1));
    EXPECT_EQ((it++)->t, UTCTime(0.2));
    EXPECT_EQ((it++)->t, UTCTime(0.25));
    EXPECT_EQ((it++)->t, UTCTime(0.3));
    EXPECT_EQ((it++)->t, UTCTime(0.35));
    EXPECT_EQ((it++)->t, UTCTime(0.4));
    EXPECT_EQ((it++)->t, UTCTime(0.5));
    EXPECT_EQ(it, est.measurements_.end());
    EXPECT_EQ(est.t(), UTCTime(0.50));
    EXPECT_EQ(est.states_.size(), 8ul);
}

TEST_F(RtkEkfEstimatorTest, FurtherRewind)
{
    // t:       .1  .2  .3  .4  .5
    // IMU      x   x   x   x   x
    // PntPos         x<-----
    //                 ----->
    // galObs     x<-------------
    //             ------------->
    {
        InSequence seq;

        // normal progression
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.3).quantized())));

        // catchup
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(cb_, pointPosCb(Eq(UTCTime(0.25).quantized())));

        // finish prop
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.25).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.3).quantized())));

        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.4).quantized())));

        // delayed->catchup
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(cb_, pointPosCb(Eq(UTCTime(0.15).quantized())));

        // finish prop
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.15).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.20).quantized())));
        EXPECT_CALL(cb_, pointPosCb(Eq(UTCTime(0.25).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.25).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.4).quantized())));

        // Final in-order measurement
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.5).quantized())));
    }

    pointPosMeas m;
    m.z.setZero();

    est.addMeasurement(UTCTime(0.1), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.2), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.3), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.4), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.25), m);
    est.addMeasurement(UTCTime(0.5), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.15), m);
    est.addMeasurement(UTCTime(0.6), ImuMeas::Zero());

    auto it = est.measurements_.begin();
    EXPECT_EQ((it++)->t, UTCTime(0.1));
    EXPECT_EQ((it++)->t, UTCTime(0.15));
    EXPECT_EQ((it++)->t, UTCTime(0.2));
    EXPECT_EQ((it++)->t, UTCTime(0.25));
    EXPECT_EQ((it++)->t, UTCTime(0.3));
    EXPECT_EQ((it++)->t, UTCTime(0.4));
    EXPECT_EQ((it++)->t, UTCTime(0.5));
    EXPECT_EQ((it++)->t, UTCTime(0.6));
    EXPECT_EQ(it, est.measurements_.end());
    EXPECT_EQ(est.t(), UTCTime(0.6));
    EXPECT_EQ(est.states_.size(), 9ul);
}

TEST_F(RtkEkfEstimatorTest, MoveBufferFront)
{
    const int OVERFLOW = 3;
    const int times = est.states_.capacity() + OVERFLOW;
    EXPECT_CALL(cb_, dynamicsCb(_)).Times(times);

    const double dt = 0.1;
    UTCTime t(dt);
    for (int i = 0; i < times; ++i)
    {
        est.addMeasurement(t, ImuMeas::Zero());
        t += dt;
    }

    EXPECT_EQ(est.t(), UTCTime(dt * (est.states_.capacity() + OVERFLOW)));
    EXPECT_EQ(est.states_.front().x.t, est.t() - est.MAX_DELAY_AGE_S);
    EXPECT_EQ(est.states_.front().x.t, est.measurements_.begin()->t);
}

TEST_F(RtkEkfEstimatorTest, RewindAfterMoveDelay)
{
    const int OVERFLOW = 3;
    const int times = est.states_.capacity() + OVERFLOW;
    {
        InSequence seq;

        EXPECT_CALL(cb_, dynamicsCb(_)).Times(times);
        EXPECT_CALL(cb_, pointPosCb(_));
        EXPECT_CALL(cb_, dynamicsCb(_)).Times(5);
    }

    const double dt = 0.1;
    UTCTime t(dt);
    for (int i = 0; i < times; ++i)
    {
        est.addMeasurement(t, ImuMeas::Zero());
        t += dt;
    }

    pointPosMeas m;
    m.z.setZero();
    est.addMeasurement(est.t() - 0.5, pointPosMeas());
}

TEST_F(RtkEkfEstimatorTest, MeasBeforeDelayedThrowError)
{
    const int OVERFLOW = 3;
    const int times = est.states_.capacity() + OVERFLOW;
    EXPECT_CALL(cb_, dynamicsCb(_)).Times(times);

    const double dt = 0.1;
    UTCTime t(dt);
    for (int i = 0; i < times; ++i)
    {
        est.addMeasurement(t, ImuMeas::Zero());
        t += dt;
    }

    EXPECT_NOK(est.addMeasurement(UTCTime(0.3), pointPosMeas()));
}

TEST_F(RtkEkfEstimatorTest, MeasOnTopRewind)
{
    // t:       .1  .2  .3  .4  .5
    // IMU      x   x   x   x   x
    // PntPos           x<---
    //                   --->

    {
        InSequence seq;

        // normal progression
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.3).quantized())));

        // catchup
        EXPECT_CALL(cb_, pointPosCb(Eq(UTCTime(0.3))));
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.3).quantized())));

        // finish prop
        EXPECT_CALL(cb_, dynamicsCb(Eq(UTCTime(0.4))));
    }
    pointPosMeas m;
    m.z.setZero();
    est.addMeasurement(UTCTime(0.1), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.2), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.3), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.4), ImuMeas::Zero());
    est.addMeasurement(UTCTime(0.3), m);
    est.addMeasurement(UTCTime(0.5), ImuMeas::Zero());

    auto it = est.measurements_.begin();
    EXPECT_EQ((it++)->t, UTCTime(0.1));
    EXPECT_EQ((it++)->t, UTCTime(0.2));
    EXPECT_EQ((it++)->t, UTCTime(0.3));
    EXPECT_EQ((it++)->t, UTCTime(0.3));
    EXPECT_EQ((it++)->t, UTCTime(0.4));
    EXPECT_EQ((it++)->t, UTCTime(0.5));
    EXPECT_EQ(it, est.measurements_.end());
    EXPECT_EQ(est.t(), UTCTime(0.5));
}

}  // namespace ekf
}  // namespace mc

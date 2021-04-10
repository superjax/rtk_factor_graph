#include "core/ekf/rtk_ekf_estimator.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <variant>

#include "common/test_helpers.h"

namespace mc {
namespace ekf {

using ::testing::_;
using ::testing::Eq;
using ::testing::InSequence;

class MockEkf : public RtkEkfBase<MockEkf>
{
 public:
    MockEkf() = default;

    template <typename MeasType, typename... Args>
    typename MeasType::Residual h(const typename MeasType::ZType& z,
                                  const State& x,
                                  typename MeasType::Jac* jac,
                                  const Args&... args) const
    {
        if constexpr (std::is_same_v<MeasType, pointPosMeas>)
        {
            pointPosCb(x.t);
        }
        if constexpr (std::is_same_v<MeasType, gpsObsMeas>)
        {
            gpsObsCb(x.t);
        }
        if constexpr (std::is_same_v<MeasType, galObsMeas>)
        {
            galObsCb(x.t);
        }
        if constexpr (std::is_same_v<MeasType, gloObsMeas>)
        {
            gloObsCb(x.t);
        }
        if constexpr (std::is_same_v<MeasType, fixAndHoldMeas>)
        {
            fixAndHoldCb(x.t);
        }
        return MeasType::Residual::Zero();
    }

    Vec3 p_e_g2e(const State& x) const { return x.T_I2e.transformp(x.pose.transforma(x.p_b2g)); }

    ErrorState errorStateDynamics(ErrorState, State, Input, Input) const
    {
        errorStateDynCb();
        return ErrorState::Zero();
    }

    ErrorState dynamics(const State& x, Input, MockEkf::StateJac*, MockEkf::InputJac*) const
    {
        dynamicsCb(x.t);
        return ErrorState::Zero();
    }

    MOCK_CONST_METHOD1(pointPosCb, void(const UTCTime& t));
    MOCK_CONST_METHOD1(gpsObsCb, void(const UTCTime& t));
    MOCK_CONST_METHOD1(galObsCb, void(const UTCTime& t));
    MOCK_CONST_METHOD1(gloObsCb, void(const UTCTime& t));
    MOCK_CONST_METHOD1(fixAndHoldCb, void(const UTCTime& t));

    MOCK_CONST_METHOD0(errorStateDynCb, void());
    MOCK_CONST_METHOD1(dynamicsCb, void(const UTCTime& t));
};

using MockEstimator = EkfEstimator<MockEkf>;

class RtkEkfEstimatorTest : public ::testing::Test
{
 public:
    void SetUp() override
    {
        UTCTime t0 = UTCTime(0);
        State x0 = State::Identity();
        dxVec P0 = dxVec::Ones();
        Vec6 point_pos_cov = Vec6::Ones();
        Vec2 gps_obs_cov = Vec2::Ones();
        Vec2 gal_obs_cov = Vec2::Ones();
        Vec2 glo_obs_cov = Vec2::Ones();
        double fix_and_hold_cov = 1.0;
        Vec6 imu_cov = Vec6::Ones();
        Vec<ErrorState::SIZE> process_cov = Vec<ErrorState::SIZE>::Ones();

        est.init(t0, x0, P0, point_pos_cov, gps_obs_cov, gal_obs_cov, glo_obs_cov, fix_and_hold_cov,
                 imu_cov, process_cov);
    }
    MockEstimator est;
};

TEST_F(RtkEkfEstimatorTest, IntegrateNegative)
{
    est.addMeasurement(UTCTime(-0.1), ImuMeas());
}

TEST_F(RtkEkfEstimatorTest, InOrderIMU)
{
    // t:       .1  .2  .3  .4  .5
    // IMU      x   x   x   x   x
    {
        InSequence seq;

        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.4).quantized())));
    }

    est.addMeasurement(UTCTime(0.1), ImuMeas());
    est.addMeasurement(UTCTime(0.2), ImuMeas());
    est.addMeasurement(UTCTime(0.3), ImuMeas());
    est.addMeasurement(UTCTime(0.4), ImuMeas());
    est.addMeasurement(UTCTime(0.5), ImuMeas());

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

        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(est.ekf_, pointPosCb(Eq(UTCTime(0.35).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.35).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.4).quantized())));
    }

    pointPosMeas m;
    m.z.setZero();

    est.addMeasurement(UTCTime(0.1), ImuMeas());
    est.addMeasurement(UTCTime(0.2), ImuMeas());
    est.addMeasurement(UTCTime(0.3), ImuMeas());
    est.addMeasurement(UTCTime(0.35), m);
    est.addMeasurement(UTCTime(0.4), ImuMeas());
    est.addMeasurement(UTCTime(0.5), ImuMeas());

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

        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(est.ekf_, pointPosCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.4).quantized())));
    }

    pointPosMeas m;
    m.z.setZero();

    est.addMeasurement(UTCTime(0.1), ImuMeas());
    est.addMeasurement(UTCTime(0.2), ImuMeas());
    est.addMeasurement(UTCTime(0.3), ImuMeas());
    est.addMeasurement(UTCTime(0.3), m);
    est.addMeasurement(UTCTime(0.4), ImuMeas());
    est.addMeasurement(UTCTime(0.5), ImuMeas());

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
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));

        // catchup
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(est.ekf_, pointPosCb(Eq(UTCTime(0.25).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.25).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.4).quantized())));
    }

    pointPosMeas m;
    m.z.setZero();

    est.addMeasurement(UTCTime(0.1), ImuMeas());
    est.addMeasurement(UTCTime(0.2), ImuMeas());
    est.addMeasurement(UTCTime(0.3), ImuMeas());
    est.addMeasurement(UTCTime(0.4), ImuMeas());
    est.addMeasurement(UTCTime(0.25), m);
    est.addMeasurement(UTCTime(0.5), ImuMeas());

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
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));

        // catchup
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(est.ekf_, pointPosCb(Eq(UTCTime(0.25).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.25).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.4).quantized())));

        // catchup #2
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(est.ekf_, pointPosCb(Eq(UTCTime(0.35).quantized())));

        // finish prop
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.35).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.4).quantized())));
    }

    pointPosMeas m;
    m.z.setZero();

    est.addMeasurement(UTCTime(0.1), ImuMeas());
    est.addMeasurement(UTCTime(0.2), ImuMeas());
    est.addMeasurement(UTCTime(0.3), ImuMeas());
    est.addMeasurement(UTCTime(0.4), ImuMeas());
    est.addMeasurement(UTCTime(0.25), m);
    est.addMeasurement(UTCTime(0.5), ImuMeas());
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
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));

        // catchup
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(est.ekf_, pointPosCb(Eq(UTCTime(0.25).quantized())));

        // finish prop
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.25).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));

        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.4).quantized())));

        // delayed->catchup
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(est.ekf_, pointPosCb(Eq(UTCTime(0.15).quantized())));

        // finish prop
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.15).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.20).quantized())));
        EXPECT_CALL(est.ekf_, pointPosCb(Eq(UTCTime(0.25).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.25).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.4).quantized())));

        // Final in-order measurement
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.5).quantized())));
    }

    pointPosMeas m;
    m.z.setZero();

    est.addMeasurement(UTCTime(0.1), ImuMeas());
    est.addMeasurement(UTCTime(0.2), ImuMeas());
    est.addMeasurement(UTCTime(0.3), ImuMeas());
    est.addMeasurement(UTCTime(0.4), ImuMeas());
    est.addMeasurement(UTCTime(0.25), m);
    est.addMeasurement(UTCTime(0.5), ImuMeas());
    est.addMeasurement(UTCTime(0.15), m);
    est.addMeasurement(UTCTime(0.6), ImuMeas());

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
    EXPECT_CALL(est.ekf_, dynamicsCb(_)).Times(times);

    const double dt = 0.1;
    UTCTime t(dt);
    for (int i = 0; i < times; ++i)
    {
        est.addMeasurement(t, ImuMeas());
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

        EXPECT_CALL(est.ekf_, dynamicsCb(_)).Times(times);
        EXPECT_CALL(est.ekf_, pointPosCb(_));
        EXPECT_CALL(est.ekf_, dynamicsCb(_)).Times(5);
    }

    const double dt = 0.1;
    UTCTime t(dt);
    for (int i = 0; i < times; ++i)
    {
        est.addMeasurement(t, ImuMeas());
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
    EXPECT_CALL(est.ekf_, dynamicsCb(_)).Times(times);

    const double dt = 0.1;
    UTCTime t(dt);
    for (int i = 0; i < times; ++i)
    {
        est.addMeasurement(t, ImuMeas());
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
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));

        // catchup
        EXPECT_CALL(est.ekf_, pointPosCb(Eq(UTCTime(0.3))));
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));

        // finish prop
        EXPECT_CALL(est.ekf_, dynamicsCb(Eq(UTCTime(0.4))));
    }
    pointPosMeas m;
    m.z.setZero();
    est.addMeasurement(UTCTime(0.1), ImuMeas());
    est.addMeasurement(UTCTime(0.2), ImuMeas());
    est.addMeasurement(UTCTime(0.3), ImuMeas());
    est.addMeasurement(UTCTime(0.4), ImuMeas());
    est.addMeasurement(UTCTime(0.3), m);
    est.addMeasurement(UTCTime(0.5), ImuMeas());

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

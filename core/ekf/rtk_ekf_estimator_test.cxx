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
    MockEkf& operator=(const MockEkf& other)
    {
        x_ = other.x_;
        u_ = other.u_;
        cov_ = other.cov_;
        return *this;
    }
    template <typename MeasType, typename... Args>
    typename MeasType::Residual h(const typename MeasType::ZType& z,
                                  const State& x,
                                  typename MeasType::Jac* jac,
                                  const Args&... args) const
    {
        if constexpr (std::is_same_v<MeasType, pointPosMeas>)
        {
            pointPosCb(x_.t);
        }
        if constexpr (std::is_same_v<MeasType, gpsObsMeas>)
        {
            gpsObsCb(x_.t);
        }
        if constexpr (std::is_same_v<MeasType, galObsMeas>)
        {
            galObsCb(x_.t);
        }
        if constexpr (std::is_same_v<MeasType, gloObsMeas>)
        {
            gloObsCb(x_.t);
        }
        if constexpr (std::is_same_v<MeasType, fixAndHoldMeas>)
        {
            fixAndHoldCb(x_.t);
        }
        return MeasType::Residual::Zero();
    }

    Vec3 p_e_g2e() const { return x().T_I2e.transformp(x().pose.transforma(x().p_b2g)); }

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
        Vec<60> process_cov = Vec<60>::Ones();

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

        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.4).quantized())));
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
    EXPECT_EQ(est.catchup_ekf_.t(), UTCTime(0));
    EXPECT_EQ(est.delayed_ekf_.t(), UTCTime(0));
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

        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(est.propagated_ekf_, pointPosCb(Eq(UTCTime(0.35).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.35).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.4).quantized())));
    }

    est.addMeasurement(UTCTime(0.1), ImuMeas());
    est.addMeasurement(UTCTime(0.2), ImuMeas());
    est.addMeasurement(UTCTime(0.3), ImuMeas());
    est.addMeasurement(UTCTime(0.35), pointPosMeas());
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
    EXPECT_EQ(est.catchup_ekf_.t(), UTCTime(0));
    EXPECT_EQ(est.delayed_ekf_.t(), UTCTime(0));
}

TEST_F(RtkEkfEstimatorTest, OnTopOfMeasurements)
{
    // t:       .1  .2  .3  .4  .5
    // IMU      x   x   x   x   x
    // PntPos           x
    {
        InSequence seq;

        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(est.propagated_ekf_, pointPosCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.4).quantized())));
    }

    est.addMeasurement(UTCTime(0.1), ImuMeas());
    est.addMeasurement(UTCTime(0.2), ImuMeas());
    est.addMeasurement(UTCTime(0.3), ImuMeas());
    est.addMeasurement(UTCTime(0.3), pointPosMeas());
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
    EXPECT_EQ(est.catchup_ekf_.t(), UTCTime(0));
    EXPECT_EQ(est.delayed_ekf_.t(), UTCTime(0));
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
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));

        // catchup
        EXPECT_CALL(est.catchup_ekf_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(est.catchup_ekf_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(est.catchup_ekf_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(est.catchup_ekf_, pointPosCb(Eq(UTCTime(0.25).quantized())));

        // finish prop
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.25).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));

        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.4).quantized())));
    }
    est.addMeasurement(UTCTime(0.1), ImuMeas());
    est.addMeasurement(UTCTime(0.2), ImuMeas());
    est.addMeasurement(UTCTime(0.3), ImuMeas());
    est.addMeasurement(UTCTime(0.4), ImuMeas());
    est.addMeasurement(UTCTime(0.25), pointPosMeas());
    est.addMeasurement(UTCTime(0.5), ImuMeas());

    auto it = est.measurements_.begin();
    EXPECT_EQ((it++)->t, UTCTime(0.1));
    EXPECT_EQ((it++)->t, UTCTime(0.2));
    EXPECT_EQ((it++)->t, UTCTime(0.25));
    EXPECT_EQ((it++)->t, UTCTime(0.3));
    EXPECT_EQ((it++)->t, UTCTime(0.4));
    EXPECT_EQ((it++)->t, UTCTime(0.5));
    EXPECT_EQ(it, est.measurements_.end());
    EXPECT_EQ(est.catchup_ekf_.t(), UTCTime(0.25));
    EXPECT_EQ(est.delayed_ekf_.t(), UTCTime(0));
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
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));

        // catchup
        EXPECT_CALL(est.catchup_ekf_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(est.catchup_ekf_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(est.catchup_ekf_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(est.catchup_ekf_, pointPosCb(Eq(UTCTime(0.25).quantized())));

        // finish prop
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.25).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));

        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.4).quantized())));

        // catchup #2
        EXPECT_CALL(est.catchup_ekf_, dynamicsCb(Eq(UTCTime(0.25).quantized())));
        EXPECT_CALL(est.catchup_ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(est.catchup_ekf_, pointPosCb(Eq(UTCTime(0.35).quantized())));

        // finish prop
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.35).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.4).quantized())));
    }
    est.addMeasurement(UTCTime(0.1), ImuMeas());
    est.addMeasurement(UTCTime(0.2), ImuMeas());
    est.addMeasurement(UTCTime(0.3), ImuMeas());
    est.addMeasurement(UTCTime(0.4), ImuMeas());
    est.addMeasurement(UTCTime(0.25), pointPosMeas());
    est.addMeasurement(UTCTime(0.5), ImuMeas());
    est.addMeasurement(UTCTime(0.35), pointPosMeas());

    auto it = est.measurements_.begin();
    EXPECT_EQ((it++)->t, UTCTime(0.1));
    EXPECT_EQ((it++)->t, UTCTime(0.2));
    EXPECT_EQ((it++)->t, UTCTime(0.25));
    EXPECT_EQ((it++)->t, UTCTime(0.3));
    EXPECT_EQ((it++)->t, UTCTime(0.35));
    EXPECT_EQ((it++)->t, UTCTime(0.4));
    EXPECT_EQ((it++)->t, UTCTime(0.5));
    EXPECT_EQ(it, est.measurements_.end());
    EXPECT_EQ(est.catchup_ekf_.t(), UTCTime(0.35));
    EXPECT_EQ(est.delayed_ekf_.t(), UTCTime(0));
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
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));

        // catchup
        EXPECT_CALL(est.catchup_ekf_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(est.catchup_ekf_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(est.catchup_ekf_, dynamicsCb(Eq(UTCTime(0.2).quantized())));
        EXPECT_CALL(est.catchup_ekf_, pointPosCb(Eq(UTCTime(0.25).quantized())));

        // finish prop
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.25).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));

        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.4).quantized())));

        // delayed->catchup
        EXPECT_CALL(est.catchup_ekf_, dynamicsCb(Eq(UTCTime(0.0).quantized())));
        EXPECT_CALL(est.catchup_ekf_, dynamicsCb(Eq(UTCTime(0.1).quantized())));
        EXPECT_CALL(est.catchup_ekf_, pointPosCb(Eq(UTCTime(0.15).quantized())));

        // finish prop
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.15).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.20).quantized())));
        EXPECT_CALL(est.propagated_ekf_, pointPosCb(Eq(UTCTime(0.25).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.25).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.4).quantized())));

        // Final in-order measurement
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.5).quantized())));
    }
    est.addMeasurement(UTCTime(0.1), ImuMeas());
    est.addMeasurement(UTCTime(0.2), ImuMeas());
    est.addMeasurement(UTCTime(0.3), ImuMeas());
    est.addMeasurement(UTCTime(0.4), ImuMeas());
    est.addMeasurement(UTCTime(0.25), pointPosMeas());
    est.addMeasurement(UTCTime(0.5), ImuMeas());
    est.addMeasurement(UTCTime(0.15), pointPosMeas());
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
    EXPECT_EQ(est.catchup_ekf_.t(), UTCTime(0.15));
    EXPECT_EQ(est.delayed_ekf_.t(), UTCTime(0));
}

TEST_F(RtkEkfEstimatorTest, MoveDelayed)
{
    EXPECT_CALL(est.propagated_ekf_, dynamicsCb(_)).Times(15);
    EXPECT_CALL(est.delayed_ekf_, dynamicsCb(_)).Times(4);

    const double dt = 0.1;
    UTCTime t(0.1);
    for (int i = 0; i < 15; ++i)
    {
        est.addMeasurement(t, ImuMeas());
        t += dt;
    }

    auto it = est.measurements_.begin();
    EXPECT_EQ(it->t, est.delayed_ekf_.t() + dt);
    EXPECT_EQ((it++)->t, UTCTime(0.5));
    EXPECT_EQ((it++)->t, UTCTime(0.6));
    EXPECT_EQ((it++)->t, UTCTime(0.7));
    EXPECT_EQ((it++)->t, UTCTime(0.8));
    EXPECT_EQ((it++)->t, UTCTime(0.9));
    EXPECT_EQ((it++)->t, UTCTime(1.0));
    EXPECT_EQ((it++)->t, UTCTime(1.1));
    EXPECT_EQ((it++)->t, UTCTime(1.2));
    EXPECT_EQ((it++)->t, UTCTime(1.3));
    EXPECT_EQ((it++)->t, UTCTime(1.4));
    EXPECT_EQ((it++)->t, UTCTime(1.5));
    EXPECT_EQ(est.propagated_ekf_.t(), UTCTime(1.5));
    EXPECT_EQ(it, est.measurements_.end());
    EXPECT_EQ(est.catchup_ekf_.t(), est.delayed_ekf_.t());
}

TEST_F(RtkEkfEstimatorTest, RewindAfterMoveDelay)
{
    EXPECT_CALL(est.propagated_ekf_, dynamicsCb(_)).Times(15);
    EXPECT_CALL(est.delayed_ekf_, dynamicsCb(_)).Times(4);
    {
        InSequence seq;

        EXPECT_CALL(est.catchup_ekf_, dynamicsCb(_)).Times(5);
        EXPECT_CALL(est.catchup_ekf_, pointPosCb(Eq(UTCTime(0.85))));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(_)).Times(7);
    }

    const double dt = 0.1;
    UTCTime t(0.1);
    for (int i = 0; i < 15; ++i)
    {
        est.addMeasurement(t, ImuMeas());
        t += dt;
    }
    est.addMeasurement(UTCTime(0.85), pointPosMeas());

    auto it = est.measurements_.begin();
    EXPECT_EQ(it->t, est.delayed_ekf_.t() + dt);
    EXPECT_EQ((it++)->t, UTCTime(0.5));
    EXPECT_EQ((it++)->t, UTCTime(0.6));
    EXPECT_EQ((it++)->t, UTCTime(0.7));
    EXPECT_EQ((it++)->t, UTCTime(0.8));
    EXPECT_EQ((it++)->t, UTCTime(0.85));
    EXPECT_EQ((it++)->t, UTCTime(0.9));
    EXPECT_EQ((it++)->t, UTCTime(1.0));
    EXPECT_EQ((it++)->t, UTCTime(1.1));
    EXPECT_EQ((it++)->t, UTCTime(1.2));
    EXPECT_EQ((it++)->t, UTCTime(1.3));
    EXPECT_EQ((it++)->t, UTCTime(1.4));
    EXPECT_EQ((it++)->t, UTCTime(1.5));
    EXPECT_EQ(est.propagated_ekf_.t(), UTCTime(1.5));
    EXPECT_EQ(it, est.measurements_.end());
    EXPECT_EQ(est.catchup_ekf_.t(), UTCTime(0.85));
}

TEST_F(RtkEkfEstimatorTest, MeasBeforeDelayedThrowError)
{
    EXPECT_CALL(est.propagated_ekf_, dynamicsCb(_)).Times(15);
    EXPECT_CALL(est.delayed_ekf_, dynamicsCb(_)).Times(4);

    const double dt = 0.1;
    UTCTime t(0.1);
    for (int i = 0; i < 15; ++i)
    {
        est.addMeasurement(t, ImuMeas());
        t += dt;
    }
    EXPECT_NOK(est.addMeasurement(UTCTime(0.3), pointPosMeas()));
}

TEST_F(RtkEkfEstimatorTest, MeasOnTopOfDelayed)
{
    EXPECT_CALL(est.propagated_ekf_, dynamicsCb(_)).Times(15);
    EXPECT_CALL(est.delayed_ekf_, dynamicsCb(_)).Times(4);

    const double dt = 0.1;
    UTCTime t(0.1);
    for (int i = 0; i < 15; ++i)
    {
        est.addMeasurement(t, ImuMeas());
        t += dt;
    }

    {
        InSequence seq;

        EXPECT_CALL(est.catchup_ekf_, pointPosCb(Eq(UTCTime(0.4))));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(_)).Times(11);
        EXPECT_CALL(est.delayed_ekf_, pointPosCb(Eq(UTCTime(0.4))));
    }
    EXPECT_OK(est.addMeasurement(UTCTime(0.4), pointPosMeas()));
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
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(_)).Times(4);

        // catchup
        EXPECT_CALL(est.catchup_ekf_, dynamicsCb(_)).Times(3);
        EXPECT_CALL(est.catchup_ekf_, pointPosCb(Eq(UTCTime(0.3).quantized())));

        // finish prop
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));

        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.4).quantized())));
    }
    est.addMeasurement(UTCTime(0.1), ImuMeas());
    est.addMeasurement(UTCTime(0.2), ImuMeas());
    est.addMeasurement(UTCTime(0.3), ImuMeas());
    est.addMeasurement(UTCTime(0.4), ImuMeas());
    est.addMeasurement(UTCTime(0.3), pointPosMeas());
    est.addMeasurement(UTCTime(0.5), ImuMeas());

    auto it = est.measurements_.begin();
    EXPECT_EQ((it++)->t, UTCTime(0.1));
    EXPECT_EQ((it++)->t, UTCTime(0.2));
    EXPECT_EQ((it++)->t, UTCTime(0.3));
    EXPECT_EQ((it++)->t, UTCTime(0.3));
    EXPECT_EQ((it++)->t, UTCTime(0.4));
    EXPECT_EQ((it++)->t, UTCTime(0.5));
    EXPECT_EQ(it, est.measurements_.end());
    EXPECT_EQ(est.catchup_ekf_.t(), UTCTime(0.3));
    EXPECT_EQ(est.delayed_ekf_.t(), UTCTime(0));
}

TEST_F(RtkEkfEstimatorTest, MeasOnTopDoubleRewind)
{
    // t:       .1  .2  .3  .4  .5
    // IMU      x   x   x   x   x
    // PntPos         x<-----
    //                 ----->
    // PntPos         x
    //                 ----->
    {
        InSequence seq;

        // normal progression
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(_)).Times(4);

        // catchup
        EXPECT_CALL(est.catchup_ekf_, dynamicsCb(_)).Times(3);
        EXPECT_CALL(est.catchup_ekf_, pointPosCb(Eq(UTCTime(0.3).quantized())));

        // finish prop
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.4).quantized())));

        // catchup again
        EXPECT_CALL(est.catchup_ekf_, pointPosCb(Eq(UTCTime(0.3).quantized())));

        // propagate again
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.3).quantized())));
        EXPECT_CALL(est.propagated_ekf_, dynamicsCb(Eq(UTCTime(0.4).quantized())));
    }
    est.addMeasurement(UTCTime(0.1), ImuMeas());
    est.addMeasurement(UTCTime(0.2), ImuMeas());
    est.addMeasurement(UTCTime(0.3), ImuMeas());
    est.addMeasurement(UTCTime(0.4), ImuMeas());
    est.addMeasurement(UTCTime(0.3), pointPosMeas());
    est.addMeasurement(UTCTime(0.5), ImuMeas());
    est.addMeasurement(UTCTime(0.3), pointPosMeas());

    auto it = est.measurements_.begin();
    EXPECT_EQ((it++)->t, UTCTime(0.1));
    EXPECT_EQ((it++)->t, UTCTime(0.2));
    EXPECT_EQ((it++)->t, UTCTime(0.3));
    EXPECT_EQ((it++)->t, UTCTime(0.3));
    EXPECT_EQ((it++)->t, UTCTime(0.3));
    EXPECT_EQ((it++)->t, UTCTime(0.4));
    EXPECT_EQ((it++)->t, UTCTime(0.5));
    EXPECT_EQ(it, est.measurements_.end());
    EXPECT_EQ(est.catchup_ekf_.t(), UTCTime(0.3));
    EXPECT_EQ(est.delayed_ekf_.t(), UTCTime(0));
}

}  // namespace ekf
}  // namespace mc

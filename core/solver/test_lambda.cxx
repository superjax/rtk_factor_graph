#include <gtest/gtest.h>

#include <Eigen/LU>
#include <limits>

#include "common/matrix_defs.h"
#include "common/test_helpers.h"
#include "core/solver/lambda.h"

namespace mc {
namespace core {
namespace solver {

using namespace Eigen;

#define T transpose()

TEST(Argsort, SortVector)
{
    VectorXd v(5);
    for (int i = 0; i < 1000; ++i)
    {
        v.setRandom();
        VectorXd sort_order = argsort(v);
        double prev_value = std::numeric_limits<double>::lowest();
        const VectorXd sorted = sort_order.asPermutation() * v;
        for (int j = 0; j < v.rows(); ++j)
        {
            EXPECT_LT(prev_value, sorted(j));
            prev_value = sorted(j);
        }
    }
}

TEST(Argsort, SortMatrix)
{
    // clang-format off
    const Mat4 A = (Mat4() <<   0.680375,   0.823295,  -0.444451,  -0.270431,
                               -0.211234,  -0.604897,    0.10794,  0.0268018,
                                0.566198,  -0.329554, -0.0452059,   0.904459,
                                 0.59688,   0.536459,   0.257742,    0.83239).finished();
    // clang-format on
    const VectorXd row_sort_order = argsort(A.col(2));
    const VectorXd col_sort_order = argsort(A.row(2));

    // clang-format off
    const Mat4 row_sorted = (Mat4() <<   0.680375,   0.823295,  -0.444451,  -0.270431,
                                         0.566198,  -0.329554, -0.0452059,   0.904459,
                                        -0.211234,  -0.604897,    0.10794,  0.0268018,
                                          0.59688,   0.536459,   0.257742,    0.83239).finished();
    const Mat4 col_sorted = (Mat4() <<   0.823295,  -0.444451,  0.680375,  -0.270431,
                                        -0.604897,    0.10794, -0.211234,  0.0268018,
                                        -0.329554, -0.0452059,  0.566198,   0.904459,
                                         0.536459,   0.257742,   0.59688,    0.83239).finished();
    // clang-format on
    MAT_EQ(row_sorted, row_sort_order.asPermutation() * A);
    MAT_EQ(col_sorted, A * col_sort_order.asPermutation().transpose());
}

TEST(MatrixTools, modf)
{
    const Mat4 A = 100 * Mat4::Random();

    const Mat4 Amod = modf(A);

    Mat4 int_part;
    const Mat4 Amod2 = modf(A, &int_part);

    EXPECT_TRUE((Amod.array().abs() < 1.0).all());
    MAT_EQ(Amod, Amod2);
    MAT_EQ(Amod2 + int_part, A);
    MAT_EQ(int_part.cast<int>(), int_part);
}

TEST(Lambda, LTDL)
{
    Mat5 Q = Mat5::Random();
    Q = Q * Q.transpose();  // make a positive definite matrix
    Mat5 L(5, 5);
    Vec5 D(5, 1);
    LTDL(Q, L, D);

    MAT_EQ(Q, L.transpose() * D.asDiagonal() * L);
}

TEST(Lambda, integerGaussTransformation)
{
    Mat5 L = Mat5::Random() * 10;
    L.triangularView<Eigen::Upper>().setZero();
    L.diagonal().setConstant(1.0);
    Vec5 D = Vec5::Random();
    Vec5 a = Vec5::Random();
    Mat5 Z;
    Z.setIdentity();

    const Mat5 Q = L.transpose() * D.asDiagonal() * L;
    const Vec5 a_orig = a;

    for (int i = 0; i < 5; ++i)
    {
        for (int j = 0; j < i; ++j)
        {
            integerGaussTransformations(i, j, L, a, Z);

            const Mat5 LZ = L * Z.cast<double>();

            // Compute Qz
            const Mat5 Qz = L.transpose() * D.asDiagonal() * L;

            // Compute Qa from Qz using Z⁻¹
            const Eigen::Matrix<double, 5, 5> Zinv = Z.cast<double>().inverse();
            const Mat5 Qhat = Zinv.transpose() * Qz * Zinv;
            const Vec5 ahat = Zinv.transpose() * a;

            EXPECT_GT(abs(LZ(i, j)), 0.5);
            MAT_EQ(Q, Qhat);
            MAT_EQ(a_orig, ahat);
        }
    }

    EXPECT_TRUE(((L - Mat5::Identity()).array() < 0.5).all());
}

TEST(Lambda, permute)
{
    Mat5 L = Mat5::Random() * 10;
    L.triangularView<Eigen::Upper>().setZero();
    L.diagonal().setConstant(1.0);
    Vec5 D = Vec5::Random();
    Vec5 a = Vec5::Random();
    Mat5 Z;
    Z.setIdentity();

    for (int k = 0; k < 4; ++k)
    {
        const double diff = D(k + 1) - D(k);
        const double delta = D(k) + L(k + 1, k) * L(k + 1, k) * D(k + 1);
        if (delta < D(k + 1))
        {
            permute(k, delta, a, L, D, Z);
            const double new_diff = D(k + 1) - D(k);
            EXPECT_LT(new_diff, diff);
        }
    }
}

TEST(Lambda, reduce)
{
    // clang-format off
    const Mat3 Qahat =
    (Mat3() << 6.2900, 5.9780, 0.5440,
             5.9780, 6.2920, 2.3400,
             0.5440, 2.3400, 6.2880).finished();
    // clang-format on
    const Vec3 ahat(0.45, 0.1, 0.97);

    Mat3 L;
    Mat3 Z;
    Vec3 zhat;
    Vec3 D;
    reduction(Qahat, ahat, zhat, L, D, Z);

    const Mat3 Qzhat = Z.cast<double>().transpose() * Qahat * Z.cast<double>();

    // clang-format off
    const Mat3 Qzhat_true = (Mat3() << 4.476, 0.334, 0.23,
                                       0.334, 1.146, 0.082,
                                       0.23,  0.082, 0.626).finished();
    const Mat3 L_true = (Mat3() << 1.,         0.,         0.,
                                   0.26766778, 1.,         0.,
                                   0.36741214, 0.13099042, 1.).finished();
    const Vec3 D_true(4.31015841, 1.13525879, 0.626);
    const Vec3 zhat_true(-1.57, 2.02, 0.35);
    const Mat3 Z_true = (Mat3() << -2.,  3.,  1.,
                                    3., -3., -1.,
                                   -1.,  1.,  0.).finished();
    // clang-format on

    MATRIX_CLOSE(Qzhat, Qzhat_true, 1e-3);
    MATRIX_CLOSE(L, L_true, 1e-7);
    MATRIX_CLOSE(D, D_true, 1e-5);
    MATRIX_CLOSE(Z.cast<double>().transpose() * ahat, zhat_true, 1e-3);
    MAT_EQ(Z, Z_true);
}

TEST(Lambda, search)
{
    // clang-format off
    const Mat3 Qahat =
    (Mat3() << 6.2900, 5.9780, 0.5440,
             5.9780, 6.2920, 2.3400,
             0.5440, 2.3400, 6.2880).finished();
    // clang-format on
    const Vec3 ahat(0.45, 0.1, 0.97);

    Mat3 L;
    Mat3 Z;
    Vec3 zhat;
    Vec3 D;
    reduction(Qahat, ahat, zhat, L, D, Z);
    MatrixXd z_fixed(3, 3);
    VectorXd sqrd_norms(3);
    search(zhat, L, D, 3, z_fixed, sqrd_norms);

    // clang-format off
    const Matrix3i answer = (Matrix3i() << -2, -1, -3,
                                            2,  2,  2,
                                            0,  0,  0).finished();
    const Vector3d answer_sqrd_norms(0.2183311, 0.30727258, 0.59340968);
    // clang-format on

    MAT_EQ(answer, z_fixed);
    MATRIX_CLOSE(answer_sqrd_norms, sqrd_norms, 1e-7);
}

TEST(Lambda, lambda_small)
{
    // clang-format off
    const Mat3 Qahat =
    (Mat3() << 6.2900, 5.9780, 0.5440,
             5.9780, 6.2920, 2.3400,
             0.5440, 2.3400, 6.2880).finished();
    // clang-format on
    Vec3 ahat(5.4500, 3.1000, 2.9700);

    Eigen::VectorXd a_fixed(3);
    double ratio;

    lambda(ahat, Qahat, a_fixed, Out(ratio));

    const Matrix<int, 3, 1> oracle_a_fixed(5, 3, 4);
    const double oracle_ratio = 0.7105453351162826;

    EXPECT_NEAR(oracle_ratio, ratio, 1e-8);
    MAT_EQ(oracle_a_fixed, a_fixed);
}

TEST(Lambda, lambda_medium)
{
    // clang-format off
    const Mat6 Qahat = (Mat6() <<  19068.8559508787, -15783.972282037000,  -17334.200587597500,  14411.9239749603000,  10055.71700893590, -14259.29529038720,
                                  -15783.9722820370,  59027.703840981500,   38142.692753110200,    562.7173880246450, -13830.08559606760,  27373.42630130190,
                                  -17334.2005875975,  38142.692753110200,   28177.565389352800,  -7000.5022049704500, -11695.86740593060,  21886.16806305320,
                                   14411.9239749603,    562.717388024645,   -7000.502204970450,  15605.5082283690000,   5039.70281815470,  -9648.96530646004,
                                   10055.7170089359, -13830.085596067600,  -11695.867405930600,   5039.7028181547000,   6820.77250679480,  -6880.24051213224,
                                  -14259.2952903872,  27373.426301301900,   21886.168063053200,  -9648.9653064600400,  -6880.24051213224,  23246.54896269450).finished();
    const Vec6 ahat = (Vec6() << -28490.8566886116, 65752.6299198198, 38830.3666554972, 5003.70833517778, -29196.0699104593, -297.658932458787).finished();
    // clang-format on

    Eigen::VectorXd a_fixed(6);
    double ratio;

    lambda(ahat, Qahat, a_fixed, Out(ratio));

    const Matrix<int, 6, 1> oracle_a_fixed =
        (Matrix<int, 6, 1>() << -28506, 65833, 38880, 5008, -29210, -257).finished();
    const double oracle_ratio = 0.8275167404788059;

    EXPECT_NEAR(oracle_ratio, ratio, 1e-8);
    MAT_EQ(oracle_a_fixed, a_fixed);
}

TEST(Lambda, lambda_big)
{
    // clang-format off
    const Mat12 Qahat = (Mat12() <<  19068.8559508787,	-15783.9722820370,	-17334.2005875975,	14411.9239749603,	10055.7170089359,	-14259.2952903872,	14858.8484050976,	-12299.1993741839,	-13507.1694819930,	11230.0704356810,	7835.62344938376,	-11111.1393808147,
                                     -15783.9722820370,	59027.7038409815,	38142.6927531102,	562.717388024645,	-13830.0855960676,	27373.4263013019,	-12299.1993747356,	45995.6129934030,	29721.5785731468,	438.480887460148,	-10776.6902686912,	21329.9423774758,
                                     -17334.2005875975,	38142.6927531102,	28177.5653893528,	-7000.50220497045,	-11695.8674059306,	21886.1680630532,	-13507.1694826246,	29721.5785738846,	21956.5440705992,	-5454.93697674992,	-9113.66310734779,	17054.1567378091,
                                     14411.9239749603,	562.717388024645,	-7000.50220497045,	15605.5082283690,	5039.70281815470,	-9648.96530646004,	11230.0704356773,	438.480887731461,	-5454.93697653627,	12160.1358938811,	3927.04096307733,	-7518.67445855756,
                                     10055.7170089359,	-13830.0855960676,	-11695.8674059306,	5039.70281815470,	6820.77250679480,	-6880.24051213224,	7835.62344947055,	-10776.6902682086,	-9113.66310687634,	3927.04096320258,	5314.88728015545,	-5361.22656658847,
                                     -14259.2952903872,	27373.4263013019,	21886.1680630532,	-9648.96530646004,	-6880.24051213224,	23246.5489626945,	-11111.1393809211,	21329.9423779274,	17054.1567375591,	-7518.67445829957,	-5361.22656681708,	18114.1936088811,
                                     14858.8484050976,	-12299.1993747356,	-13507.1694826246,	11230.0704356773,	7835.62344947055,	-11111.1393809211,	11578.3237340013,	-9583.79156943782,	-10525.0669778554,	8750.70438611838,	6105.68076067050,	-8658.03053539344,
                                     -12299.1993741839,	45995.6129934030,	29721.5785738846,	438.480887731461,	-10776.6902682086,	21329.9423779274,	-9583.79156943782,	35840.7376978353,	23159.6717654859,	341.673569568934,	-8397.42083743563,	16620.7344703582,
                                     -13507.1694819930,	29721.5785731468,	21956.5440705992,	-5454.93697653627,	-9113.66310687634,	17054.1567375591,	-10525.0669778554,	23159.6717654859,	17108.9956804894,	-4250.60009053988,	-7101.55551676305,	13288.9534523001,
                                     11230.0704356810,	438.480887460148,	-5454.93697674992,	12160.1358938811,	3927.04096320258,	-7518.67445829957,	8750.70438611838,	341.673569568934,	-4250.60009053988,	9475.43086798586,	3060.03207008500,	-5858.70721928591,
                                     7835.62344938376,	-10776.6902686912,	-9113.66310734779,	3927.04096307733,	5314.88728015545,	-5361.22656681708,	6105.68076067050,	-8397.42083743563,	-7101.55551676305,	3060.03207008500,	4141.47090961885,	-4177.57899193454,
                                     -11111.1393808147,	21329.9423774758,	17054.1567378091,	-7518.67445855756,	-5361.22656658847,	18114.1936088811,	-8658.03053539344,	16620.7344703582,	13288.9534523001,	-5858.70721928591,	-4177.57899193454,	14114.9563601479).finished();
    const Vec12 ahat = (Vec12() << -28490.8566886116,65752.6299198198,38830.3666554972,5003.70833517778,-29196.0699104593,-297.658932458787,-22201.0284440701,51235.8374755528,30257.7809603224,3899.40332138829,-22749.1853575113,-159.278779870217  ).finished();
    // clang-format on

    Eigen::VectorXd a_fixed(12);
    double ratio;

    lambda(ahat, Qahat, a_fixed, Out(ratio));

    const Matrix<int, 12, 1> oracle_a_fixed =
        (Matrix<int, 12, 1>() << -28451, 65749, 38814, 5025, -29165, -278, -22170, 51233, 30245,
         3916, -22725, -144)
            .finished();
    const double oracle_ratio = 0.4746843941521999;

    EXPECT_NEAR(oracle_ratio, ratio, 1e-8);
    MAT_EQ(oracle_a_fixed, a_fixed);
}

}  // namespace solver
}  // namespace core
}  // namespace mc

#include <Eigen/LU>

#include "common/defs.h"
#include "common/matrix_defs.h"
#include "core/solver/lambda.h"

namespace mc {
namespace core {
namespace solver {

#define T transpose()
using namespace Eigen;

void LTDL(const MatrixXd& _Q, Ref<MatrixXd> L, Ref<VectorXd> D)
{
    using std::sqrt;

    MatrixXd Q = _Q;  // make a working copy

    check(Q.rows() == Q.cols());
    check(L.rows() == L.cols());
    check(L.rows() == Q.rows());
    check(D.rows() == Q.rows());
    check(D.cols() == 1);

    const int N = Q.rows();

    L.triangularView<StrictlyUpper>().setZero();

    for (int i = N - 1; i >= 0; --i)
    {
        D(i) = Q(i, i);
        L.block(i, 0, 1, i + 1) = Q.block(i, 0, 1, i + 1) / sqrt(Q(i, i));
        for (int j = 0; j < i; ++j)
        {
            Q.block(j, 0, 1, j + 1) -= L.block(i, 0, 1, j + 1) * L(i, j);
        }
        L.block(i, 0, 1, i + 1) /= L(i, i);
    }

    check((D.array() > 0).all(), "LᵀDL Solution not positive definite");
}

void lambda(const VectorXd& a,
            const MatrixXd& Q,
            Ref<VectorXd> a_fixed,
            Out<double> ratio,
            int n_candidates)
{
    const int n = a.rows();

    // Pull integer offsets off the estimates (makes problem better conditioned)
    VectorXd incr(n);
    VectorXd ahat = modf(a, &incr);

    VectorXd zhat(n);
    VectorXd D(n);
    MatrixXd Qzhat(n, n);
    MatrixXd L(n, n);
    MatrixXd Z(n, n);
    reduction(Q, ahat, zhat, L, D, Z);

    MatrixXd candidates(n, n_candidates);
    VectorXd sqrd_norms(n_candidates);
    search(zhat, L, D, n_candidates, candidates, sqrd_norms);

    *ratio = sqrd_norms(0) / sqrd_norms(1);
    const MatrixXd Zinv = Z.inverse().array().round().matrix();
    a_fixed = Zinv.T * candidates.col(0);

    a_fixed += incr;
}

void integerGaussTransformations(int i, int j, Ref<MatrixXd> L, Ref<VectorXd> a, Ref<MatrixXd> Z)
{
    check(i > j);
    check(L.rows() == L.cols());
    check(Z.rows() == Z.cols());
    check(L.rows() == Z.rows());
    check(a.rows() == L.rows());
    check(a.cols() == 1);

    const int N = L.rows();
    const double mu = std::round(L(i, j));
    if (mu != 0)
    {
        L.block(i, j, N - i, 1) -= mu * L.block(i, i, N - i, 1);
        Z.col(j) -= mu * Z.col(i);
        a(j) -= mu * a(i);
    }
}

void permute(int k,
             double delta,
             Ref<VectorXd> a,
             Ref<MatrixXd> L,
             Ref<VectorXd> D,
             Ref<MatrixXd> Z)
{
    check(L.rows() == L.cols());
    check(Z.rows() == Z.cols());
    check(L.rows() == Z.rows());
    check(a.rows() == L.rows());
    check(D.rows() == a.rows());
    check(a.cols() == 1);
    check(D.cols() == 1);

    const int N = a.rows();
    const double lambda = D(k + 1) * L(k + 1, k) / delta;
    const double eta = D(k) / delta;
    D(k) = eta * D(k + 1);
    D(k + 1) = delta;
    const Mat2 tmp = (Mat2() << -L(k + 1, k), 1, eta, lambda).finished();
    L.block(k, 0, 2, k) = tmp * L.block(k, 0, 2, k);
    L(k + 1, k) = lambda;
    L.block(k + 2, k, N - (k + 2), 1).swap(L.block(k + 2, k + 1, N - (k + 2), 1));
    Z.col(k).swap(Z.col(k + 1));
    std::swap(a(k), (a(k + 1)));
}

void reduction(const MatrixXd& Q,
               const VectorXd& a,
               Ref<VectorXd> z,
               Ref<MatrixXd> L,
               Ref<VectorXd> D,
               Ref<MatrixXd> Z)
{
    check(Q.rows() == Q.cols());
    check(L.rows() == L.cols());
    check(Z.rows() == Z.cols());
    check(L.rows() == Z.rows());
    check(a.rows() == L.rows());
    check(D.rows() == a.rows());
    const int N = Q.rows();
    z = a;

    LTDL(Q, L, D);

    Z.setIdentity();
    int k1 = N - 2;
    bool sw = true;

    while (sw)
    {
        int k = N - 1;
        sw = false;
        while (!sw && k > 0)
        {
            k -= 1;
            if (k <= k1)
            {
                for (int j = k + 1; j < N; ++j)
                {
                    integerGaussTransformations(j, k, L, z, Z);
                }
            }
            const double delta = D(k) + L(k + 1, k) * L(k + 1, k) * D(k + 1);
            if (delta < D(k + 1))
            {
                permute(k, delta, z, L, D, Z);
                k1 = k;
                sw = true;
            }
        }
    }
}

void search(const VectorXd& ahat,
            const MatrixXd& L,
            const VectorXd& D,
            const int n_cands,
            Ref<MatrixXd> a_fixed,
            Ref<VectorXd> sqrd_norms)
{
    using std::round;
    check(ahat.rows() == L.rows());
    check(ahat.rows() == D.rows());
    check(L.rows() == L.cols());

    const int n = ahat.rows();
    const int end = n - 1;
    // a_fixed.resize(n, n_cands);
    a_fixed.setZero();
    // sqrd_norms.resize(n);
    sqrd_norms.setZero();

    double Chi2 = 1e18;
    VectorXd dist = VectorXd::Zero(n);
    bool end_search = false;
    int count = -1;  // number of candidates

    VectorXd acond = VectorXd::Zero(n);
    VectorXd zcond = VectorXd::Zero(n);
    VectorXd step = VectorXd::Zero(n);

    acond(end) = ahat(end);
    zcond(end) = round(acond(end));

    double left = acond(end) - zcond(end);
    step(end) = sign(left);

    if (step(end) == 0)
    {
        step(end) = 1;
    }

    int imax = n_cands - 1;
    MatrixXd S = MatrixXd::Zero(n, n);

    int k = end;

    while (!end_search)
    {
        double newdist = dist(k) + left * left / D(k);
        if (newdist < Chi2)
        {
            if (k != 0)
            {
                k -= 1;
                dist(k) = newdist;
                S.block(k, 0, 1, k + 1) =
                    S.block(k + 1, 0, 1, k + 1) +
                    (zcond(k + 1) - acond(k + 1)) * L.block(k + 1, 0, 1, k + 1);
                acond(k) = ahat(k) + S(k, k);
                zcond(k) = round(acond(k));
                left = acond(k) - zcond(k);
                step(k) = sign(left);

                if (step(k) == 0)
                {
                    step(k) = 1;
                }
            }
            else
            {
                // Case 2: store the found candidate and try next valid integer
                if (count < n_cands - 2)
                {
                    // Store the first n_cands-1 initial points as candidates
                    count += 1;
                    a_fixed.col(count) = zcond;
                    sqrd_norms(count) = newdist;
                }
                else
                {
                    a_fixed.col(imax) = zcond;
                    sqrd_norms(imax) = newdist;

                    sqrd_norms.maxCoeff(&imax);
                    Chi2 = sqrd_norms(imax);
                }
                zcond(0) += step(0);
                left = acond(0) - zcond(0);
                step(0) = -step(0) - sign(step(0));
            }
        }
        else
        {
            // Case 3 exit, or move up
            if (k == end)
            {
                end_search = true;
            }
            else
            {
                k += 1;
                zcond(k) += step(k);  // next valid integer
                left = acond(k) - zcond(k);
                step(k) = -step(k) - sign(step(k));
            }
        }
    }

    // sort the arrays from best to least best candidates
    const VectorXd order = argsort(sqrd_norms);
    const auto perm = order.asPermutation();
    sqrd_norms = perm * sqrd_norms;
    a_fixed = a_fixed * perm.T;
}

}  // namespace solver
}  // namespace core
}  // namespace mc

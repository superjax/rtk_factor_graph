#include "common/check.h"
#include "common/matrix_defs.h"
#include "common/out.h"

namespace mc {
namespace core {
namespace solver {

// solves
// z˜ = argmin_z (z-a)Q(z-a)ᵀ
// template <int N>
void lambda(const Eigen::VectorXd& a,
            const Eigen::MatrixXd& Q,
            Eigen::Ref<Eigen::VectorXd> a_fixed,
            Out<double> ratio,
            int n_candidates = 3);

void LTDL(const Eigen::MatrixXd& _Q, Eigen::Ref<Eigen::MatrixXd> L, Eigen::Ref<Eigen::VectorXd> D);

void integerGaussTransformations(int i,
                                 int j,
                                 Eigen::Ref<Eigen::MatrixXd> L,
                                 Eigen::Ref<Eigen::VectorXd> a,
                                 Eigen::Ref<Eigen::MatrixXd> Z);

void permute(int k,
             double delta,
             Eigen::Ref<Eigen::VectorXd> a,
             Eigen::Ref<Eigen::MatrixXd> L,
             Eigen::Ref<Eigen::VectorXd> D,
             Eigen::Ref<Eigen::MatrixXd> Z);

void reduction(const Eigen::MatrixXd& Q,
               const Eigen::VectorXd& a,
               Eigen::Ref<Eigen::VectorXd> z,
               Eigen::Ref<Eigen::MatrixXd> L,
               Eigen::Ref<Eigen::VectorXd> D,
               Eigen::Ref<Eigen::MatrixXd> Z);

void search(const Eigen::VectorXd& ahat,
            const Eigen::MatrixXd& L,
            const Eigen::VectorXd& D,
            const int n_cands,
            Eigen::Ref<Eigen::MatrixXd> a_fixed,
            Eigen::Ref<Eigen::VectorXd> sqrd_norms);

//     using MatN = Eigen::MatrixXd;
//     using VecN = Eigen::VectorXd;
//     using MatNi = Eigen:MatrixXd;

//     MatN& D(*_D);
//     MatN& L(*_L);
//     VecN& a(*_a);
//     MatNi& Z(*_Z);

//     LDLT(Q);
//     L = matrixL();
//     D = vectorD();

//     Z.setIdentity();

//     int k = N - 1;
//     int k1 = k;

//     while (k > 0)
//     {
//         for (int i = k; k < N; i++)
//         {
//             integerGaussTransformations(i, j, L, a, Z);
//         }
//     }
//     D(k + 1, k + 1) = D(k, k) + L(k + 1, k) * L(k + 1, k) * D(k + 1, k + 1);
//     if (D(k + 1, k + 1) < D(k + 1))
//     {
//     }
// }

}  // namespace solver
}  // namespace core
}  // namespace mc

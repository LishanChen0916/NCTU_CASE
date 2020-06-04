#include "math_pseudoinverse_solver.h"

namespace math {

// public func.

const char *PseudoinverseSolver::tag()
{
    return "PseudoinverseSolver";
}

PseudoinverseSolver::PseudoinverseSolver()
    :LinearSystemSolver()
{
}

PseudoinverseSolver::~PseudoinverseSolver()
{
}

std::string PseudoinverseSolver::id() const
{
    return std::string(PseudoinverseSolver::tag());
}

math::VectorNd_t PseudoinverseSolver::Solve(
        const math::MatrixN_t &coef_mat,
        const math::VectorNd_t &desired_vector
        ) const
{
    math::VectorNd_t angularVector(coef_mat.cols());
    math::MatrixN_t JacobianInverse(coef_mat.cols(), coef_mat.rows());

    Eigen::JacobiSVD<math::MatrixN_t> svd(coef_mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd sigma(3, 3);

    sigma.setIdentity();
    sigma(0, 0) = svd.singularValues()[0];
    sigma(1, 1) = svd.singularValues()[1];
    sigma(2, 2) = svd.singularValues()[2];

    JacobianInverse = svd.matrixV() * sigma.inverse() * svd.matrixU().transpose();
    angularVector = JacobianInverse * desired_vector;

    return angularVector;
}

// protected func.

// private func.

} // namespace math {

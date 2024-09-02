#pragma once

#include <numeric>

#include "legged_mpc/osqp.hpp"

struct MPCProblem
{
    std::size_t window_size;
    std::vector<VectorX<OSQPFloat>> xref, lb, ub;
    std::vector<MatrixX<OSQPFloat>> Q, R, A, B, C;
};

void addNonZeroTriplets(std::vector<Triplet<OSQPFloat>> &triplets, const MatrixX<OSQPFloat> &matrix,
                        const std::size_t &roff, const std::size_t &coff)
{
    for (std::size_t i = 0; i < matrix.rows(); ++i)
    {
        for (std::size_t j = 0; j < matrix.cols(); ++j)
        {
            const OSQPFloat val = matrix(i, j);
            if (val != 0.0)
            {
                triplets.push_back(Triplet<OSQPFloat>(roff + i, coff + j, val));
            }
        }
    }
}

QPProblem getQPProblem(const MPCProblem &mpc)
{
    if (mpc.window_size == 0)
    {
        throw std::invalid_argument("MPC window size must be greater than 0");
    }

    const int num_states = mpc.Q[0].rows();

    QPProblem qp;
    qp.Nx = num_states * mpc.window_size;
    qp.n = qp.Nx + qp.Nu;
    qp.m = qp.Nx + 2 * qp.Nu;

    qp.H = SparseMatrix<OSQPFloat>(qp.n, qp.n);
    qp.g = VectorX<OSQPFloat>::Zero(qp.n);
    qp.Ac = SparseMatrix<OSQPFloat>(qp.m, qp.n);
    qp.lc = VectorX<OSQPFloat>::Zero(qp.m);
    qp.uc = VectorX<OSQPFloat>::Zero(qp.m);

    qp.lc.segment(0, num_states) = -mpc.xref[0];
    qp.uc.segment(0, num_states) = -mpc.xref[0];

    std::size_t nu_offset = 0, nc_offset = 0;
    std::vector<Triplet<OSQPFloat>> H_triplets;
    std::vector<Triplet<OSQPFloat>> Ac_triplets;
    for (std::size_t i = 0; i < mpc.window_size; ++i)
    {
        const std::size_t nx_offset = num_states * i;

        // Hessian
        addNonZeroTriplets(H_triplets, mpc.Q[i], nx_offset, nx_offset);
        addNonZeroTriplets(H_triplets, mpc.R[i], qp.Nx + nu_offset, qp.Nx + nu_offset);

        // Gradient
        qp.g.segment(nx_offset, num_states) = -mpc.Q[i] * mpc.xref[i];

        // Linear Constraints
        if (i < mpc.window_size - 1)
        {
            addNonZeroTriplets(Ac_triplets, mpc.A[i], nx_offset + num_states, nx_offset);
            addNonZeroTriplets(Ac_triplets, mpc.B[i], qp.Nx + nx_offset + num_states, qp.Nx + nu_offset);
            addNonZeroTriplets(Ac_triplets, mpc.C[i], qp.Nx + 2 * nu_offset, qp.Nx + nu_offset);
        }

        // Control bounds
        qp.lc.segment(qp.Nx + nc_offset, mpc.C[i].rows()) = mpc.lb[i];
        qp.uc.segment(qp.Nx + nc_offset, mpc.C[i].rows()) = mpc.ub[i];

        nu_offset += mpc.R[i].rows();
        nc_offset += mpc.C[i].rows();
    }

    for (std::size_t i = 0; i < qp.Nx; ++i)
    {
        Ac_triplets.push_back(Triplet<OSQPFloat>(i, i, -1.0));
    }

    qp.Nu = nu_offset;

    qp.H.setFromTriplets(H_triplets.begin(), H_triplets.end());
    qp.Ac.setFromTriplets(Ac_triplets.begin(), Ac_triplets.end());

    return qp;
}
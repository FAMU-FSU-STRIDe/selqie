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
    assert(mpc.window_size > 1);

    assert(mpc.A.size() == mpc.window_size);
    const std::size_t num_states = mpc.A[0].cols();
    const std::size_t Nx = num_states * mpc.window_size;

    assert(mpc.B.size() == mpc.window_size - 1);
    assert(mpc.C.size() == mpc.window_size - 1);
    std::size_t Nu = 0;
    std::size_t Nc = 0;
    for (std::size_t i = 0; i < mpc.window_size - 1; ++i)
    {
        Nu += mpc.B[i].cols();
        Nc += mpc.C[i].rows();
    }

    QPProblem qp;
    qp.n = Nx + Nu;
    qp.m = Nx + Nc;

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
        const std::size_t num_controls = mpc.B[i].cols();
        const std::size_t num_constraints = mpc.C[i].rows();

        // Hessian
        assert(mpc.Q[i].rows() == num_states && mpc.Q[i].cols() == num_states);
        addNonZeroTriplets(H_triplets, mpc.Q[i], nx_offset, nx_offset);

        // Gradient
        assert(mpc.xref[i].rows() == num_states);
        qp.g.segment(nx_offset, num_states) = -mpc.Q[i] * mpc.xref[i];

        if (i < mpc.window_size - 1)
        {
            // Hessian
            assert(mpc.R[i].rows() == num_controls && mpc.R[i].cols() == num_controls);
            addNonZeroTriplets(H_triplets, mpc.R[i], Nx + nu_offset, Nx + nu_offset);

            // Linear Constraints
            assert(mpc.A[i].rows() == num_states && mpc.A[i].cols() == num_states);
            addNonZeroTriplets(Ac_triplets, mpc.A[i], nx_offset + num_states, nx_offset);

            assert(mpc.B[i].rows() == num_states);
            addNonZeroTriplets(Ac_triplets, mpc.B[i], Nx + nx_offset + num_states, Nx + nu_offset);

            assert(mpc.C[i].cols() == num_controls);
            addNonZeroTriplets(Ac_triplets, mpc.C[i], Nx + 2 * nu_offset, Nx + nu_offset);

            // Control bounds
            assert(mpc.lb[i].rows() == num_constraints);
            qp.lc.segment(Nx + nc_offset, mpc.C[i].rows()) = mpc.lb[i];

            assert(mpc.ub[i].rows() == num_constraints);
            qp.uc.segment(Nx + nc_offset, mpc.C[i].rows()) = mpc.ub[i];

            nu_offset += num_controls;
            nc_offset += num_constraints;
        }
    }

    for (std::size_t i = 0; i < Nx; ++i)
    {
        Ac_triplets.push_back(Triplet<OSQPFloat>(i, i, -1.0));
    }

    qp.H.setFromTriplets(H_triplets.begin(), H_triplets.end());
    qp.Ac.setFromTriplets(Ac_triplets.begin(), Ac_triplets.end());

    return qp;
}
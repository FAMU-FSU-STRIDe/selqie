#pragma once

#include <memory>
#include <vector>
#include <chrono>
#include <numeric>

#include "osqp/osqp.h"
#include "eigen3/Eigen/Sparse"
#include "eigen3/Eigen/Dense"

using OSQPVector = Eigen::VectorX<OSQPFloat>;
using OSQPMatrix = Eigen::MatrixX<OSQPFloat>;
using OSQPSparseMatrix = Eigen::SparseMatrix<OSQPFloat>;
using OSQPTriplet = Eigen::Triplet<OSQPFloat>;

struct QPProblem
{
    std::size_t n, m;
    OSQPSparseMatrix H, Ac;
    OSQPVector g, lc, uc;
};

struct QPSolution
{
    OSQPInt exit_flag;
    std::chrono::microseconds run_time, setup_time, solve_time;
    OSQPVector xstar;
};

static OSQPCscMatrix convertEigenSparseToCSC(const OSQPSparseMatrix &matrix)
{
    OSQPCscMatrix M;
    M.m = matrix.rows();
    M.n = matrix.cols();
    M.nz = -1;
    M.nzmax = matrix.nonZeros();
    M.x = new OSQPFloat[M.nzmax];
    M.i = new OSQPInt[M.nzmax];
    M.p = new OSQPInt[M.n + 1];

    int k = 0;
    M.p[0] = 0;
    for (int j = 0; j < matrix.outerSize(); ++j)
    {
        for (OSQPSparseMatrix::InnerIterator it(matrix, j); it; ++it)
        {
            M.x[k] = it.value();
            M.i[k] = it.row();
            ++k;
        }
        M.p[j + 1] = k;
    }

    return M;
}

static void cleanupCSC(OSQPCscMatrix &M)
{
    delete[] M.x;
    delete[] M.i;
    delete[] M.p;
}

static QPSolution solveOSQP(const QPProblem &qp, OSQPSettings *settings)
{
    OSQPCscMatrix P = convertEigenSparseToCSC(qp.H);
    OSQPCscMatrix A = convertEigenSparseToCSC(qp.Ac);

    OSQPSolver *solver;
    QPSolution solution;
    solution.exit_flag = osqp_setup(&solver, &P, qp.g.data(), &A, qp.lc.data(), qp.uc.data(), qp.m, qp.n, settings);
    if (solution.exit_flag != 0)
    {
        return solution;
    }

    using namespace std::chrono;
    solution.exit_flag = osqp_solve(solver);
    solution.xstar = OSQPVector(Eigen::Map<OSQPVector>(solver->solution->x, qp.n));
    solution.run_time = microseconds(static_cast<time_t>(solver->info->run_time * 1e6));
    solution.setup_time = microseconds(static_cast<time_t>(solver->info->setup_time * 1e6));
    solution.solve_time = microseconds(static_cast<time_t>(solver->info->solve_time * 1e6));

    osqp_cleanup(solver);
    cleanupCSC(P);
    cleanupCSC(A);

    return solution;
}

struct MPCProblem
{
    std::size_t N;
    OSQPVector x0;
    std::vector<OSQPVector> xref, lb, ub;
    std::vector<OSQPMatrix> Q, R, A, B, C;
};

static void addNonZeroTriplets(std::vector<OSQPTriplet> &triplets, const OSQPMatrix &matrix,
                               const Eigen::Index &roff, const Eigen::Index &coff)
{
    for (Eigen::Index i = 0; i < matrix.rows(); ++i)
    {
        for (Eigen::Index j = 0; j < matrix.cols(); ++j)
        {
            const OSQPFloat val = matrix(i, j);
            if (val != 0.0)
            {
                triplets.push_back(OSQPTriplet(roff + i, coff + j, val));
            }
        }
    }
}

static QPProblem getQPProblem(const MPCProblem &mpc)
{
    assert(mpc.N > 1);
    assert(mpc.xref.size() == mpc.N);
    assert(mpc.Q.size() == mpc.N);
    assert(mpc.R.size() == mpc.N - 1);
    assert(mpc.A.size() == mpc.N - 1);
    assert(mpc.B.size() == mpc.N - 1);
    assert(mpc.C.size() == mpc.N - 1);
    assert(mpc.lb.size() == mpc.N - 1);
    assert(mpc.ub.size() == mpc.N - 1);

    const std::size_t num_states = mpc.A[0].cols();
    const std::size_t Nx = num_states * mpc.N;

    std::size_t Nu = 0;
    std::size_t Nc = 0;
    for (std::size_t i = 0; i < mpc.N - 1; ++i)
    {
        Nu += mpc.B[i].cols();
        Nc += mpc.C[i].rows();
    }

    QPProblem qp;
    qp.n = Nx + Nu;
    qp.m = Nx + Nc;

    qp.H = OSQPSparseMatrix(qp.n, qp.n);
    qp.g = OSQPVector::Zero(qp.n);
    qp.Ac = OSQPSparseMatrix(qp.m, qp.n);
    qp.lc = OSQPVector::Zero(qp.m);
    qp.uc = OSQPVector::Zero(qp.m);

    assert(std::size_t(mpc.x0.rows()) == num_states);
    qp.lc.segment(0, num_states) = -mpc.x0;
    qp.uc.segment(0, num_states) = -mpc.x0;

    std::size_t nu_offset = 0, nc_offset = 0;
    std::vector<OSQPTriplet> H_triplets;
    std::vector<OSQPTriplet> Ac_triplets;
    for (std::size_t i = 0; i < mpc.N; ++i)
    {
        const std::size_t nx_offset = num_states * i;
        const std::size_t num_controls = mpc.B[i].cols();
        const std::size_t num_constraints = mpc.C[i].rows();

        // Hessian
        assert(std::size_t(mpc.Q[i].rows()) == num_states && std::size_t(mpc.Q[i].cols()) == num_states);
        addNonZeroTriplets(H_triplets, mpc.Q[i], nx_offset, nx_offset);

        // Gradient
        assert(std::size_t(mpc.xref[i].rows()) == num_states);
        qp.g.segment(nx_offset, num_states) = -mpc.Q[i] * mpc.xref[i];

        if (i < mpc.N - 1)
        {
            // Hessian
            assert(std::size_t(mpc.R[i].rows()) == num_controls && std::size_t(mpc.R[i].cols()) == num_controls);
            addNonZeroTriplets(H_triplets, mpc.R[i], Nx + nu_offset, Nx + nu_offset);

            // Linear Constraints
            assert(std::size_t(mpc.A[i].rows()) == num_states && std::size_t(mpc.A[i].cols()) == num_states);
            addNonZeroTriplets(Ac_triplets, mpc.A[i], nx_offset + num_states, nx_offset);

            assert(std::size_t(mpc.B[i].rows()) == num_states);
            addNonZeroTriplets(Ac_triplets, mpc.B[i], nx_offset + num_states, Nx + nu_offset);

            assert(std::size_t(mpc.C[i].cols()) == num_controls);
            addNonZeroTriplets(Ac_triplets, mpc.C[i], Nx + nu_offset, Nx + nc_offset);

            // Control bounds
            assert(std::size_t(mpc.lb[i].rows()) == num_constraints);
            qp.lc.segment(Nx + nc_offset, mpc.C[i].rows()) = mpc.lb[i];

            assert(std::size_t(mpc.ub[i].rows()) == num_constraints);
            qp.uc.segment(Nx + nc_offset, mpc.C[i].rows()) = mpc.ub[i];

            nu_offset += num_controls;
            nc_offset += num_constraints;
        }
    }

    for (std::size_t i = 0; i < Nx; ++i)
    {
        Ac_triplets.push_back(OSQPTriplet(i, i, -1.0));
    }

    qp.H.setFromTriplets(H_triplets.begin(), H_triplets.end());
    qp.Ac.setFromTriplets(Ac_triplets.begin(), Ac_triplets.end());

    return qp;
}
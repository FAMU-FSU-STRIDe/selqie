#pragma once

#include <memory>
#include <vector>
#include <chrono>

#include "osqp/osqp.h"
#include <eigen3/Eigen/Sparse>

using namespace Eigen;
using namespace std::chrono;

struct QPProblem
{
    std::size_t n, m;
    SparseMatrix<OSQPFloat> H, Ac;
    VectorX<OSQPFloat> g, lc, uc;
};

struct QPSolution
{
    OSQPInt exit_flag;
    microseconds run_time, setup_time, solve_time;
    VectorX<OSQPFloat> xstar, ustar;
};

static OSQPCscMatrix convertEigenSparseToCSC(const Eigen::SparseMatrix<OSQPFloat> &matrix)
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
        for (Eigen::SparseMatrix<OSQPFloat>::InnerIterator it(matrix, j); it; ++it)
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

    solution.exit_flag = osqp_solve(solver);
    solution.xstar = VectorX<OSQPFloat>(Map<VectorX<OSQPFloat>>(solver->solution->x, qp.n));
    solution.run_time = microseconds(static_cast<time_t>(solver->info->run_time * 1e6));
    solution.setup_time = microseconds(static_cast<time_t>(solver->info->setup_time * 1e6));
    solution.solve_time = microseconds(static_cast<time_t>(solver->info->solve_time * 1e6));

    osqp_cleanup(solver);
    cleanupCSC(P);
    cleanupCSC(A);

    return solution;
}
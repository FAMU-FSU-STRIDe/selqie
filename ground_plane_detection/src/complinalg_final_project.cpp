#include <fstream>
#include <chrono>
#include <random>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Dense>

static inline Eigen::VectorXf least_squares_eigen_householder_qr(const Eigen::MatrixXf &A, const Eigen::VectorXf &b)
{
    assert(A.rows() == b.size());
    return A.colPivHouseholderQr().solve(b);
}

static inline Eigen::VectorXf least_squares_eigen_svd(const Eigen::MatrixXf &A, const Eigen::VectorXf &b)
{
    assert(A.rows() == b.size());
    return A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
}

static inline Eigen::VectorXf least_squares_eigen_normal_equations(const Eigen::MatrixXf &A, const Eigen::VectorXf &b) 
{
    assert(A.rows() == b.size());
    return (A.transpose() * A).ldlt().solve(A.transpose() * b);
}

pcl::PointCloud<pcl::PointXYZ> generate_random_cloud(const std::size_t &size, 
                                                     const double &min_xy, const double &max_xy, 
                                                     const Eigen::Vector3f &coeffs, const double &noise = 0.0)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());

    std::uniform_real_distribution<> rand_xy(min_xy, max_xy);
    std::normal_distribution<> rand_noise(0.0, noise);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = size;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);
    for (std::size_t i = 0; i < cloud.size(); i++)
    {
        cloud[i].x = min_xy + rand_xy(gen);
        cloud[i].y = min_xy + rand_xy(gen);
        cloud[i].z = coeffs(0) * cloud[i].x + coeffs(1) * cloud[i].y + coeffs(2) + rand_noise(gen);
    }
    return cloud;
}

int main(int, char **)
{
    const int cloud_size = 10000;
    const double min_xy = -10.0;
    const double max_xy = 10.0;
    const Eigen::Vector3f coeffs(1.0, 2.0, 3.0);
    const double noise = 0.1;

    std::ofstream timing_file, error_file;
    timing_file.open("/tmp/cla_final_experiment_timing.txt");
    error_file.open("/tmp/cla_final_experiment_error.txt");

    const int test_iterations = 10000;
    for (int i = 1; i <= test_iterations; i++)
    {
        // Generate random point cloud
        const auto pcl_cloud = generate_random_cloud(cloud_size, min_xy, max_xy, coeffs, noise);

        // Create A and b Matrices
        Eigen::MatrixX3f A(pcl_cloud.size(), 3);
        Eigen::VectorXf b(pcl_cloud.size());
        for (std::size_t i = 0; i < pcl_cloud.size(); i++)
        {
            A(i, 0) = pcl_cloud[i].x;
            A(i, 1) = pcl_cloud[i].y;
            A(i, 2) = 1.0;
            b(i) = pcl_cloud[i].z;
        }

        // Eigen Housholder QR
        auto cstart = std::chrono::high_resolution_clock::now();
        const auto x_householder_qr = least_squares_eigen_householder_qr(A, b);
        auto cend = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::duration_cast<std::chrono::microseconds>(cend - cstart).count();
        auto error = (x_householder_qr - coeffs).squaredNorm();
        timing_file << time << " ";
        error_file << error << " ";

        // Eigen SVD
        cstart = std::chrono::high_resolution_clock::now();
        const auto x_svd = least_squares_eigen_svd(A, b);
        cend = std::chrono::high_resolution_clock::now();
        time = std::chrono::duration_cast<std::chrono::microseconds>(cend - cstart).count();
        error = (x_svd - coeffs).squaredNorm();
        timing_file << time << " ";
        error_file << error << " ";

        // Eigen Normal Equations
        cstart = std::chrono::high_resolution_clock::now();
        const auto x_normal_equations = least_squares_eigen_normal_equations(A, b);
        cend = std::chrono::high_resolution_clock::now();
        time = std::chrono::duration_cast<std::chrono::microseconds>(cend - cstart).count();
        error = (x_normal_equations - coeffs).squaredNorm();
        timing_file << time << " ";
        error_file << error << std::endl;

        timing_file << "\n";
        error_file << "\n";

        printf("Iteration %d\n", i);
    }

    timing_file.close();
    error_file.close();

    printf("Done\n");
    return 0;
}
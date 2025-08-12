#include <iostream>
#include <random>  
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>

#include <vector>

const float rand_num_max = 20.0f;
const float rand_num_min = 0.0f;

typedef pcl::PointXYZ PointType;

float randomfloat(float min, float max)
{
    static std::mt19937 engine(std::random_device{}());
    std::uniform_real_distribution<float> dist(min, max);
    return dist(engine);
}

inline Eigen::Quaternionf so3_exp(const Eigen::Vector3f& omega) {
  double theta_sq = omega.dot(omega);

  double theta;
  double imag_factor;
  double real_factor;
  if(theta_sq < 1e-10) {
    theta = 0;
    double theta_quad = theta_sq * theta_sq;
    imag_factor = 0.5 - 1.0 / 48.0 * theta_sq + 1.0 / 3840.0 * theta_quad;
    real_factor = 1.0 - 1.0 / 8.0 * theta_sq + 1.0 / 384.0 * theta_quad;
  } else {
    theta = std::sqrt(theta_sq);
    double half_theta = 0.5 * theta;
    imag_factor = std::sin(half_theta) / theta;
    real_factor = std::cos(half_theta);
  }

  return Eigen::Quaternionf(real_factor, imag_factor * omega.x(), imag_factor * omega.y(), imag_factor * omega.z());
}

void generate_pc(Eigen::Matrix4f& gt, pcl::PointCloud<PointType>::Ptr& src, pcl::PointCloud<PointType>::Ptr& tgt, const int N)
{
    Eigen::Matrix3f R = gt.block<3, 3>(0, 0);
    Eigen::Vector3f t = gt.block<3, 1>(0, 3);

    for(int i = 0; i < N; i++)
    {
        Eigen::Vector3f pt_eigen(randomfloat(rand_num_min, rand_num_max), randomfloat(rand_num_min, rand_num_max), randomfloat(rand_num_min, rand_num_max));
        PointType pt;
        pt.x = pt_eigen[0];
        pt.y = pt_eigen[1];
        pt.z = pt_eigen[2];
        src->points.emplace_back(pt);
        Eigen::Vector3f pt_eigen_tgt = R * pt_eigen + t;
        PointType pt_tgt;
        pt_tgt.x = pt_eigen_tgt[0];
        pt_tgt.y = pt_eigen_tgt[1];
        pt_tgt.z = pt_eigen_tgt[2];
        tgt->points.emplace_back(pt_tgt);
    }
}

int main()
{
    pcl::PointCloud<PointType>::Ptr src(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr tgt(new pcl::PointCloud<PointType>());
    const int N = 1000;
    Eigen::Matrix4f gt = Eigen::Matrix4f::Identity();
    gt(0, 3) = 10.0;
    gt(1, 3) = 10.0;
    gt(2, 3) = 10.0;
    generate_pc(gt, src, tgt, N);

    pcl::KdTreeFLANN<PointType>::Ptr kdtree_cloud(new pcl::KdTreeFLANN<PointType>());
    kdtree_cloud->setInputCloud(tgt);

    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f final_transformation = initial_guess;

    int iter_max_num = 500;
    int iter = 0;
    float pre_err = -1;

    while(iter < iter_max_num)
    {
        iter++;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> pairs;

        pcl::PointCloud<PointType>::Ptr src_transformed(new pcl::PointCloud<PointType>);
        pcl::transformPointCloud (*src, *src_transformed, final_transformation);

        for(int i = 0; i < src_transformed->points.size(); i++)
        {
            PointType pt = src_transformed->points[i];
            PointType original_pt = src->points[i];
            kdtree_cloud->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis); 
            PointType tgt_pt = tgt->points[pointSearchInd[0]];
            if(pointSearchSqDis[0] > 5.0) continue;
            pairs.emplace_back(std::make_pair(Eigen::Vector3f(original_pt.x, original_pt.y, original_pt.z), 
                Eigen::Vector3f(tgt_pt.x, tgt_pt.y, tgt_pt.z))); 
        }

        Eigen::Matrix<float, 6, 6> A = Eigen::Matrix<float, 6, 6>::Zero();
        Eigen::Matrix<float, 6, 1> b = Eigen::Matrix<float, 6, 1>::Zero();

        Eigen::Matrix3f R = final_transformation.block<3, 3>(0, 0);
        Eigen::Vector3f t = final_transformation.block<3, 1>(0, 3);

        float curr_err = 0.0;

        for(const auto p: pairs)
        {
            Eigen::Vector3f src_pt = p.first;
            Eigen::Vector3f tgt_pt = p.second;
            Eigen::Matrix3f skew_mat;
            skew_mat << 0, -src_pt[0], src_pt[1],
                        src_pt[0], 0, -src_pt[2],
                        -src_pt[1], src_pt[2], 0;
            Eigen::Matrix<float, 3, 6> Jacobian;
            Jacobian.block<3, 3>(0, 0) = R * skew_mat;
            Jacobian.block<3, 3>(0, 3) = -Eigen::Matrix3f::Identity();
            Eigen::Vector3f err = tgt_pt - (R * src_pt + t);
            curr_err += err.transpose() * err;
            A += Jacobian.transpose() * Jacobian;
            b += Jacobian.transpose() * err;
        }

        int numberOfConstraints = 1;

        Eigen::MatrixXf augmented_A(A.rows() + numberOfConstraints, A.cols() + numberOfConstraints);
        augmented_A.setZero();
        
        Eigen::VectorXf augmented_b(A.rows() + numberOfConstraints);
        augmented_b.setZero();

        Eigen::VectorXf constrainedVector(A.cols());
        constrainedVector.setZero();
        constrainedVector(4) = 1.0f;
        augmented_A.topLeftCorner(A.rows(), A.cols()) = A;
        augmented_A.bottomLeftCorner(1, A.cols()) = constrainedVector.transpose();
        augmented_A.topRightCorner(A.rows(), 1) = constrainedVector;
        augmented_b.head(b.rows()) = b;
        augmented_b(A.rows() + numberOfConstraints - 1) = 0.0f;
        Eigen::VectorXf dx_solved(A.rows() + numberOfConstraints);

        dx_solved = augmented_A.householderQr().solve(-augmented_b);

        Eigen::Matrix4f delta_transformation = Eigen::Matrix4f::Identity();
        delta_transformation.block<3, 3>(0, 0) = so3_exp(dx_solved.head<3>()).toRotationMatrix();
        delta_transformation.block<3, 1>(0, 3) = dx_solved.segment(3, 3);
        final_transformation = delta_transformation * final_transformation;
        Eigen::Vector3f delta_trans = delta_transformation.block<3, 1>(0, 3);

        std::cout << "current error: " << curr_err << std::endl;
        std::cout << "delta_trans norm: " << delta_trans.norm() << std::endl;

        if(delta_trans.norm() < 1e-2) break;

    }
    std::cout << "final transformation: " << std::endl;
    std::cout << final_transformation << std::endl;

    return 0;
}
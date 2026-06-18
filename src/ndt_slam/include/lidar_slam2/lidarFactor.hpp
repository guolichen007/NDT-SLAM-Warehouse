#pragma once

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_slam2 {

// 点到线约束因子（用于角点匹配）
class EdgeSE3PointToLine : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3PointToLine(const Eigen::Vector3d& curr_point_, 
                       const Eigen::Vector3d& last_point_a_,
                       const Eigen::Vector3d& last_point_b_)
        : curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_) {}

    bool read(std::istream& is) override {
        return true;
    }

    bool write(std::ostream& os) const override {
        return true;
    }

    void computeError() override {
        const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
        Eigen::Isometry3d T = v1->estimate();
        
        Eigen::Vector3d lp = T * curr_point;
        Eigen::Vector3d nu = (lp - last_point_a).cross(lp - last_point_b);
        Eigen::Vector3d de = last_point_a - last_point_b;
        
        double de_norm = de.norm();
        if (de_norm < 1e-6) {
            _error.setZero();
        } else {
            _error = nu / de_norm;
        }
    }

private:
    Eigen::Vector3d curr_point, last_point_a, last_point_b;
};

// 点到面约束因子（用于平面点匹配）
class EdgeSE3PointToPlane : public g2o::BaseUnaryEdge<1, Eigen::Vector3d, g2o::VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3PointToPlane(const Eigen::Vector3d& curr_point_,
                        const Eigen::Vector3d& last_point_j_,
                        const Eigen::Vector3d& last_point_l_,
                        const Eigen::Vector3d& last_point_m_)
        : curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
          last_point_m(last_point_m_) {
        // 计算平面法向量
        ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
        ljm_norm.normalize();
    }

    bool read(std::istream& is) override {
        return true;
    }

    bool write(std::ostream& os) const override {
        return true;
    }

    void computeError() override {
        const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
        Eigen::Isometry3d T = v1->estimate();
        
        Eigen::Vector3d lp = T * curr_point;
        _error[0] = (lp - last_point_j).dot(ljm_norm);
    }

private:
    Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
    Eigen::Vector3d ljm_norm;
};

} // namespace lidar_slam2

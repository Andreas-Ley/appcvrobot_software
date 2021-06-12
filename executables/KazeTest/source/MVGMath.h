#pragma once

#include <Eigen/Dense>

class EDecomposer {
    public:
        EDecomposer() = default;
        EDecomposer(const Eigen::Matrix3f &E);
        void decompose(const Eigen::Matrix3f &E);

        Eigen::Matrix<float, 3, 4> getP(unsigned variation);
    protected:
        Eigen::Matrix3f m_U;
        Eigen::Matrix3f m_V;
};

class PointTriangulator {
    public:
        PointTriangulator(const Eigen::Matrix<float, 3, 4> &P1, const Eigen::Matrix<float, 3, 4> &P2);

        Eigen::Vector4f triangulate(const Eigen::Vector2f &loc1, const Eigen::Vector2f &loc2);
    protected:
        Eigen::Matrix<float, 3, 4> m_P1;
        Eigen::Matrix<float, 3, 4> m_P2;
        Eigen::JacobiSVD<Eigen::Matrix4f> m_svd;

};
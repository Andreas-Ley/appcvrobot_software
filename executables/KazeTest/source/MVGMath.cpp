#include "MVGMath.h"

EDecomposer::EDecomposer(const Eigen::Matrix3f &E)
{
    decompose(E);
}

void EDecomposer::decompose(const Eigen::Matrix3f &E)
{
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);

    m_U = svd.matrixU();
    m_V = svd.matrixV();

    if (m_U.determinant() < 0.0f)
        m_U = -m_U;

    if (m_V.determinant() < 0.0f)
        m_V = -m_V;
}

Eigen::Matrix<float, 3, 4> EDecomposer::getP(unsigned variation)
{
    Eigen::Matrix<float, 3, 4> res;

    Eigen::Matrix3f W;
    W <<
        0.0f, -1.0f, 0.0f,
        1.0f,  0.0f, 0.0f,
        0.0f,  0.0f, 1.0f;

    if (variation & 1)
        res.block<3, 3>(0,0) = m_U * W * m_V.transpose();
    else
        res.block<3, 3>(0,0) = m_U * W.transpose() * m_V.transpose();

    if (variation & 2)
        res.block<3, 1>(0, 3) = m_U.block<3, 1>(0, 2);
    else
        res.block<3, 1>(0, 3) = -m_U.block<3, 1>(0, 2);

    return res;
}


PointTriangulator::PointTriangulator(const Eigen::Matrix<float, 3, 4> &P1, const Eigen::Matrix<float, 3, 4> &P2) : m_P1(P1), m_P2(P2)
{
}

Eigen::Vector4f PointTriangulator::triangulate(const Eigen::Vector2f &loc1, const Eigen::Vector2f &loc2)
{
    Eigen::Matrix4f A;
    A.block<1, 4>(0, 0) = loc1[0] * m_P1.block<1, 4>(2, 0) - m_P1.block<1, 4>(0, 0);
    A.block<1, 4>(1, 0) = loc1[1] * m_P1.block<1, 4>(2, 0) - m_P1.block<1, 4>(1, 0);
    A.block<1, 4>(2, 0) = loc2[0] * m_P2.block<1, 4>(2, 0) - m_P2.block<1, 4>(0, 0);
    A.block<1, 4>(3, 0) = loc2[1] * m_P2.block<1, 4>(2, 0) - m_P2.block<1, 4>(1, 0);

    m_svd.compute(A, Eigen::ComputeFullV);

    return m_svd.matrixV().block<4, 1>(0, 3);
}


#include "RansacPnP.h"

#include <iostream>

template<typename Type>
Eigen::Matrix<Type, 3, 3> computeClosestOrthonormalMatrix(const Eigen::Matrix<Type, 3, 3> &M)
{
    Eigen::Matrix<Type, 3, 3> MtM = M.transpose() * M;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Type, 3, 3>> es(MtM);

    Eigen::Matrix<Type, 3, 3> MtM_invSqrt;
#if 0
    MtM_invSqrt  = es.eigenvectors().col(0) * es.eigenvectors().col(0).transpose() * (std::abs(es.eigenvalues()[0]) > 1e-20f?(Type)1/std::sqrt(es.eigenvalues()[0]):(Type)1);
    MtM_invSqrt += es.eigenvectors().col(1) * es.eigenvectors().col(1).transpose() * (std::abs(es.eigenvalues()[1]) > 1e-20f?(Type)1/std::sqrt(es.eigenvalues()[1]):(Type)1);
    MtM_invSqrt += es.eigenvectors().col(2) * es.eigenvectors().col(2).transpose() * (std::abs(es.eigenvalues()[2]) > 1e-20f?(Type)1/std::sqrt(es.eigenvalues()[2]):(Type)1);
#else
	MtM_invSqrt = es.operatorInverseSqrt();
#endif

    return M * MtM_invSqrt;
}

template<typename Type>
void PerspectiveNPoint_DLT(const std::vector<Eigen::Matrix<Type, 3, 1>> &points2D,
                           const std::vector<Eigen::Matrix<Type, 4, 1>> &points3D,
                           Eigen::Matrix<Type, 3, 3> &rotation,
                           Eigen::Matrix<Type, 3, 1> &translation)
{
    Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> designMatrix;
    designMatrix.resize(3*points2D.size(), 12);
    for (unsigned i = 0; i < points2D.size(); i++) {
        const Eigen::Matrix<Type, 3, 1> &u = points2D[i];
        const Eigen::Matrix<Type, 4, 1> &x = points3D[i];

        designMatrix.template block<1, 4>(i*3+0, 0).setZero();
        designMatrix.template block<1, 4>(i*3+0, 4) = -u[2] * x;
        designMatrix.template block<1, 4>(i*3+0, 8) = u[1] * x;

        designMatrix.template block<1, 4>(i*3+1, 0) = u[2] * x;
        designMatrix.template block<1, 4>(i*3+1, 4).setZero();
        designMatrix.template block<1, 4>(i*3+1, 8) = -u[0] * x;

        designMatrix.template block<1, 4>(i*3+2, 0) = -u[1] * x;
        designMatrix.template block<1, 4>(i*3+2, 4) = u[0] * x;
        designMatrix.template block<1, 4>(i*3+2, 8).setZero();
    }

    Eigen::JacobiSVD<Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>> svd(designMatrix, Eigen::ComputeFullV);

    auto fVals = svd.matrixV().col(11);


    rotation(0, 0) = fVals[0];
    rotation(0, 1) = fVals[1];
    rotation(0, 2) = fVals[2];
    translation[0] = fVals[3];
    rotation(1, 0) = fVals[4];
    rotation(1, 1) = fVals[5];
    rotation(1, 2) = fVals[6];
    translation[1] = fVals[7];
    rotation(2, 0) = fVals[8];
    rotation(2, 1) = fVals[9];
    rotation(2, 2) = fVals[10];
    translation[2] = fVals[11];

    translation = rotation.colPivHouseholderQr().solve(translation);
    rotation = computeClosestOrthonormalMatrix<double>(rotation.template cast<double>()).template cast<float>();
    if (rotation.determinant() < 0)
        rotation = -rotation;
    translation = rotation * translation;
}

unsigned RansacPnP::process(const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector4f>> &pairs,
                     std::vector<float> &sqrDistances, Eigen::Matrix3f &R, Eigen::Vector3f &t)
{
    std::uniform_int_distribution<unsigned> randomPair(0, pairs.size()-1);

    Eigen::Matrix<float, 24, 12> D;
    D.setZero();

    Eigen::Matrix3f rotation;
    Eigen::Vector3f translation;

    unsigned bestScore = 0;

    for (unsigned iter = 0; iter < m_numIters; iter++) {
        for (unsigned i = 0; i < 8; i++) {
            unsigned idx = randomPair(m_rng);

            const auto &u = pairs[idx].first;
            const auto &x = pairs[idx].second;

            D.block<1, 4>(i*3+0, 0).setZero();
            D.block<1, 4>(i*3+0, 4) = -x;
            D.block<1, 4>(i*3+0, 8) = u[1] * x;

            D.block<1, 4>(i*3+1, 0) = x;
            D.block<1, 4>(i*3+1, 4).setZero();
            D.block<1, 4>(i*3+1, 8) = -u[0] * x;

            D.block<1, 4>(i*3+2, 0) = -u[1] * x;
            D.block<1, 4>(i*3+2, 4) = u[0] * x;
            D.block<1, 4>(i*3+2, 8).setZero();
        }

        Eigen::JacobiSVD<Eigen::Matrix<float, 24, 12>> svd(D, Eigen::ComputeFullV);

        auto fVals = svd.matrixV().col(11);

        rotation(0, 0) = fVals[0];
        rotation(0, 1) = fVals[1];
        rotation(0, 2) = fVals[2];
        translation[0] = fVals[3];
        rotation(1, 0) = fVals[4];
        rotation(1, 1) = fVals[5];
        rotation(1, 2) = fVals[6];
        translation[1] = fVals[7];
        rotation(2, 0) = fVals[8];
        rotation(2, 1) = fVals[9];
        rotation(2, 2) = fVals[10];
        translation[2] = fVals[11];

        translation = rotation.colPivHouseholderQr().solve(translation);
        rotation = computeClosestOrthonormalMatrix<double>(rotation.template cast<double>()).template cast<float>();
        if (rotation.determinant() < 0)
            rotation = -rotation;
        translation = rotation * translation;

        unsigned score = 0;
        for (const auto &p : pairs) {
            Eigen::Vector3f eyespace = rotation * p.second.head<3>() + translation*p.second[3];
            if (std::abs(eyespace[2]) < 1e-20) continue;
            Eigen::Vector2f proj = eyespace.head<2>() / eyespace[2];

            float sqrDistance = (p.first - proj).squaredNorm();
            float thresh = m_maxDistance*m_maxDistance;

            if (sqrDistance < thresh)
                score++;
        }

        if (score > bestScore) {
            bestScore = score;
            R = rotation;
            t = translation;
        }
    }

    sqrDistances.resize(pairs.size());
    for (unsigned i = 0; i < pairs.size(); i++) {
        const auto &p = pairs[i];
        Eigen::Vector3f eyespace = R * p.second.head<3>() + t*p.second[3];
        if (std::abs(eyespace[2]) < 1e-20) {
            sqrDistances[i] = 1e30f;
            continue;
        }
        Eigen::Vector2f proj = eyespace.head<2>() / eyespace[2];

        float sqrDistance = (p.first - proj).squaredNorm();
        sqrDistances[i] = sqrDistance;
    }

    return bestScore;
}

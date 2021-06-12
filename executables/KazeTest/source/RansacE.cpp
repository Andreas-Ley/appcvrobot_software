#include "RansacE.h"

unsigned RansacE::process(const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> &pairs,
    std::vector<float> &distances, Eigen::Matrix3f &bestE)
{

    std::uniform_int_distribution<unsigned> randomPair(0, pairs.size()-1);

    Eigen::Matrix<float, 8, 9> D;

    unsigned bestScore = 0;

    for (unsigned iter = 0; iter < m_numIters; iter++) {
        for (unsigned i = 0; i < 8; i++) {
            unsigned idx = randomPair(m_rng);

            D(i, 0) = pairs[idx].first[0] * pairs[idx].second[0];
            D(i, 1) = pairs[idx].first[1] * pairs[idx].second[0];
            D(i, 2) = 1.0f                * pairs[idx].second[0];
            D(i, 3) = pairs[idx].first[0] * pairs[idx].second[1];
            D(i, 4) = pairs[idx].first[1] * pairs[idx].second[1];
            D(i, 5) = 1.0f                * pairs[idx].second[1];
            D(i, 6) = pairs[idx].first[0];
            D(i, 7) = pairs[idx].first[1];
            D(i, 8) = 1.0f;
        }

        Eigen::JacobiSVD<Eigen::Matrix<float, 8, 9>> svd(D, Eigen::ComputeFullV);

        auto svdV = svd.matrixV();

        Eigen::Matrix3f E;
        E <<
                svdV(0, 8),
                svdV(1, 8),
                svdV(2, 8),
                svdV(3, 8),
                svdV(4, 8),
                svdV(5, 8),
                svdV(6, 8),
                svdV(7, 8),
                svdV(8, 8);


        Eigen::JacobiSVD<Eigen::Matrix3f> svd2(E, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Vector3f singularValues(1.0f, 1.0f, 0.0f);

        E = svd2.matrixU() * singularValues.asDiagonal() * svd2.matrixV().transpose();

        unsigned score = 0;
        for (const auto &p : pairs) {
            float rx = p.first[0] * E(0, 0) + p.first[1] * E(0, 1) + E(0, 2);
            float ry = p.first[0] * E(1, 0) + p.first[1] * E(1, 1) + E(1, 2);
            float rz = p.first[0] * E(2, 0) + p.first[1] * E(2, 1) + E(2, 2);

            float r = p.second[0] * rx + p.second[1] * ry + rz;

            if (std::abs(r) < m_maxDistance)
                score++;
        }

        if (score > bestScore) {
            bestScore = score;
            bestE = E;
        }
    }

    distances.resize(pairs.size());

    for (unsigned i = 0; i < pairs.size(); i++) {
        const auto &p = pairs[i];
        float rx = p.first[0] * bestE(0, 0) + p.first[1] * bestE(0, 1) + bestE(0, 2);
        float ry = p.first[0] * bestE(1, 0) + p.first[1] * bestE(1, 1) + bestE(1, 2);
        float rz = p.first[0] * bestE(2, 0) + p.first[1] * bestE(2, 1) + bestE(2, 2);

        float r = p.second[0] * rx + p.second[1] * ry + rz;

        distances[i] = std::abs(r);
    }

    return bestScore;
}

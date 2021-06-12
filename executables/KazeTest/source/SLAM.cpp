#include "SLAM.h"

#include "RansacE.h"
#include "RansacH.h"
#include "MVGMath.h"
#include "Image.h"

#include <iostream>
#include <fstream>

SLAM::SLAM(InternalCalibration internalCalib) : m_internalCalib(internalCalib)
{

}

bool SLAM::tryIngestImage(Image image)
{
    auto frame = processImage(image);

    switch (m_map.m_cameras.size()) {
        case 0:
            firstFrame(std::move(frame));
        break;
        case 1:
            secondFrame(std::move(frame));
        break;
        default:
            nextFrame(std::move(frame));
        break;
    }

    return true;
}


Frame SLAM::processImage(Image image)
{
    Frame res;

    res.extractKeypoints(image, m_brief);
    res.buildKPGrid();

    return res;
}


void SLAM::firstFrame(Frame frame)
{
    if (frame.getKeypoints().size() < m_settings.minKeypointsFirstFrame) return;

    new Camera(m_map, std::move(frame), &m_internalCalib);
}

void SLAM::secondFrame(Frame frame)
{
    if (frame.getKeypoints().size() < m_settings.minEInliersSecondFrame) return;

    auto &firstCamera = m_map.m_cameras.front();


    //////////////////   Match keypoints
    std::vector<RawMatch> rawMatches;
    firstCamera.getFrame().matchWith(frame, rawMatches);

    std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> pairs;
    std::vector<std::pair<unsigned, unsigned>> pairIndices;

    Eigen::Matrix3f KInv = m_internalCalib.internalCalib.inverse();

    //////////////////   Filter matches

    unsigned thresN = m_settings.maxDistanceRatio.numerator();
    unsigned thresD = m_settings.maxDistanceRatio.denominator();
    for (unsigned i = 0; i < rawMatches.size(); i++) {
        auto &match = rawMatches[i];
        if (match.bestDistance > m_settings.matchMaxDistance) continue;

        if (match.bestDistance * thresD > match.secondBestDistance * thresN) continue;

        const auto &kpA = firstCamera.getFrame().getKeypoints()[i];
        const auto &kpB = frame.getKeypoints()[match.dstIdx];

        // todo: Radial distortion

        Eigen::Vector3f locA = KInv * Eigen::Vector3f(kpA.x, kpA.y, 1.0f);
        Eigen::Vector3f locB = KInv * Eigen::Vector3f(kpB.x, kpB.y, 1.0f);

        pairs.push_back({locA.head<2>()/locA[2], locB.head<2>()/locB[2]});
        pairIndices.push_back({i, match.dstIdx});
    }

    if (pairs.size() < m_settings.minEInliersSecondFrame) return;

    //////////////////   Ransac E
    std::vector<float> distances;
    RansacE ransacE;
    ransacE.setMaxDistance(m_settings.ransacThresh);

    Eigen::Matrix3f E;
    unsigned numInliersE = ransacE.process(pairs, distances, E);

    if (numInliersE < m_settings.minEInliersSecondFrame) return;

    //////////////////   Ransac H
    {
        std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> Hpairs;
        Hpairs.reserve(pairs.size());
        for (unsigned i = 0; i < pairs.size(); i++)
            if (distances[i] < m_settings.ransacThresh)
                Hpairs.push_back(pairs[i]);

        RansacH ransacH;
        ransacH.setMaxDistance(m_settings.ransacThresh);
        Eigen::Matrix3f H;
        unsigned numInliersH = ransacH.process(pairs, H);

        if (numInliersH > numInliersE * m_settings.maxEHInliersSecondFrameFraction) return;
    }

    //////////////////   Estimate pose

    EDecomposer eDecomposer(E);

    unsigned bestVariation = 0;
    unsigned bestVariationInFront = 0;

    for (unsigned variation = 0; variation < 4; variation++) {

        Eigen::Matrix<float, 3, 4> P1, P2;
        P1.setIdentity();
        P2 = eDecomposer.getP(variation);

        unsigned inFront = 0;
        PointTriangulator triangulator(P1, P2);
        for (unsigned i = 0; i < pairs.size(); i++) {
            if (distances[i] > m_settings.ransacThresh) continue;
            auto &p = pairs[i];

            Eigen::Vector4f loc3D = triangulator.triangulate(p.first, p.second);

            float proj_z = P2.block<1, 4>(2,0) * loc3D;

            if (loc3D[2] > 0.0f == loc3D[3] > 0.0f &&
                proj_z > 0.0f == loc3D[3] > 0.0f)
                inFront++;

        }

        if (inFront > bestVariationInFront) {
            bestVariationInFront = inFront;
            bestVariation = variation;
        }
    }

    if (bestVariationInFront < m_settings.minInitialTracks) return;

    //////////////////   Create second camera

    Eigen::Matrix<float, 3, 4> P2 = eDecomposer.getP(bestVariation);
    CameraPose pose;
    pose.world2eye = P2.block<3,3>(0,0);
    pose.location = -pose.world2eye.transpose() * P2.block<3,1>(0, 3);


    auto *newCamera = new Camera(m_map, std::move(frame), &m_internalCalib, pose);


    //////////////////   Add tracks


    Eigen::Matrix<float, 3, 4> P1;
    P1.setIdentity();

    std::vector<NewTrack> tracks;
    tracks.reserve(pairs.size());

    PointTriangulator triangulator(P1, P2);
    for (unsigned i = 0; i < pairs.size(); i++) {
        if (distances[i] > m_settings.ransacThresh) continue;
        auto &p = pairs[i];

        // todo: radial distortion

        Eigen::Vector4f loc3D = triangulator.triangulate(p.first, p.second);

        float proj_z1 = P1.block<1, 4>(2,0) * loc3D;
        float proj_z2 = P2.block<1, 4>(2,0) * loc3D;

        if (proj_z1 > 0.0f == loc3D[3] > 0.0f &&
            proj_z2 > 0.0f == loc3D[3] > 0.0f) {

            tracks.push_back({
                .location = loc3D,
                .srcKeypointIdx = pairIndices[i].first,
                .dstKeypointIdx = pairIndices[i].second,
                .matchScore = 0
            });
        }
    }

    firstCamera.addNewTracks(newCamera, tracks.size(), tracks.data());


    std::fstream plyFile("tracks.ply", std::fstream::out);
    m_map.exportToPly(plyFile);

    std::cout << "Success!" << std::endl;
}

void SLAM::nextFrame(Frame frame)
{
}


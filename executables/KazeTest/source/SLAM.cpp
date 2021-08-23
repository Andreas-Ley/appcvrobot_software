#include "SLAM.h"

#include "RansacE.h"
#include "RansacH.h"
#include "RansacPnP.h"
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
    res.buildKPGrid(m_internalCalib);

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

        Eigen::Vector3f locA = KInv * Eigen::Vector3f(kpA.x_undistorted_times4, kpA.y_undistorted_times4, 4.0f);
        Eigen::Vector3f locB = KInv * Eigen::Vector3f(kpB.x_undistorted_times4, kpB.y_undistorted_times4, 4.0f);

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
    std::vector<Camera*> lastCameras;
    lastCameras.reserve(m_settings.numPastFramesMatch);
    std::vector<std::vector<RawMatch>> rawMatches;
    rawMatches.reserve(m_settings.numPastFramesMatch);

    for (auto it =  m_map.m_cameras.rbegin(); it != m_map.m_cameras.rend(); ++it){
        lastCameras.push_back(&*it);
        rawMatches.push_back({});
        it->getFrame().matchWith(frame, rawMatches.back());

        if (lastCameras.size() >= m_settings.numPastFramesMatch) break;
    }

    unsigned thresN = m_settings.maxDistanceRatio.numerator();
    unsigned thresD = m_settings.maxDistanceRatio.denominator();

    std::vector<std::pair<Eigen::Vector2f, Eigen::Vector4f>> pairs;

    struct PairInfo {
        Track *track;
        unsigned newCamKPIndex;
    };

    std::vector<PairInfo> pairInfo;

    Eigen::Vector3f coordinateCenterAnchor = lastCameras.front()->getPose().location;


    Eigen::Matrix3f KInv = m_internalCalib.internalCalib.inverse();

    auto considerKeypoint = [&](unsigned potentialMatchingCamera, unsigned potentialMatchingKeypoint, Track *track){
        auto &newMatch = rawMatches[potentialMatchingCamera][potentialMatchingKeypoint];

        if (newMatch.bestDistance < m_settings.matchMaxDistance)
            if (newMatch.bestDistance * thresD < newMatch.secondBestDistance * thresN) {

                auto *anchorCamera = track->getAnchor().camera;
                auto anchorKPIdx = track->getAnchor().keypointIdx;

                Eigen::Vector4f imageSpace;
                imageSpace[0] = anchorCamera->getFrame().getKeypoints()[anchorKPIdx].x_undistorted_times4;
                imageSpace[1] = anchorCamera->getFrame().getKeypoints()[anchorKPIdx].y_undistorted_times4;
                imageSpace[2] = track->getRcpZ() * 4.0f;
                imageSpace[3] = 4.0f;

                const auto &invP = anchorCamera->getInverseProjectionViewNoTranslation();

                Eigen::Vector4f cam_relative_ws = invP * imageSpace;

                Eigen::Vector3f rel_location;
                rel_location = anchorCamera->getPose().location - coordinateCenterAnchor;

                Eigen::Vector4f ws = cam_relative_ws;
                ws.head<3>() += rel_location * cam_relative_ws[3];
                ws /= ws.norm();

                Eigen::Vector3f loc = KInv * Eigen::Vector3f(
                                frame.getKeypoints()[newMatch.dstIdx].x_undistorted_times4,
                                frame.getKeypoints()[newMatch.dstIdx].y_undistorted_times4, 4.0f);
                pairs.push_back({
                    loc.head<2>()/loc[2],
                    ws
                });
                pairInfo.push_back({
                    .track = track,
                    .newCamKPIndex = newMatch.dstIdx
                });
            }
    };

    unsigned startIndex = lastCameras.back()->getCameraIndex();
    unsigned lastIndex = lastCameras.front()->getCameraIndex();

    unsigned trackCount = 0;
    for (unsigned i = 0; i < lastCameras.size(); i++) {
        for (const auto &trackRef : lastCameras[i]->getTrackReferences()) {
            considerKeypoint(i, trackRef.keypointIndex, trackRef.track);
        }
        trackCount += lastCameras[i]->getTrackReferences().size();
    }

    std::cout << "trackCount: " << trackCount << std::endl;
    std::cout << "Num pairs: " << pairs.size() << std::endl;

    if (pairs.empty())
        return;

    RansacPnP ransac;
    ransac.setMaxDistance(m_settings.ransacThresh);

    Eigen::Matrix3f R;
    Eigen::Vector3f t;
    std::vector<float> sqrDistances;
    unsigned numInliers = ransac.process(pairs, sqrDistances, R, t);

    std::cout << "First camera location: " << std::endl << lastCameras[1]->getPose().location << std::endl << std::endl;
    std::cout << "Second camera location: " << std::endl << lastCameras[0]->getPose().location << std::endl << std::endl;

    std::cout << "Num inliers for pnp: " << numInliers << " of " << pairs.size() << std::endl;

    std::cout << "R: " << std::endl << R << std::endl << std::endl;
    std::cout << "t: " << std::endl << t << std::endl << std::endl;

    if (numInliers < m_settings.minPnPInliers) return;

    CameraPose pose;
    pose.world2eye = R;
    pose.location = coordinateCenterAnchor - R.transpose() * t;
    std::cout << "pose.location: " << std::endl << pose.location << std::endl << std::endl;

    auto *newCamera = new Camera(m_map, std::move(frame), &m_internalCalib, pose);

    for (unsigned i = 0; i < pairs.size(); i++) {
        if (sqrDistances[i] > m_settings.ransacThresh*m_settings.ransacThresh) continue;

        bool alreadyAdded = false;
        auto track = pairInfo[i].track;
        for (auto it = track->getObservation().rbegin(); it != track->getObservation().rend(); ++it) {
            if (it->camera != newCamera) break;

            if (it->keypointIdx == pairInfo[i].newCamKPIndex) {
                alreadyAdded = true;
                break;
            }
        }

        if (!alreadyAdded) {
            pairInfo[i].track->addObservation({
                .camera = newCamera,
                .keypointIdx = (uint16_t) pairInfo[i].newCamKPIndex,
                .matchScore = 0
            });
        }
    }

    std::fstream plyFile("tracks.ply", std::fstream::out);
    m_map.exportToPly(plyFile);


    generateNewTracks(*newCamera, *lastCameras.front());
}


void SLAM::generateNewTracks(Camera &camera1, Camera &camera2)
{
    auto relativePose_c2 = camera2.getPose().getPoseRelativeTo(camera1.getPose());

    Eigen::Matrix3f KInv = m_internalCalib.internalCalib.inverse();



    Eigen::Vector3f t = -relativePose_c2.world2eye*relativePose_c2.location;

    Eigen::Matrix3f skewSymmetricMatrix;
    skewSymmetricMatrix.setZero();
    skewSymmetricMatrix(0, 1) = -t[2];
    skewSymmetricMatrix(1, 0) = t[2];

    skewSymmetricMatrix(0, 2) = t[1];
    skewSymmetricMatrix(2, 0) = -t[1];

    skewSymmetricMatrix(1, 2) = -t[0];
    skewSymmetricMatrix(2, 1) = t[0];

    Eigen::Matrix3f E = skewSymmetricMatrix * relativePose_c2.world2eye;

    Eigen::Matrix3f F = KInv.transpose() * E * KInv;


    std::vector<Track*> cam1_kpInUse(camera1.getFrame().getKeypoints().size(), nullptr);
    for (auto &ref : camera1.getTrackReferences())
        cam1_kpInUse[ref.keypointIndex] = ref.track;
    std::vector<Track*> cam2_kpInUse(camera2.getFrame().getKeypoints().size(), nullptr);
    for (auto &ref : camera2.getTrackReferences())
        cam2_kpInUse[ref.keypointIndex] = ref.track;


    std::vector<RawMatch> rawMatches;
    camera1.getFrame().matchWith(camera2.getFrame(), F, 20, rawMatches);

    unsigned thresN = m_settings.maxDistanceRatio.numerator();
    unsigned thresD = m_settings.maxDistanceRatio.denominator();

    std::vector<NewTrack> tracks;
    tracks.reserve(rawMatches.size());

    Eigen::Matrix<float, 3, 4> P1;
    P1.setIdentity();
    P1.block<3,3>(0,0) = camera1.getPose().world2eye;
    P1.block<3,1>(0, 3) = -camera1.getPose().world2eye * camera1.getPose().location;
    //P1 = m_internalCalib.internalCalib * P1;


    Eigen::Matrix<float, 3, 4> P2;
    P2.setIdentity();
    P2.block<3,3>(0,0) = camera2.getPose().world2eye;
    P2.block<3,1>(0, 3) = -camera2.getPose().world2eye * camera2.getPose().location;
    //P2 = m_internalCalib.internalCalib * P2;



    PointTriangulator triangulator(P1, P2);


    unsigned count = 0;
    for (unsigned srcIdx = 0; srcIdx < rawMatches.size(); srcIdx++) {
        auto &newMatch = rawMatches[srcIdx];
        if (cam1_kpInUse[srcIdx] != nullptr && cam1_kpInUse[srcIdx] == cam2_kpInUse[newMatch.dstIdx])
            continue;

        if (newMatch.bestDistance < m_settings.matchMaxDistance)
            if (newMatch.bestDistance * thresD < newMatch.secondBestDistance * thresN) {
                count++;

                const auto &kpA = camera1.getFrame().getKeypoints()[srcIdx];
                const auto &kpB = camera2.getFrame().getKeypoints()[newMatch.dstIdx];

                Eigen::Vector3f locA = KInv * Eigen::Vector3f(kpA.x_undistorted_times4, kpA.y_undistorted_times4, 4.0f);
                Eigen::Vector3f locB = KInv * Eigen::Vector3f(kpB.x_undistorted_times4, kpB.y_undistorted_times4, 4.0f);

                Eigen::Vector4f loc3D = triangulator.triangulate(locA.head<2>()/locA[2], locB.head<2>()/locB[2]);

                float proj_z1 = P1.block<1, 4>(2,0) * loc3D;
                float proj_z2 = P2.block<1, 4>(2,0) * loc3D;

                if (proj_z1 > 0.0f == loc3D[3] > 0.0f &&
                    proj_z2 > 0.0f == loc3D[3] > 0.0f) {

                    tracks.push_back({
                        .location = loc3D,
                        .srcKeypointIdx = srcIdx,
                        .dstKeypointIdx = newMatch.dstIdx,
                        .matchScore = 0
                    });
                }

            }
    }

    camera1.addNewTracks(&camera2, tracks.size(), tracks.data());

    std::cout << "Potential new tracks: " << count << std::endl;
}
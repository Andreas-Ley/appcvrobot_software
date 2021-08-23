#include "Map.h"

#include "Keypoint.h"

#include <iostream>

#if 0
void InternalCalibration::undistortKeypoints(Keypoint *kps, unsigned count) const
{
    const Eigen::Matrix3f &K = internalCalib;
    const Eigen::Vector3f &distortion = distortion_k;

    for (unsigned i = 0; i < count; i++) {
        Eigen::Vector2f distortedPixel(kps[i].x + 0.5f, kps[i].y + 0.5f);
        Eigen::Vector2f distortedCenteredPixel = distortedPixel - Eigen::Vector2f(K(0, 2), K(1, 2));
        Eigen::Vector2f distortedCentered;
        distortedCentered[0] = distortedCenteredPixel[0] / K(0,0);
        distortedCentered[1] = distortedCenteredPixel[1] / K(1,1);

        float distortedRadius = std::sqrt(distortedCentered[0]*distortedCentered[0] + distortedCentered[1]*distortedCentered[1]);

        /*
            distortedRadius = undistortedRadius * (1 + distortion[0] * undistortedSqrRadius +
                                                    distortion[1] * undistortedSqrRadius*undistortedSqrRadius +
                                                    distortion[2] * undistortedSqrRadius*undistortedSqrRadius*undistortedSqrRadius);
        */

        float minUndistortedRadius = 0.0f;
        float maxUndistortedRadius = distortedRadius * 3.0f;

        for (unsigned iter = 0; iter < 10; iter++) {
            float centerUndistorted = (maxUndistortedRadius + minUndistortedRadius) * 0.5;
            float centerUndistortedSqr = centerUndistorted*centerUndistorted;

            float centerDistorted = centerUndistorted * (1 + distortion[0] * centerUndistortedSqr +
                                                    distortion[1] * centerUndistortedSqr*centerUndistortedSqr +
                                                    distortion[2] * centerUndistortedSqr*centerUndistortedSqr*centerUndistortedSqr);

            if (centerDistorted > distortedRadius)
                maxUndistortedRadius = centerUndistorted;
            else
                minUndistortedRadius = centerUndistorted;
        }

        float undistortedR = (maxUndistortedRadius + minUndistortedRadius) * 0.5f;

        float undistortedX = K(0, 2) + undistortedR/distortedRadius * distortedCenteredPixel[0];
        float undistortedY = K(1, 2) + undistortedR/distortedRadius * distortedCenteredPixel[1];

        kps[i].x_undistorted_times4 = (int)(undistortedX * 4.0f);
        kps[i].y_undistorted_times4 = (int)(undistortedY * 4.0f);
    }
}
#else
void InternalCalibration::undistortKeypoints(Keypoint *kps, unsigned count) const
{
    const Eigen::Matrix3f &K = internalCalib;
    const Eigen::Vector4f &distortion = distortion_theta;

    for (unsigned i = 0; i < count; i++) {
        Eigen::Vector2f distortedPixel(kps[i].x + 0.5f, kps[i].y + 0.5f);
        Eigen::Vector2f distortedCenteredPixel = distortedPixel - Eigen::Vector2f(K(0, 2), K(1, 2));
        Eigen::Vector2f distortedCentered;
        distortedCentered[0] = distortedCenteredPixel[0] / K(0,0);
        distortedCentered[1] = distortedCenteredPixel[1] / K(1,1);

        float distortedRadius = std::sqrt(distortedCentered[0]*distortedCentered[0] + distortedCentered[1]*distortedCentered[1]);

        /*
            distortedRadius = undistortedRadius * (1 + distortion[0] * undistortedSqrRadius +
                                                    distortion[1] * undistortedSqrRadius*undistortedSqrRadius +
                                                    distortion[2] * undistortedSqrRadius*undistortedSqrRadius*undistortedSqrRadius);
        */

        float minUndistortedRadius = 0.0f;
        float maxUndistortedRadius = distortedRadius * 3.0f;

        for (unsigned iter = 0; iter < 10; iter++) {
            float centerUndistorted = (maxUndistortedRadius + minUndistortedRadius) * 0.5;

            float theta = std::atan(centerUndistorted);
            float sqrTheta = theta*theta;

            float centerDistorted = theta * (1 + distortion[0] * sqrTheta +
                                                    distortion[1] * sqrTheta*sqrTheta +
                                                    distortion[2] * sqrTheta*sqrTheta*sqrTheta +
                                                    distortion[3] * sqrTheta*sqrTheta*sqrTheta*sqrTheta);

            if (centerDistorted > distortedRadius)
                maxUndistortedRadius = centerUndistorted;
            else
                minUndistortedRadius = centerUndistorted;
        }

        float undistortedR = (maxUndistortedRadius + minUndistortedRadius) * 0.5f;

        float undistortedX = K(0, 2) + undistortedR/distortedRadius * distortedCenteredPixel[0];
        float undistortedY = K(1, 2) + undistortedR/distortedRadius * distortedCenteredPixel[1];

        kps[i].x_undistorted_times4 = (int)(undistortedX * 4.0f);
        kps[i].y_undistorted_times4 = (int)(undistortedY * 4.0f);
    }
}
#endif


Camera::Camera(Map &map, Frame frame, InternalCalibration *internalCalib) : m_map(map), m_frame(std::move(frame)), m_internalCalib(internalCalib)
{
    m_pose.location.setZero();
    m_pose.world2eye.setIdentity();

    m_cameraIndex = map.m_cameras.size();
    map.m_cameras.push_back(*this);

    m_inversePNoT = computeInverseProjectionView(false);
}

Camera::Camera(Map &map, Frame frame, InternalCalibration *internalCalib, const CameraPose &pose) : Camera(map, std::move(frame), internalCalib)
{
    m_pose = pose;

    m_inversePNoT = computeInverseProjectionView(false);
}

Camera::~Camera()
{
    //m_mapMember.unlink();
}

void Camera::addNewTracks(Camera *otherCamera, unsigned count, NewTrack *tracks)
{
    auto& trackAllocator = m_map.getTrackAllocator();
    for (unsigned i = 0; i < count; i++) {
        assert(!keyPointIsReferenced(tracks[i].srcKeypointIdx));
        assert(!otherCamera->keyPointIsReferenced(tracks[i].dstKeypointIdx));

        Eigen::Vector3f d = tracks[i].location.head<3>() - m_pose.location * tracks[i].location[3];

        float projZ = m_pose.world2eye.block<1,3>(2, 0) * d;
        if (std::abs(projZ) < 1e-20f) continue;

        float rcpZ = tracks[i].location[3] / projZ;

        Track* track = trackAllocator.allocate(rcpZ, this, tracks[i].srcKeypointIdx);
        track->addObservation({
            .camera = otherCamera,
            .keypointIdx = (uint16_t)tracks[i].dstKeypointIdx,
            .matchScore = (uint16_t)tracks[i].matchScore,
            .residual_times_16 = 0
        });
    }
}



bool Camera::keyPointIsReferenced(unsigned idx) const
{
    for (const auto& t : m_trackReferences){
        if (idx == t.keypointIndex)
            return true;
    }
    return false;
}

void Camera::addTrackReference(TrackReference trackRef, Lock<Track>)
{
    m_trackReferences.push_back(trackRef);
}

Eigen::Matrix4f Camera::computeInverseProjectionView(bool includeTranslation)
{
    const auto &pose = getPose();

    Eigen::Matrix4f projectionInternal, projection, projectionExternal;

    const auto &K = getInternalCalibration()->internalCalib;

    projectionInternal.setIdentity();
    projectionInternal.block<2,2>(0,0) = K.block<2,2>(0,0) / K(2,2);
    projectionInternal(0,3) = K(0,2) / K(2,2);
    projectionInternal(1,3) = K(1,2) / K(2,2);

    projection.setIdentity();
    projection(2,2) = 0.0f;
    projection(2,3) = 1.0f;
    projection(3,3) = 0.0f;
    projection(3,2) = 1.0f;

    projectionExternal.setIdentity();
    projectionExternal.block<3,3>(0,0) = pose.world2eye;

    if (includeTranslation)
        throw "todo: think about this!";

    return (projectionInternal * projection * projectionExternal).inverse();
}



void Map::exportToPly(std::fstream &plyFile)
{
    plyFile
        << "ply" << std::endl
        << "format ascii 1.0" << std::endl
        << "element vertex " << m_tracks.size() << std::endl
        << "property float x" << std::endl
        << "property float y" << std::endl
        << "property float z" << std::endl
        << "end_header" << std::endl;


    for (auto &t : m_tracks) {

        auto &camera = *t.getAnchor().camera;

        const auto &pose = camera.getPose();

        Eigen::Matrix4f invP = camera.computeInverseProjectionView(false);

        auto srcKeypointIdx = t.getAnchor().keypointIdx;

        Eigen::Vector4f imageSpace;
        imageSpace[0] = camera.getFrame().getKeypoints()[srcKeypointIdx].x_undistorted_times4;
        imageSpace[1] = camera.getFrame().getKeypoints()[srcKeypointIdx].y_undistorted_times4;
        imageSpace[2] = t.getRcpZ() * 4.0f;
        imageSpace[3] = 4.0f;

        Eigen::Vector4f cam_relative_ws = invP * imageSpace;

        Eigen::Vector3f ws = pose.location + cam_relative_ws.head<3>() / cam_relative_ws[3];

        plyFile << ws[0] << ' ' << ws[1] << ' ' << ws[2] << std::endl;


    }
}

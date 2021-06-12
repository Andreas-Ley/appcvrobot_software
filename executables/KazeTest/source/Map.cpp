#include "Map.h"

#include <iostream>

void CameraMatches::addNewTracks(const CameraPose &pose, Camera *otherCamera, unsigned count, NewTrack *tracks)
{
    m_tracks.reserve(m_tracks.size() + count);
    for (unsigned i = 0; i < count; i++) {
        /*
            Eigen::Vector3f d = tracks[i].location.head<3>()/tracks[i].location[3] - pose.location;

            float projZ = pose.world2eye.block<3,1>(2, 0) * d;
            float rcpZ = 1.0f / projZ;

            // expand by tracks[i].location[3]

            Eigen::Vector3f d = tracks[i].location.head<3>()/tracks[i].location[3] - pose.location;

            float projZ = pose.world2eye.block<3,1>(2, 0) * d;
            float rcpZ = tracks[i].location[3] / (projZ * tracks[i].location[3]);
        */

        Eigen::Vector3f d = tracks[i].location.head<3>() - pose.location * tracks[i].location[3];

        float projZ = pose.world2eye.block<1,3>(2, 0) * d;
        if (std::abs(projZ) < 1e-20f) continue;

        float rcpZ = tracks[i].location[3] / projZ;

        m_tracks.push_back({
            .srcKeypointIdx = (uint16_t)tracks[i].srcKeypointIdx,
            .rcpZ = rcpZ,
            .rcpZ_stdDev = 0.0f,
            .matches = {
                {
                    .otherCamera = otherCamera,
                    .dstKeypointIdx = (uint16_t)tracks[i].dstKeypointIdx,
                    .matchScore = (uint16_t)tracks[i].matchScore,
                    .residual_times_16 = 0
                }
            }
        });
    }
}


Camera::Camera(Map &map, Frame frame, InternalCalibration *internalCalib) : m_map(map), m_frame(std::move(frame)), m_internalCalib(internalCalib)
{
    m_pose.location.setZero();
    m_pose.world2eye.setIdentity();

    map.m_cameras.push_back(*this);
}

Camera::Camera(Map &map, Frame frame, InternalCalibration *internalCalib, const CameraPose &pose) : Camera(map, std::move(frame), internalCalib)
{
    m_pose = pose;
}

Camera::~Camera()
{
    //m_mapMember.unlink();
}

void Camera::addNewTracks(Camera *otherCamera, unsigned count, NewTrack *tracks)
{
    m_matches.addNewTracks(m_pose, otherCamera, count, tracks);
}




void Map::exportToPly(std::fstream &plyFile)
{

    unsigned numPoints = 0;
    for (auto &camera : m_cameras)
        numPoints += camera.getMatches().getTracks().size();

    plyFile
        << "ply" << std::endl
        << "format ascii 1.0" << std::endl
        << "element vertex " << numPoints << std::endl
        << "property float x" << std::endl
        << "property float y" << std::endl
        << "property float z" << std::endl
        << "end_header" << std::endl;


    for (auto &camera : m_cameras) {

        const auto &pose = camera.getPose();

        Eigen::Matrix4f projectionInternal, projection, projectionExternal;

        const auto &K = camera.getInternalCalibration()->internalCalib;

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

        Eigen::Matrix4f invP = (projectionInternal * projection * projectionExternal).inverse();


        for (auto &t : camera.getMatches().getTracks()) {

            Eigen::Vector4f imageSpace;
            imageSpace[0] = camera.getFrame().getKeypoints()[t.srcKeypointIdx].x;
            imageSpace[1] = camera.getFrame().getKeypoints()[t.srcKeypointIdx].y;
            std::cout << t.rcpZ << std::endl;
            imageSpace[2] = t.rcpZ;
            imageSpace[3] = 1.0f;

            Eigen::Vector4f cam_relative_ws = invP * imageSpace;

            Eigen::Vector3f ws = pose.location + cam_relative_ws.head<3>() / cam_relative_ws[3];

            plyFile << ws[0] << ' ' << ws[1] << ' ' << ws[2] << std::endl;

        }
    }

}

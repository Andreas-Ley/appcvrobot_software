#pragma once

#include <boost/intrusive/list.hpp>

#include <Eigen/Dense>

#include "Frame.h"

#include <fstream>


/*

 - Fetch first frame
 - Fetch 2nd frame (repeat until movement)
 - match first & second
 - RANSAC F extraction -> compute E -> compute relative pose

 - triangulate 3D points

 ------------------------------------------

 - Fetch n-th frame
 - Match against previous k frames
    - RANSAC F or RANSAC pose, or both?
 - If it fails, match against triangulated points (for reinitialization)
 - Estimate pose
 - Optimize pose (only adjust new camera)
 - Triangulate new (yet not triangulated) matches
 - Match against triangulated points that should be in view frustrum

 - Bundle adjust until next frame
*/


namespace bi = boost::intrusive;

class Map;
class Camera;

struct InternalCalibration {
    Eigen::Matrix3f internalCalib;
    Eigen::Vector3f distortion_k;
};

struct CameraPose {
    Eigen::Vector3f location;
    Eigen::Matrix3f world2eye;
};

struct NewTrack {
    Eigen::Vector4f location;
    unsigned srcKeypointIdx;
    unsigned dstKeypointIdx;
    unsigned matchScore;
};

class CameraMatches {
    public:
        struct MatchList {
            Camera* otherCamera;
            uint16_t dstKeypointIdx; // Index into other frame's keypoint list
            uint16_t matchScore;
            uint16_t residual_times_16;
        };
        struct Track {
            uint16_t srcKeypointIdx; // Index into frame's keypoint list
            float rcpZ;
            float rcpZ_stdDev;
            std::vector<MatchList> matches;
        };

        void addNewTracks(const CameraPose &pose, Camera *otherCamera, unsigned count, NewTrack *tracks);

        inline const std::vector<Track> &getTracks() const { return m_tracks; }
    protected:
        std::vector<Track> m_tracks;
};

class Camera {
    public:
        typedef bi::list_member_hook<> MapListEntry;
        MapListEntry m_mapMember;

        Camera(Map &map, Frame frame, InternalCalibration *internalCalib);
        Camera(Map &map, Frame frame, InternalCalibration *internalCalib, const CameraPose &pose);
        ~Camera();

        Camera(const Camera &) = delete;
        void operator=(const Camera &) = delete;

        inline const Frame &getFrame() const { return m_frame; }

        inline const CameraMatches &getMatches() const { return m_matches; }
        inline const CameraPose &getPose() const { return m_pose; }
        inline InternalCalibration *getInternalCalibration() const { return m_internalCalib; }

        void addNewTracks(Camera *otherCamera, unsigned count, NewTrack *tracks);
    protected:
        Map &m_map;
        Frame m_frame;

        CameraPose m_pose;
        InternalCalibration *m_internalCalib;

        CameraMatches m_matches;

};

class Map {
    public:
        typedef bi::list<Camera, bi::member_hook<Camera, Camera::MapListEntry, &Camera::m_mapMember>> CameraList;

        CameraList m_cameras;


        void exportToPly(std::fstream &plyFile);
};
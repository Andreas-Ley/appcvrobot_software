#pragma once

#include <boost/intrusive/list.hpp>

#include <Eigen/Dense>

#include "Frame.h"
#include "Track.h"

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

template<class T>
class Lock
{
    private:
        Lock() = default;

        friend T;
};



class Map;
class Camera;

struct Keypoint;

struct InternalCalibration {
    Eigen::Matrix3f internalCalib;
    Eigen::Vector4f distortion_theta;

    void undistortKeypoints(Keypoint *kps, unsigned count) const;
};

struct CameraPose {
    Eigen::Vector3f location;
    Eigen::Matrix3f world2eye;

    CameraPose getPoseRelativeTo(const CameraPose &anchor) const;
};

struct NewTrack {
    Eigen::Vector4f location;
    unsigned srcKeypointIdx;
    unsigned dstKeypointIdx;
    unsigned matchScore;
};

class Camera {
    public:
        struct TrackReference {
            Track* track = nullptr;
            uint16_t keypointIndex;
            uint16_t matchScore;
        };
        typedef bi::list_member_hook<> MapListEntry;
        MapListEntry m_mapMember;

        Camera(Map &map, Frame frame, InternalCalibration *internalCalib);
        Camera(Map &map, Frame frame, InternalCalibration *internalCalib, const CameraPose &pose);
        ~Camera();

        Camera(const Camera &) = delete;
        void operator=(const Camera &) = delete;

        inline unsigned getCameraIndex() const { return m_cameraIndex; }

        inline const Frame &getFrame() const { return m_frame; }

        inline const CameraPose &getPose() const { return m_pose; }
        inline InternalCalibration *getInternalCalibration() const { return m_internalCalib; }

        Eigen::Matrix4f computeInverseProjectionView(bool includeTranslation);
        const Eigen::Matrix4f &getInverseProjectionViewNoTranslation() { return m_inversePNoT; }

        void addNewTracks(Camera *otherCamera, unsigned count, NewTrack *tracks);

        bool keyPointIsReferenced(unsigned idx) const;

        void addTrackReference(TrackReference trackRef, Lock<Track>);

        inline const std::vector<TrackReference> &getTrackReferences() const { return  m_trackReferences; }
    protected:
        Map &m_map;
        unsigned m_cameraIndex;
        Frame m_frame;

        CameraPose m_pose;
        InternalCalibration *m_internalCalib;

        Eigen::Matrix4f m_inversePNoT;

        std::vector<TrackReference> m_trackReferences;

};

class Map {
    public:
        typedef bi::list<Camera, bi::member_hook<Camera, Camera::MapListEntry, &Camera::m_mapMember>> CameraList;

        CameraList m_cameras;

        void exportToPly(std::fstream &plyFile);

        TrackList& getTracks() { return m_tracks; }

    protected:
        TrackList m_tracks;
};
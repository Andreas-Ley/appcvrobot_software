
#pragma once

#include <boost/intrusive/list.hpp>

#include <vector>
#include <stdint.h>

class Camera;


namespace bi = boost::intrusive;


class Track {
    public:
        Track (float rcpZ, Camera* anchorCamera, uint16_t keypointIdx);

        typedef bi::list_member_hook<> TrackListEntry;
        TrackListEntry m_trackMember;

        struct Observation {
            Camera* camera = nullptr;
            uint16_t keypointIdx = ~0u;
            uint16_t matchScore = 0;
            uint16_t residual_times_16 = 0;
        };

        Observation &getAnchor() { return m_observations.front(); }

        void addObservation(Observation observation);

        inline const std::vector<Observation> &getObservation() const {return m_observations; }

        inline float getRcpZ() const { return m_rcpZ; }
    protected:
        std::vector<Observation> m_observations;
        float m_rcpZ = -1.0f;
        float m_rcpZ_stdDev = -1.0f;

};

class TrackList
{
    public:
        template<typename ...Args>
        Track *allocate(Args &&... args) { m_tracks.push_back(*(new Track(std::forward<Args>(args)...))); return &m_tracks.back(); }
        void free(Track *track) { delete track; }

        auto begin() { return m_tracks.begin(); }
        auto end() { return m_tracks.end(); }

        auto size() const { return m_tracks.size(); }
    protected:

        typedef bi::list<Track, bi::member_hook<Track, Track::TrackListEntry, &Track::m_trackMember>> TrackListType;
        TrackListType m_tracks;
};

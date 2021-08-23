#include "Track.h"

#include "Map.h"

Track::Track (float rcpZ, Camera* anchorCamera, uint16_t keypointIdx):
    m_rcpZ(rcpZ)
{
    addObservation(Observation{.camera=anchorCamera, .keypointIdx=keypointIdx});
}

void Track::addObservation(Observation observation)
{
    m_observations.push_back(observation);
    observation.camera->addTrackReference({.track = this, .keypointIndex = observation.keypointIdx, .matchScore = observation.matchScore}, {});
}
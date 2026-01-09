#ifndef VEHICLESTATETRACKER_H
#define VEHICLESTATETRACKER_H

#include "abv_navigation/IStateFetcher.h"
#include "common/Configurations.h"
#include "common/ConfigurationManager.h"
#include "RosStatePublisher.h"
#include <memory>
#include <thread> 

class VehicleStateTracker
{
public:
    VehicleStateTracker(const std::string& aRigidBodyName);
    ~VehicleStateTracker();

    enum class FetcherType
    {
        SIMULATED, 
        OPTITRACK, 
        VICON, 
        NUM_TYPES
    };

    void stateTrackerLoop(); 

    bool doStateTracking() {std::lock_guard<std::mutex> lock(mStateTrackingMutex); return mDoStateTracking; }
    void setStateTracking(bool aFlag) {std::lock_guard<std::mutex> lock(mStateTrackingMutex); mDoStateTracking = aFlag; }

private:
    // polymorphic state fetcher interface so we arent tied to optiTrack
    std::shared_ptr<IStateFetcher> mStateFetcher; // state fetcher interface class 
    RosStatePublisher mStatePublisher; 
    StateTrackerConfig mConfig; 

    bool mDoStateTracking;

    std::mutex mStateTrackingMutex; 
    std::thread mStateTrackingThread;  

    VehicleStateTracker::FetcherType toEnum(std::string aTrackerType);
};
#endif // VEHICLESTATETRACKER_H
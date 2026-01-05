#ifndef VEHICLESTATETRACKER_H
#define VEHICLESTATETRACKER_H

#include "IStateFetcher.h"
#include "Configurations.h"
#include "ConfigurationManager.h"
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

    Eigen::Vector3d getCurrentPose() { std::lock_guard<std::mutex> lock(mCurrentPoseMutex); return mCurrentPose; }
    Eigen::Vector3d getCurrentVelocity() {std::lock_guard<std::mutex> lock(mCurrentVelocityMutex); return mCurrentVelocity; }

    Eigen::Matrix<float, 12, 1> getCurrentState() { std::scoped_lock lock(mCurrentStateMutex); return mCurrentState; }

    bool doStateTracking() {std::lock_guard<std::mutex> lock(mStateTrackingMutex); return mDoStateTracking; }
    void setStateTracking(bool aFlag) {std::lock_guard<std::mutex> lock(mStateTrackingMutex); mDoStateTracking = aFlag; }

private:
    // polymorphic state fetcher interface so we arent tied to optiTrack
    std::shared_ptr<IStateFetcher> mStateFetcher; // state fetcher interface class 
    StateTrackerConfig mConfig; 

    bool mDoStateTracking;

    Eigen::Vector3d mCurrentPose;
    Eigen::Vector3d mCurrentVelocity;
    Eigen::Matrix<float, 12, 1> mCurrentState;

    std::mutex mCurrentStateMutex;
    std::mutex mCurrentPoseMutex;
    std::mutex mCurrentVelocityMutex;
    std::mutex mStateTrackingMutex; 

    std::thread mStateTrackingThread;  

    VehicleStateTracker::FetcherType toEnum(std::string aTrackerType);
    void setCurrentState(const Eigen::Matrix<float, 12, 1>& aState) { std::scoped_lock lock(mCurrentStateMutex); mCurrentState = aState; }
    
    std::vector<double> toVector(const Eigen::Matrix<float, 12, 1>& aState);

};
#endif // VEHICLESTATETRACKER_H
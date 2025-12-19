
#include "abv_controller/VehicleStateTracker.h"
#include "abv_controller/OptitrackStateFetcher_LibMocap.h"
#include "abv_controller/SimulatedStateFetcher.h"
#include "abv_controller/RateController.hpp"
#include "plog/Log.h"
#include <thread>

using OptitrackStateFetcher = OptitrackStateFetcher_LibMocap; 

VehicleStateTracker::VehicleStateTracker(const std::string& aRigidBodyName) : 
            mConfig(ConfigurationManager::getInstance()->getStateTrackerConfig()), 
            mCurrentState(Eigen::VectorXf::Zero(12))
{
    FetcherType typeToMake = toEnum(mConfig.mInterface); 

    switch (typeToMake)
    {
        case FetcherType::SIMULATED:
            
            mStateFetcher = std::make_shared<SimulatedStateFetcher>(); 
            LOGD << "Configuring ABV to use Simulated state feedback"; 
            
            break;
        case FetcherType::OPTITRACK: 
            
            mStateFetcher = std::make_shared<OptitrackStateFetcher>(mConfig.mServerIp, mConfig.mLocalIp, mConfig.mRigidBodyId, aRigidBodyName); 
            LOGD << "Configuring ABV to use OptiTrack for state feedback"; 
            
            break;
        default:
            break;
    } 

    mStateTrackingThread = std::thread(&VehicleStateTracker::stateTrackerLoop, this); 
}

VehicleStateTracker::~VehicleStateTracker()
{
    setStateTracking(false); 

    if(mStateTrackingThread.joinable())
    {
        LOGD << "Joining state tracking thread"; 
        mStateTrackingThread.join(); 
    }

}

VehicleStateTracker::FetcherType VehicleStateTracker::toEnum(std::string aTrackerType)
{
    VehicleStateTracker::FetcherType enumToReturn; 

    if("Simulated" == aTrackerType || "simulated" == aTrackerType)
    {
        enumToReturn = VehicleStateTracker::FetcherType::SIMULATED; 
    }
    else if ("Optitrack" == aTrackerType || "optitrack" == aTrackerType || "optiTrack" == aTrackerType || "OptiTrack" == aTrackerType)
    {
        enumToReturn = VehicleStateTracker::FetcherType::OPTITRACK; 
    }
    else
    {
        LOGE << "Unsupported state fetcher type: " << aTrackerType; 
    }

    return enumToReturn; 
}

void VehicleStateTracker::stateTrackerLoop()
{  
    RateController rate(mConfig.mRate); 
    
    if(!mStateFetcher->init())
    {
        LOGE << "Could not initialize state fetcher of type: " << mConfig.mInterface; 
        return; 
    }
    
    LOGD << "Starting state tracking thread"; 
    setStateTracking(true); 

    while(doStateTracking())
    {
        rate.start(); 

        auto state = mStateFetcher->fetchState();
        setCurrentState(state); 

        rate.block(); 
    }
}
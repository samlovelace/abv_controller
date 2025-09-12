
#include "abv_controller/VehicleStateTracker.h"


#ifdef USE_NATNET
#include "abv_controller/OptitrackStateFetcher_NatNet.h"
using OptitrackStateFetcher = OptitrackStateFetcher_NatNet;
#else
#include "abv_controller/OptitrackStateFetcher_LibMocap.h"
using OptitrackStateFetcher = OptitrackStateFetcher_LibMocap;
#endif

#include "abv_controller/SimulatedStateFetcher.h"
#include "plog/Log.h"
#include <thread>

VehicleStateTracker::VehicleStateTracker(const std::string& aRigidBodyName) : mConfig(ConfigurationManager::getInstance()->getStateTrackerConfig()), mCurrentState(Eigen::VectorXd::Zero(6))
{
    FetcherType typeToMake = toEnum(mConfig.mInterface); 

    switch (typeToMake)
    {
    case FetcherType::SIMULATED:
        mStateFetcher = std::make_shared<SimulatedStateFetcher>(); 
        LOGD << "Configuring ABV to use Simulated state feedback"; 
        break;
    case FetcherType::OPTITRACK: 
        mStateFetcher = std::make_shared<OptitrackStateFetcher>(mConfig.mNetwork, mConfig.mRigidBodyId, aRigidBodyName); 
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

// TODO: figure out what behavior we want for this. Constantly state tracking or able to stop and restart?
void VehicleStateTracker::stateTrackerLoop()
{   
    LOGD << "Starting state tracking thread"; 

    const std::chrono::duration<double> loop_duration(1.0 / mConfig.mRate);
    
    if(!mStateFetcher->init())
    {
        LOGE << "Could not initialize state fetcher of type: " << mConfig.mInterface; 
        return; 
    }

    while(doStateTracking())
    {
        auto start = std::chrono::steady_clock::now(); 

        mStateFetcher->fetchState(); 

        // Calculate the time taken for the loop iteration
        auto loop_end = std::chrono::steady_clock::now();
        auto elapsed = loop_end - start;

        // Sleep for the remaining time to maintain the frequency
        if (elapsed < loop_duration) {
            std::this_thread::sleep_for(loop_duration - elapsed);
        } else {
            LOGE << "Loop overrun! Elapsed time: " 
                      << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()
                      << " ms\n";
        }
    }
}





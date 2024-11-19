
#include "abv_controller/VehicleStateTracker.h"
#include "abv_controller/OptitrackStateFetcher.h"
#include "abv_controller/SimulatedStateFetcher.h"
#include "plog/Log.h"

VehicleStateTracker::VehicleStateTracker() : mConfig(ConfigurationManager::getInstance()->getStateTrackerConfig())
{
    FetcherType typeToMake = toEnum(mConfig.mInterface); 

    switch (typeToMake)
    {
    case FetcherType::SIMULATED:
        mStateFetcher = std::make_shared<SimulatedStateFetcher>(); 
        LOGD << "Configuring ABV to use Simulated state feedback"; 
        break;
    case FetcherType::OPTITRACK: 
        mStateFetcher = std::make_shared<OptitrackStateFetcher>(); 
        LOGD << "Configuring ABV to use OptiTrack for state feedback"; 
        break;
    default:
        break;
    }
}

VehicleStateTracker::~VehicleStateTracker()
{
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




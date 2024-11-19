
#include "abv_controller/Vehicle.h"
#include "plog/Log.h"


Vehicle::Vehicle(/* args */) : 
    mThrusterCommander(std::make_unique<ThrusterCommander>()), mStateTracker(std::make_shared<VehicleStateTracker>()),
    mStatePublisher(std::make_unique<RosStatePublisher>(mStateTracker)), mLastInputRecvdAt(std::chrono::steady_clock::now()),
    mStaleInputThreshold(std::chrono::duration<double>(std::chrono::milliseconds(500)))
{
    
}

Vehicle::~Vehicle()
{
}

void Vehicle::doThrusterControl()
{
    Eigen::Vector3d controlInput = getControlInput(); 
    mThrusterCommander->commandThrusters(controlInput);
}

void Vehicle::doPoseControl()
{
    LOGW << "Goal Pose: " << getGoalPose(); 
    // 1. get the current pose of the vehicle and subtract from goal pose

    // 2. give error to mController to calc control input vector

    // 3. setControlInput()

    // 4. doThrusterControl()
}

void Vehicle::setControlInput(Eigen::Vector3d aControlInput)
{
    // pls dont banish me for using a single mutex on two resources 
    std::lock_guard<std::mutex> lock(mControlInputMutex); 
    mControlInput = aControlInput;
    mLastInputRecvdAt = std::chrono::steady_clock::now(); 
}
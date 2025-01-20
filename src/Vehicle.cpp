
#include "abv_controller/Vehicle.h"
#include "plog/Log.h"
#include <thread>


Vehicle::Vehicle(/* args */) : 
    mThrusterCommander(std::make_unique<ThrusterCommander>()), mStateTracker(std::make_shared<VehicleStateTracker>()),
    mStatePublisher(std::make_unique<RosStatePublisher>(mStateTracker)),mController(std::make_unique<Controller>()),
    mLastInputRecvdAt(std::chrono::steady_clock::now()),mStaleInputThreshold(std::chrono::duration<double>(std::chrono::milliseconds(500)))
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
    Eigen::Vector3d poseError = mStateTracker->getCurrentPose() - getGoalPose(); 
    Eigen::Vector3d controlInput = mController->computeControlInput(poseError); 
    setControlInput(controlInput);
    doThrusterControl(); 
}

void Vehicle::setControlInput(Eigen::Vector3d aControlInput)
{
    // pls dont banish me for using a single mutex on two resources 
    std::lock_guard<std::mutex> lock(mControlInputMutex); 
    mControlInput = aControlInput;
    mLastInputRecvdAt = std::chrono::steady_clock::now(); 
}
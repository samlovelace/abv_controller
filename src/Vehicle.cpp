
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

void Vehicle::doVelocityControl()
{
    Eigen::Vector3d velError = mStateTracker->getCurrentVelocity() - getGoalVelocity(); 
    Eigen::Vector3d controlInput = mController->computeControlInput(velError); 
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

void Vehicle::setGoalPose(Eigen::Vector3d aGoalPose) 
{
    std::lock_guard<std::mutex> lock(mGoalPoseMutex); 
    mGoalPose = aGoalPose;
}
void Vehicle::setGoalVelocity(Eigen::Vector3d aGoalVel)
{
    std::lock_guard<std::mutex> lock(mGoalVelocityMutex); 
    mGoalVelocity = aGoalVel; 
}

Eigen::Vector3d Vehicle::getGoalPose() 
{
    std::lock_guard<std::mutex> lock(mGoalPoseMutex); 
    return mGoalPose; 
}

Eigen::Vector3d Vehicle::getGoalVelocity() 
{
    std::lock_guard<std::mutex> lock(mGoalVelocityMutex);
    return mGoalVelocity; 
}

Eigen::Vector3d Vehicle::getControlInput() 
{
    std::lock_guard<std::mutex> lock(mControlInputMutex); 
    return mControlInput; 
}

bool Vehicle::isControlInputStale() 
{
    return std::chrono::steady_clock::now() - mLastInputRecvdAt > mStaleInputThreshold ? true : false;
}
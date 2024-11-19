#ifndef VEHICLE_H
#define VEHICLE_H

#include <eigen3/Eigen/Dense>
#include <mutex> 
#include <memory>
#include "abv_controller/ThrusterCommander.h"
#include "abv_controller/VehicleStateTracker.h"
#include "abv_controller/RosStatePublisher.h"

class Vehicle
{
public:
    Vehicle();
    ~Vehicle();

    void doThrusterControl(); 
    void doPoseControl(); 

    void setGoalPose(Eigen::Vector3d aGoalPose) {std::lock_guard<std::mutex> lock(mGoalPoseMutex); mGoalPose = aGoalPose;}
    Eigen::Vector3d getGoalPose() {std::lock_guard<std::mutex> lock(mGoalPoseMutex); return mGoalPose; }
    void setControlInput(Eigen::Vector3d aControlInput);
    Eigen::Vector3d getControlInput() {std::lock_guard<std::mutex> lock(mControlInputMutex); return mControlInput; }

    bool isControlInputStale() {return std::chrono::steady_clock::now() - mLastInputRecvdAt > mStaleInputThreshold ? true : false;}


private:
    Eigen::Vector3d mGoalPose; 
    Eigen::Vector3d mControlInput; 
    std::chrono::steady_clock::time_point mLastInputRecvdAt; 
    std::chrono::duration<double> mStaleInputThreshold;  

    std::mutex mGoalPoseMutex; 
    std::mutex mControlInputMutex; 

    std::unique_ptr<ThrusterCommander> mThrusterCommander;
    std::shared_ptr<VehicleStateTracker> mStateTracker; 
    std::unique_ptr<RosStatePublisher> mStatePublisher; 


};
#endif // VEHICLE_H


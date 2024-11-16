#ifndef VEHICLE_H
#define VEHICLE_H

#include <eigen3/Eigen/Dense>
#include <mutex> 
#include <memory>
#include "abv_controller/ThrusterCommander.h"

class Vehicle
{
public:
    Vehicle(/* args */);
    ~Vehicle();

    void doThrusterControl(); 
    void doPoseControl(); 

    void setGoalPose(Eigen::Vector3d aGoalPose) {std::lock_guard<std::mutex> lock(mGoalPoseMutex); mGoalPose = aGoalPose;}
    Eigen::Vector3d getGoalPose() {std::lock_guard<std::mutex> lock(mGoalPoseMutex); return mGoalPose; }
    void setControlInput(Eigen::Vector3d aControlInput) {std::lock_guard<std::mutex> lock(mControlInputMutex); mControlInput = aControlInput;}
    Eigen::Vector3d getControlInput() {std::lock_guard<std::mutex> lock(mControlInputMutex); return mControlInput; }


private:
    Eigen::Vector3d mGoalPose; 
    Eigen::Vector3d mControlInput; 

    std::mutex mGoalPoseMutex; 
    std::mutex mControlInputMutex; 

    std::unique_ptr<ThrusterCommander> mThrusterCommander; 

};
#endif // VEHICLE_H


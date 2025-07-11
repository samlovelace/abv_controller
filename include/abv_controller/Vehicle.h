#ifndef VEHICLE_H
#define VEHICLE_H

#include <eigen3/Eigen/Dense>
#include <mutex> 
#include <memory>
#include "abv_controller/ThrusterCommander.h"
#include "abv_controller/VehicleStateTracker.h"
#include "abv_controller/RosStatePublisher.h"
#include "abv_controller/Controller.h"

class Vehicle
{
public:
    Vehicle();
    ~Vehicle();

    bool init(); 

    void doThrusterControl(); 
    void doPoseControl(); 
    void doVelocityControl(); 

    void setGoalPose(Eigen::Vector3d aGoalPose);
    void setGoalVelocity(Eigen::Vector3d aGoalVel); 
    void setControlInput(Eigen::Vector3d aControlInput); 

    Eigen::Vector3d getGoalPose();
    Eigen::Vector3d getGoalVelocity(); 
    Eigen::Vector3d getControlInput(); 

    bool isControlInputStale(); 

private:
    Eigen::Vector3d mGoalPose; 
    Eigen::Vector3d mGoalVelocity; 
    Eigen::Vector3d mControlInput; 
    std::chrono::steady_clock::time_point mLastInputRecvdAt; 
    std::chrono::duration<double> mStaleInputThreshold;  

    std::mutex mGoalPoseMutex; 
    std::mutex mControlInputMutex; 
    std::mutex mGoalVelocityMutex;

    std::unique_ptr<ThrusterCommander> mThrusterCommander;
    std::shared_ptr<VehicleStateTracker> mStateTracker; 
    std::unique_ptr<RosStatePublisher> mStatePublisher; 
    std::unique_ptr<Controller> mController; 


};
#endif // VEHICLE_H


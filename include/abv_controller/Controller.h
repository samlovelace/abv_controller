#ifndef CONTROLLER_H
#define CONTROLLER_H
 
#include <eigen3/Eigen/Dense> 
#include <abv_controller/Configurations.h>
#include <chrono> 
 
class Controller 
{ 
public:
    Controller();
    ~Controller();

    Eigen::Vector3d computeControlInput(Eigen::Vector3d aPoseError); 

private:
    ControllerConfig mConfig; 
    std::chrono::steady_clock::time_point mPrevTime; 
    Eigen::Vector3d mPrevPoseError; 
    Eigen::Vector3d mPrevPoseErrorIntegral; 
    
    
    Eigen::Vector3d PID(Eigen::Vector3d aPoseError); 
   
};
#endif //CONTROLLER_H
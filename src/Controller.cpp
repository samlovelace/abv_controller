
#include "abv_controller/Controller.h"
#include "abv_controller/ConfigurationManager.h"

Controller::Controller() : mConfig(ConfigurationManager::getInstance()->getControllerConfig())
{

}

Controller::~Controller()
{

}

Eigen::Vector3d Controller::computeControlInput(Eigen::Vector3d aPoseError)
{
    return PID(aPoseError);
}

Eigen::Vector3d Controller::PID(Eigen::Vector3d aPoseError)
{
    using namespace std::chrono; 
    Eigen::Vector3d poseErrorDeriv; 
    Eigen::Vector3d poseErrorIntegral;
    Eigen::Vector3d controlInput; 

    for(int i = 0; i < 2; i++)
    {
        auto dt = duration_cast<milliseconds>(steady_clock::now() - mPrevTime).count();  
        poseErrorDeriv[i] = aPoseError[i] - mPrevPoseError[i] / dt;
        poseErrorIntegral[i] = mPrevPoseErrorIntegral[i] + aPoseError[i]*dt; 

        controlInput[i] = mConfig.Kp[i] * aPoseError[i] + 
                          mConfig.Ki[i] * poseErrorIntegral[i] + 
                          mConfig.Kd[i] * poseErrorDeriv[i];
    }

    mPrevPoseError = aPoseError; 
    mPrevPoseErrorIntegral = poseErrorIntegral; 
    mPrevTime = steady_clock::now();

    return controlInput;
}

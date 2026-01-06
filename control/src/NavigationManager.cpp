
#include "abv_controller/NavigationManager.h"
#include "common/RosTopicManager.h"

NavigationManager::NavigationManager(/* args */) : mAcquiredState(false)
{
    RosTopicManager::getInstance()->createSubscriber<robot_idl::msg::AbvState>("abv/state", 
                                                                               std::bind(&NavigationManager::stateCallback,
                                                                                         this, 
                                                                                         std::placeholders::_1)); 

                                                                                
}

NavigationManager::~NavigationManager()
{

}

void NavigationManager::stateCallback(const robot_idl::msg::AbvState::SharedPtr aMsg)
{
    Eigen::Matrix<double, 12, 1> state; 
    
    state[0] = aMsg->position.x; 
    state[1] = aMsg->position.y; 
    state[2] = aMsg->position.z; 

    state[3] = aMsg->velocity.x; 
    state[4] = aMsg->velocity.y; 
    state[5] = aMsg->velocity.z; 

    state[6] = aMsg->orientation.z; 
    state[7] = aMsg->orientation.y; 
    state[8] = aMsg->orientation.x; 

    state[9] = aMsg->ang_vel.z;
    state[10] = aMsg->ang_vel.y;
    state[11] = aMsg->ang_vel.x; 

    {
        std::lock_guard<std::mutex> lock(mCurrentStateMutex); 
        mCurrentState = state; 

        if(!mAcquiredState)
        {
            // if this is the first callback invoked, set that we have acquired the state data 
            mAcquiredState = true; 
        }
    }
}

void NavigationManager::setState(const Eigen::Matrix<double, 12, 1>& aState)
{
    std::lock_guard<std::mutex> lock(mCurrentStateMutex); 
    mCurrentState = aState; 
}

bool NavigationManager::hasAcquiredStateData()
{
    return mAcquiredState; 
}

Eigen::Vector3d NavigationManager::getCurrentPose()
{
    std::lock_guard<std::mutex> lock(mCurrentStateMutex); 

    Eigen::Vector3d pose; 
    pose << mCurrentState[0], mCurrentState[1], mCurrentState[6]; 
    
    return pose; 
}

Eigen::Vector3d NavigationManager::getCurrentVel()
{
    std::lock_guard<std::mutex> lock(mCurrentStateMutex); 

    Eigen::Vector3d vel; 
    vel << mCurrentState[3], mCurrentState[4], mCurrentState[9]; 

    return vel; 
}

Eigen::Matrix<double, 12, 1> NavigationManager::getCurrentState()
{
    std::lock_guard<std::mutex> lock(mCurrentStateMutex); 
    return mCurrentState; 
}

#include "abv_controller/SimulatedStateFetcher.h"
#include "abv_controller/RosTopicManager.h"


SimulatedStateFetcher::SimulatedStateFetcher(/* args */)
{
    
}

SimulatedStateFetcher::~SimulatedStateFetcher()
{
}

bool SimulatedStateFetcher::init()
{
    RosTopicManager::getInstance()->createSubscriber<robot_idl::msg::AbvState>("abv_state_simulated", 
                                                std::bind(&SimulatedStateFetcher::stateCallback, this, std::placeholders::_1));

    return true; 
}

void SimulatedStateFetcher::stateCallback(robot_idl::msg::AbvState::SharedPtr aSimState)
{
    Eigen::Matrix<double, 6, 1> state; 
    state[0]  = aSimState->position.x;
    state[1] = aSimState->position.y; 
    state[2] = aSimState->position.yaw; 

    state[3] = aSimState->velocity.x; 
    state[4] = aSimState->velocity.y; 
    state[5] = aSimState->velocity.yaw; 

    // thread safe setting of state
    setState(state); 
}

void SimulatedStateFetcher::setState(Eigen::Matrix<double, 6, 1> aState)
{
    std::lock_guard<std::mutex> lock(mStateMutex); 
    mState = aState; 
}

Eigen::Matrix<double, 6,1> SimulatedStateFetcher::fetchState()
{
    std::lock_guard<std::mutex> lock(mStateMutex); 
    return mState; 
}

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
    RosTopicManager::getInstance()->createSubscriber<robot_idl::msg::AbvState>("abv/sim/state", 
                                                std::bind(&SimulatedStateFetcher::stateCallback, this, std::placeholders::_1));

    return true; 
}

void SimulatedStateFetcher::stateCallback(robot_idl::msg::AbvState::SharedPtr aSimState)
{
    Eigen::Matrix<float, 13, 1> state; 
    state[0] = aSimState->position.x;
    state[1] = aSimState->position.y; 
    state[2] = aSimState->position.z; 

    state[3] = aSimState->velocity.x; 
    state[4] = aSimState->velocity.y; 
    state[5] = aSimState->velocity.z; 

    state[6] = aSimState->orientation.x; 
    state[7] = aSimState->orientation.y; 
    state[8] = aSimState->orientation.z; 

    state[9] = aSimState->ang_vel.x; 
    state[10] = aSimState->ang_vel.y; 
    state[11] = aSimState->ang_vel.z; 

    // thread safe setting of state
    setState(state); 
}

void SimulatedStateFetcher::setState(Eigen::Matrix<float, 13, 1> aState)
{
    std::lock_guard<std::mutex> lock(mStateMutex); 
    mState = aState; 
}

Eigen::Matrix<float, 13,1> SimulatedStateFetcher::fetchState()
{
    std::lock_guard<std::mutex> lock(mStateMutex); 
    return mState; 
}
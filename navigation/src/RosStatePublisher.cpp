
#include "common/RosTopicManager.h"
#include "common/RateController.hpp"

#include "abv_navigation/RosStatePublisher.h"

#include <thread>
#include <chrono> 
#include "plog/Log.h"

RosStatePublisher::RosStatePublisher() : mTopicName("abv/state")
{
    RosTopicManager::getInstance()->createPublisher<robot_idl::msg::AbvState>(mTopicName);
}

RosStatePublisher::~RosStatePublisher()
{

}

void RosStatePublisher::publish(const Eigen::Matrix<float, 12, 1>& aState)
{
    RosTopicManager::getInstance()->publishMessage<robot_idl::msg::AbvState>(mTopicName, convertToIdlMsg(aState)); 
}

robot_idl::msg::AbvState RosStatePublisher::convertToIdlMsg(const Eigen::Matrix<float, 12, 1>& aStateVector)
{
    robot_idl::msg::Vec3 position; 
    robot_idl::msg::Vec3 velocity;

    position.x = aStateVector[0]; 
    position.y = aStateVector[1]; 
    position.z = aStateVector[2]; 

    velocity.x = aStateVector[3]; 
    velocity.y = aStateVector[4]; 
    velocity.z = aStateVector[5]; 
    
    robot_idl::msg::Vec3 orientation; 
    orientation.x = aStateVector[8];
    orientation.y = aStateVector[7]; 
    orientation.z = aStateVector[6]; 

    robot_idl::msg::Vec3 ang_vel; 
    ang_vel.x = aStateVector[11];
    ang_vel.y = aStateVector[10]; 
    ang_vel.z = aStateVector[9]; 

    robot_idl::msg::AbvState state; 
    state.set__position(position); 
    state.set__velocity(velocity); 
    state.set__orientation(orientation); 
    state.set__ang_vel(ang_vel); 

    return state; 
}


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

void RosStatePublisher::publish(const AbvState& aState)
{
    RosTopicManager::getInstance()->publishMessage<robot_idl::msg::AbvState>(mTopicName, convertToIdlMsg(aState)); 
}

robot_idl::msg::AbvState RosStatePublisher::convertToIdlMsg(const AbvState& aStateVector)
{
    robot_idl::msg::Vec3 position; 
    robot_idl::msg::Vec3 velocity;

    position.x = aStateVector.x; 
    position.y = aStateVector.y; 
    position.z = 0.0; 

    velocity.x = aStateVector.vx; 
    velocity.y = aStateVector.vy; 
    velocity.z = 0.0; 
    
    robot_idl::msg::Vec3 orientation; 
    orientation.x = 0.0;
    orientation.y = 0.0; 
    orientation.z = aStateVector.theta;  

    robot_idl::msg::Vec3 ang_vel; 
    ang_vel.x = 0.0;
    ang_vel.y = 0.0; 
    ang_vel.z = aStateVector.omega; 

    robot_idl::msg::AbvState state; 
    state.set__position(position); 
    state.set__velocity(velocity); 
    state.set__orientation(orientation); 
    state.set__ang_vel(ang_vel); 

    return state; 
}

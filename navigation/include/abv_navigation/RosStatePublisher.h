#ifndef ROSSTATEPUBLISHER_H
#define ROSSTATEPUBLISHER_H

#include <thread> 

#include "common/Configurations.h"
#include "common/AbvState.hpp"
#include "robot_idl/msg/abv_state.hpp"

class RosStatePublisher
{
public:
    RosStatePublisher();
    ~RosStatePublisher();

    void publish(const AbvState& aState); 

private:
    
    std::string mTopicName; 
    robot_idl::msg::AbvState convertToIdlMsg(const AbvState& aStateVector); 
};

#endif // ROSSTATEPUBLISHER_H

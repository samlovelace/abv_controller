#ifndef ROSSTATEPUBLISHER_H
#define ROSSTATEPUBLISHER_H

#include "common/Configurations.h"
#include "robot_idl/msg/abv_state.hpp"
#include <thread> 

class RosStatePublisher
{
public:
    RosStatePublisher();
    ~RosStatePublisher();

    void publish(const Eigen::Matrix<float, 12, 1>& aState); 

private:
    
    std::string mTopicName; 
    robot_idl::msg::AbvState convertToIdlMsg(const Eigen::Matrix<float, 12, 1>& aStateVector); 
};

#endif // ROSSTATEPUBLISHER_H

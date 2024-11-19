#ifndef ROSSTATEPUBLISHER_H
#define ROSSTATEPUBLISHER_H

#include "abv_controller/Configurations.h"
#include "abv_controller/VehicleStateTracker.h"
#include "abv_idl/msg/abv_state.hpp"

class RosStatePublisher
{
public:
    RosStatePublisher(std::shared_ptr<VehicleStateTracker> aStateTracker);
    ~RosStatePublisher();

    void publishState(); 

private:
    StatePublisherConfig mConfig; 

    std::shared_ptr<VehicleStateTracker> mStateTracker; 

    abv_idl::msg::AbvState convertToIdlMsg(Eigen::Matrix<double, 6, 1> aStateVector); 
};

#endif // ROSSTATEPUBLISHER_H

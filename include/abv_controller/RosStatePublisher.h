#ifndef ROSSTATEPUBLISHER_H
#define ROSSTATEPUBLISHER_H

#include "abv_controller/Configurations.h"
#include "abv_controller/VehicleStateTracker.h"
#include "robot_idl/msg/abv_state.hpp"
#include <thread> 

class RosStatePublisher
{
public:
    RosStatePublisher(std::shared_ptr<VehicleStateTracker> aStateTracker);
    ~RosStatePublisher();

    void publishStateLoop(); 
    bool shouldPublishState() {std::lock_guard<std::mutex> lock(mShouldPublishStateMutex); return mShouldPublishState; }
    void setPublishingState(bool aFlag) {std::lock_guard<std::mutex> lock(mShouldPublishStateMutex); mShouldPublishState = aFlag; }

private:
    StatePublisherConfig mConfig; 
    std::shared_ptr<VehicleStateTracker> mStateTracker; 
    std::thread mStatePublishThread; 
    bool mShouldPublishState; 
    std::mutex mShouldPublishStateMutex; 
    std::string mTopicName; 

    robot_idl::msg::AbvState convertToIdlMsg(Eigen::Matrix<float, 13, 1> aStateVector); 
};

#endif // ROSSTATEPUBLISHER_H

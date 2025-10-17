
#include "abv_controller/RosStatePublisher.h"
#include "abv_controller/RosTopicManager.h"
#include <thread>
#include <chrono> 
#include "plog/Log.h"

RosStatePublisher::RosStatePublisher(std::shared_ptr<VehicleStateTracker> aStateTracker) : 
    mStateTracker(aStateTracker), mConfig(ConfigurationManager::getInstance()->getStatePublisherConfig()), 
    mTopicName("abv/state")
{
    RosTopicManager::getInstance()->createPublisher<robot_idl::msg::AbvState>(mTopicName);
    mStatePublishThread = std::thread(&RosStatePublisher::publishStateLoop, this); 
}

RosStatePublisher::~RosStatePublisher()
{
    // worst case this gets called here 
    setPublishingState(false); 
    
    if(mStatePublishThread.joinable())
    {
        LOGD << "Joining state publishing thread"; 
        mStatePublishThread.join(); 
    }
}

void RosStatePublisher::publishStateLoop()
{
    auto topicManager = RosTopicManager::getInstance(); 
    const std::chrono::duration<double> loop_duration(1.0 / mConfig.mRate); 

    LOGD << "Starting ROS state publishing thread";
    setPublishingState(true); 

    while(shouldPublishState())
    {
        auto loop_start = std::chrono::steady_clock::now();

        Eigen::Matrix<float, 12, 1> currentState = mStateTracker->getCurrentState(); 
        topicManager->publishMessage(mTopicName, convertToIdlMsg(currentState)); 

        // execution frequency control here s
        auto loop_end = std::chrono::steady_clock::now();
        auto elapsed = loop_end - loop_start;

        // Sleep for the remaining time to maintain the frequency
        if (elapsed < loop_duration) {
            std::this_thread::sleep_for(loop_duration - elapsed);
        } else {
            std::cerr << "Loop overrun! Elapsed time: " 
                        << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()
                        << " ms\n";
        }
    }
}

robot_idl::msg::AbvState RosStatePublisher::convertToIdlMsg(Eigen::Matrix<float, 12, 1> aStateVector)
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

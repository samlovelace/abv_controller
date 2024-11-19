
#include "abv_controller/RosStatePublisher.h"
#include "abv_controller/RosTopicManager.h"


#include <thread>
#include <chrono> 

RosStatePublisher::RosStatePublisher(std::shared_ptr<VehicleStateTracker> aStateTracker) : 
    mStateTracker(aStateTracker), mConfig(ConfigurationManager::getInstance()->getStatePublisherConfig())
{
    RosTopicManager::getInstance()->createPublisher<abv_idl::msg::AbvState>("abv_state"); 
}

RosStatePublisher::~RosStatePublisher()
{
}

void RosStatePublisher::publishState()
{
    bool done = false; 
    auto topicManager = RosTopicManager::getInstance(); 
    const std::chrono::duration<double> loop_duration(1.0 / mConfig.mRate); 

    while(!done)
    {
        auto loop_start = std::chrono::steady_clock::now();

        Eigen::Matrix<double, 6, 1> currentState = mStateTracker->getCurrentState(); 
        topicManager->publishMessage("abv_state", convertToIdlMsg(currentState)); 

        // execution frequency control here 
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

abv_idl::msg::AbvState RosStatePublisher::convertToIdlMsg(Eigen::Matrix<double, 6, 1> aStateVector)
{
    abv_idl::msg::AbvVec3 position; 
    abv_idl::msg::AbvVec3 velocity;

    position.x = aStateVector[0]; 
    position.y = aStateVector[1]; 
    position.yaw = aStateVector[2]; 

    velocity.x = aStateVector[3]; 
    velocity.y = aStateVector[4]; 
    velocity.yaw = aStateVector[5];  

    abv_idl::msg::AbvState state; 
    state.set__position(position); 
    state.set__velocity(velocity); 

    return state; 
}

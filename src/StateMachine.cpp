
#include "abv_controller/StateMachine.h"
#include <thread>
#include <iostream> 
#include "plog/Log.h"

StateMachine::StateMachine(std::shared_ptr<Vehicle> abv) : 
    mDone(false), mActiveState(States::IDLE), mVehicle(abv)
{
}

StateMachine::~StateMachine()
{
}

void StateMachine::run()
{
    // Desired frequency in Hz
    const double frequency = 10.0;
    // Calculate the loop duration
    const std::chrono::duration<double> loop_duration(1.0 / frequency);

    LOGD << "State Machine starting in " << toString(mActiveState);

    while(!isCommandedToStop())
    {
        auto loop_start = std::chrono::steady_clock::now(); 

        // State machine switch case here 
        switch (getActiveState())
        {
        case States::IDLE:
            break;

        case States::THRUSTER_CONTROL:

            if(mVehicle->isControlInputStale())
            {   
                // set control input to zeros so we stop the thrusters
                Eigen::Vector3d zeros = Eigen::Vector3d::Zero();  
                mVehicle->setControlInput(zeros); 
                mVehicle->doThrusterControl(); 

                setActiveState(States::IDLE); 
                break; 
            }

            mVehicle->doThrusterControl(); 
            break;

        case States::POSE_CONTROL: 
        
            mVehicle->doPoseControl(); 
            break;

        default:
            break;
        }

        // Calculate the time taken for the loop iteration
        auto loop_end = std::chrono::steady_clock::now();
        auto elapsed = loop_end - loop_start;

        // Sleep for the remaining time to maintain the frequency
        if (elapsed < loop_duration) {
            std::this_thread::sleep_for(loop_duration - elapsed);
        } else {
            LOGE << "Loop overrun! Elapsed time: " 
                      << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()
                      << " ms\n";
        }

    }
}

void StateMachine::setActiveState(StateMachine::States aState)
{
    LOGD << "Switching state machine from " << toString(getActiveState()).c_str() << " to " 
                                                    << toString(aState).c_str();

    std::lock_guard<std::mutex> lock(mActiveStateMutex); 
    mActiveState = aState;    
}

std::string StateMachine::toString(StateMachine::States aState)
{
    std::string stringToReturn = ""; 
    switch (aState)
    {
    case StateMachine::States::IDLE:
        stringToReturn = "IDLE"; 
        break;
    case StateMachine::States::THRUSTER_CONTROL: 
        stringToReturn = "THRUSTER_CONTROL";
        break; 
    case StateMachine::States::POSE_CONTROL: 
        stringToReturn = "POSE_CONTROL"; 
        break;
    default:
        break;
    }

    return stringToReturn; 
}
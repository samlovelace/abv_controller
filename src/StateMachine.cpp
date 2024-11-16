
#include "abv_controller/StateMachine.h"
#include <thread>
#include <iostream> 

StateMachine::StateMachine(/* args */) : mDone(false), mActiveState(States::IDLE)
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

    while(!isCommandedToStop())
    {
        auto loop_start = std::chrono::steady_clock::now(); 

        // State machine switch case here 
        switch (getActiveState())
        {
        case States::IDLE:
            break;

        case States::THRUSTER_CONTROL:
            thruster_control(); 
            break;

        case States::POSE_CONTROL: 
            pose_control(); 
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
            std::cerr << "Loop overrun! Elapsed time: " 
                      << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()
                      << " ms\n";
        }

    }
}

void StateMachine::thruster_control()
{
    printf("Doing Thruster Control\n"); 
}

void StateMachine::pose_control()
{
    printf("Doing Pose Control\n"); 
    thruster_control(); 
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
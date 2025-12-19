
#include "abv_controller/StateMachine.h"
#include "abv_controller/RateController.hpp"
#include "abv_controller/ConfigurationManager.h"
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
    auto config = ConfigurationManager::getInstance()->getStateMachineConfig(); 
    RateController rate(config.mRate); 

    LOGD << "State Machine starting in " << toString(mActiveState);

    while(!isCommandedToStop())
    {
        rate.start(); 

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

            // if here, control input not stale, apply it 
            mVehicle->doThrusterControl(); 
            break;

        case States::POSE_CONTROL: 
        
            mVehicle->doPoseControl(); 
            break;

        case States::VELOCITY_CONTROL: 

            mVehicle->doVelocityControl(); 
            break; 

        default:
            break;
        }

        rate.block(); 
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
    case StateMachine::States::VELOCITY_CONTROL: 
        stringToReturn = "VELOCITY_CONTROL"; 
        break; 
    default:
        stringToReturn = "UNKNOWN"; 
        break;
    }

    return stringToReturn; 
}
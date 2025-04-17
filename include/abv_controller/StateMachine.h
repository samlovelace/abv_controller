#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <mutex>
#include "abv_controller/Vehicle.h"
#include <memory>

class StateMachine
{
public:
    StateMachine(std::shared_ptr<Vehicle> abv);
    ~StateMachine();

    enum class States
    {
        IDLE, 
        THRUSTER_CONTROL, 
        POSE_CONTROL, 
        VELOCITY_CONTROL, 
        NUM_TYPES
    };
    std::string toString(States aState); 

    void run(); 

    /** Getters and Setters **/
    bool isCommandedToStop() {std::lock_guard<std::mutex> lock(mDoneMutex); return mDone; }
    void stop() {std::lock_guard<std::mutex> lock(mDoneMutex); mDone = true; }
    void setActiveState(States aNewActiveState); 
    States getActiveState() {std::lock_guard<std::mutex> lock(mActiveStateMutex); return mActiveState;}

private:
    bool mDone; 
    std::mutex mDoneMutex; 
    
    States mActiveState; 
    std::mutex mActiveStateMutex; 

    std::shared_ptr<Vehicle> mVehicle; 

};
#endif // STATEMACHINE_H

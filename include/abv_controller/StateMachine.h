#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <mutex>

class StateMachine
{
public:
    StateMachine(/* args */);
    ~StateMachine();

    enum class States
    {
        IDLE, 
        THRUSTER_CONTROL, 
        POSE_CONTROL, 
        NUM_TYPES
    };
    std::string toString(States aState); 

    void run(); 
    void thruster_control(); 
    void pose_control(); 

    /** Getters and Setters **/
    bool isCommandedToStop() {std::lock_guard<std::mutex> lock(mDoneMutex); return mDone; }
    void stop() {std::lock_guard<std::mutex> lock(mDoneMutex); mDone = true; }
    void setActiveState(States aNewActiveState) {std::lock_guard<std::mutex> lock(mActiveStateMutex); mActiveState = aNewActiveState;}
    States getActiveState() {std::lock_guard<std::mutex> lock(mActiveStateMutex); return mActiveState;}

private:
    bool mDone; 
    std::mutex mDoneMutex; 
    
    States mActiveState; 
    std::mutex mActiveStateMutex; 

};
#endif // STATEMACHINE_H

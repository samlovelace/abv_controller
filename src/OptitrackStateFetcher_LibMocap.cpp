
#include "abv_controller/OptitrackStateFetcher_LibMocap.h"
#include "plog/Log.h"
#include <iostream> 

OptitrackStateFetcher_LibMocap::OptitrackStateFetcher_LibMocap(const std::string& aServerIp, 
                                                               const std::string& aLocalIp, 
                                                               int aRigidBodyId, 
                                                               const std::string& aRigidBodyName) : 
    mID(aRigidBodyId), mRigidBodyName(aRigidBodyName), mServerIp(aServerIp), mLocalIp(aLocalIp)
{
    mAcquired.store(false);  
}

OptitrackStateFetcher_LibMocap::~OptitrackStateFetcher_LibMocap()
{
    if(mListenThread.joinable())
    {
        mListenThread.join(); 
    }
}

bool OptitrackStateFetcher_LibMocap::init()
{
    std::map<std::string, std::string> cfg; 
    cfg.insert({"hostname", mServerIp}); 
    cfg.insert({"interface_ip", mLocalIp}); 
    
    mMocap = std::unique_ptr<libmotioncapture::MotionCapture>(
        libmotioncapture::MotionCapture::connect("vrpn", cfg)
    );

    if(!mMocap)
    {
        LOGD << "Failed to initialize mocap client"; 
        return false; 
    }

    LOGV << "mocap client initialized successfully"; 
    mListenThread = std::thread(&OptitrackStateFetcher_LibMocap::listen, this); 
    return true; 
}


Eigen::Matrix<float, 12, 1> OptitrackStateFetcher_LibMocap::fetchState()
{
    std::lock_guard<std::mutex> lock(mStateMutex); 
    return mLatestState; 
} 

void OptitrackStateFetcher_LibMocap::setLatestState(Eigen::Matrix<float, 12, 1> aLatestState)
{
    std::lock_guard<std::mutex> lock(mStateMutex); 
    mLatestState = aLatestState; 
}

void OptitrackStateFetcher_LibMocap::listen()
{
    while(true)
    {
        // Get a frame
        mMocap->waitForNextFrame();

        if (mMocap->supportsRigidBodyTracking()) 
        {
            auto rigidBodies = mMocap->rigidBodies();

            if(rigidBodies.empty())
            {
                continue;
            }

            auto dt = std::chrono::steady_clock::now() - mPrevRecvdTime; 

            for (auto const& item : rigidBodies) 
            {
                double roll, pitch, yaw;
                const auto& rigidBody = item.second;
                
                if(rigidBody.name() == mRigidBodyName)
                {
                    Eigen::Vector3f pos = rigidBody.position(); 

                    // take the xyz
                    Eigen::Matrix<float, 12, 1> state; 
                    state[0] = pos(0); 
                    state[1] = pos(1); 
                    state[2] = pos(2); 

                    // compute linear velocity 
                    for(int i=0; i<3; i++)
                    {   
                        state[i+3] = (state[i] - mPrevState[i])/ dt.count(); 
                    }

                    Eigen::Quaternionf q = rigidBody.rotation(); 
                    q.normalize();

                    // Convert quaternion to Euler angles
                    double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();

                    // Roll (x-axis rotation)
                    double sinr_cosp = 2 * (qw * qx + qy * qz);
                    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
                    roll = std::atan2(sinr_cosp, cosr_cosp);

                    // Pitch (y-axis rotation)
                    double sinp = 2 * (qw * qy - qz * qx);
                    if (std::abs(sinp) >= 1)
                        pitch = std::copysign(M_PI / 2, sinp);
                    else
                        pitch = std::asin(sinp);

                    // Yaw (z-axis rotation)
                    double siny_cosp = 2 * (qw * qz + qx * qy);
                    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
                    yaw = std::atan2(siny_cosp, cosy_cosp);

                    state[6] = yaw;  
                    state[7] = pitch;
                    state[8] = roll;  

                    // angular velocity
                    // previous quaternion
                    Eigen::Quaternionf prev(mPrevState[6], mPrevState[7], mPrevState[8], mPrevState[9]); 
                    Eigen::Quaternionf qDelta = q * prev.inverse();
                    Eigen::Vector3f angularVelocity = 2.0 * qDelta.vec() / dt.count(); 

                    state[9] = angularVelocity.x(); 
                    state[10] = angularVelocity.y(); 
                    state[11] = angularVelocity.z(); 

                    // set latest state and update previous values
                    setLatestState(state); 
                    mPrevState = state; 
                    mPrevRecvdTime = std::chrono::steady_clock::now();  
                    if(!mAcquired.load())
                        mAcquired.store(true); 
                }
            }
        }
    }
}
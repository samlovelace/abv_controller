
#include "abv_controller/OptitrackStateFetcher_LibMocap.h"
#include "plog/Log.h"
#include <iostream> 

OptitrackStateFetcher_LibMocap::OptitrackStateFetcher_LibMocap(NetworkConfig aConfig, int aRigidBodyId, const std::string& aRigidBodyName) : 
    mConfig(aConfig), mID(aRigidBodyId), mRigidBodyName(aRigidBodyName)
{

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
    cfg.insert({"hostname", mConfig.Server.IP}); 
    cfg.insert({"interface_ip", mConfig.Local.IP}); 
    
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

            std::cout << "  rigid bodies:" << std::endl;
            auto dt = std::chrono::steady_clock::now() - mPrevRecvdTime; 

            for (auto const& item : rigidBodies) 
            {
                const auto& rigidBody = item.second;
                std::cout << "    \"" << rigidBody.name() << "\":" << std::endl;

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

                    // convert the quaternion to Euler angles
                    // Convert to rotation matrix
                    Eigen::Matrix3f rotationMatrix = q.toRotationMatrix();

                    // Extract Euler angles (roll, pitch, yaw)
                    // The arguments are the axes: 0 = X, 1 = Y, 2 = Z
                    Eigen::Vector3f euler = rotationMatrix.eulerAngles(0, 1, 2);
                    
                    state[6] = euler[2];  
                    state[7] = euler[1];
                    state[8] = euler[0];  

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
                }

                // TODO: remove after testing 
                const auto& position = rigidBody.position();
                const auto& rotation = rigidBody.rotation();
                std::cout << "       position: [" << position(0) << ", " << position(1) << ", " << position(2) << "]" << std::endl;
                std::cout << "       rotation: [" << rotation.w() << ", " << rotation.vec()(0) << ", "
                                                    << rotation.vec()(1) << ", " << rotation.vec()(2) << "]" << std::endl;
            }
        }
    }
}
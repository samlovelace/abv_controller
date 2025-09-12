
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
        libmotioncapture::MotionCapture::connect("optitrack", cfg)
    );

    if(!mMocap)
    {
        LOGD << "Could not create unique_ptr of mocap obj"; 
        return false; 
    }

    LOGD << "supportsRigidBodyTracking: " << mMocap->supportsRigidBodyTracking();
    LOGD << "supportsLatencyEstimate: " << mMocap->supportsLatencyEstimate(); 
    LOGD << "supportsPointCloud: " << mMocap->supportsPointCloud(); 
    LOGD << "supportsTimeStamp: " << mMocap->supportsTimeStamp();

    mListenThread = std::thread(&OptitrackStateFetcher_LibMocap::listen, this); 

    return true; 
}


Eigen::Matrix<double, 6, 1> OptitrackStateFetcher_LibMocap::fetchState()
{
    std::lock_guard<std::mutex> lock(mStateMutex); 
    return mLatestState; 
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
            std::cout << "  rigid bodies:" << std::endl;

            for (auto const& item : rigidBodies) 
            {
                const auto& rigidBody = item.second;
                std::cout << "    \"" << rigidBody.name() << "\":" << std::endl;

                if(rigidBody.name() == mRigidBodyName)
                {
                    Eigen::Vector3f pos = rigidBody.position(); 

                    // take the xyz
                    Eigen::Matrix<double, 6, 1> state; 
                    state[0] = (double) pos(0); 
                    state[1] = (double) pos(1); 
                    state[2] = (double) pos(2); 

                    // convert the quaternion to Euler angles
                    Eigen::Quaternionf q = rigidBody.rotation(); 
                    q.normalize(); 

                    Eigen::Matrix3f rotationMatrix = q.toRotationMatrix(); 
                    Eigen::Vector3f angles = rotationMatrix.eulerAngles(2, 1, 0); 

                    state[3] = (double)angles[0]; 
                    state[4] = (double)angles[1]; 
                    state[5] = (double)angles[2]; 

                    setLatestState(state); 
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

#include "abv_controller/OptitrackStateFetcher_LibMocap.h"
#include "plog/Log.h"
#include <iostream> 

OptitrackStateFetcher_LibMocap::OptitrackStateFetcher_LibMocap(NetworkConfig aConfig) : 
    mConfig(aConfig), mID(1)
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
        for (size_t frameId = 0;; ++frameId)
        {
            // Get a frame
            mMocap->waitForNextFrame();

            std::cout << "frame " << frameId << std::endl;
            if (mMocap->supportsTimeStamp()) 
            {
                std::cout << "  timestamp: " << mMocap->timeStamp() << " us" << std::endl;
            }

            if (mMocap->supportsLatencyEstimate()) 
            {
                std::cout << "  latency: " << std::endl;
                for (const auto& latency : mMocap->latency()) 
                {
                    std::cout << "    " << latency.name() << " " << latency.value() << " s" << std::endl;
                }
            }

            if (mMocap->supportsPointCloud()) 
            {
                std::cout << "  pointcloud:" << std::endl;
                auto pointcloud = mMocap->pointCloud();
                
                for (size_t i = 0; i < pointcloud.rows(); ++i) 
                {
                    const auto& point = pointcloud.row(i);
                    std::cout << "    \"" << i << "\": [" << point(0) << "," << point(1) << "," << point(2) << "]" << std::endl;
                }
            }

            if (mMocap->supportsRigidBodyTracking()) 
            {
                auto rigidBodies = mMocap->rigidBodies();

                std::cout << "  rigid bodies:" << std::endl;

                for (auto const& item: rigidBodies) {
                    const auto& rigidBody = item.second;

                    std::cout << "    \"" << rigidBody.name() << "\":" << std::endl;

                    const auto& position = rigidBody.position();
                    const auto& rotation = rigidBody.rotation();
                    std::cout << "       position: [" << position(0) << ", " << position(1) << ", " << position(2) << "]" << std::endl;
                    std::cout << "       rotation: [" << rotation.w() << ", " << rotation.vec()(0) << ", "
                                                        << rotation.vec()(1) << ", " << rotation.vec()(2) << "]" << std::endl;
                }
            }
        }
    }

}
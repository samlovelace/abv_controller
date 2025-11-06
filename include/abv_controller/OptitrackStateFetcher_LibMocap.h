#ifndef OPTITRACKSTATEFETCHER_LIBMOCAP_H
#define OPTITRACKSTATEFETCHER_LIBMOCAP_H
 
#include "IStateFetcher.h"
#include <memory>
#include <thread> 

#include "ConfigurationManager.h"
#include "Configurations.h" 

#include "libmotioncapture/motioncapture.h"

class OptitrackStateFetcher_LibMocap : public IStateFetcher
{ 
public:
    OptitrackStateFetcher_LibMocap(const std::string& aServerIp, 
                                   const std::string& aLocalIp, 
                                   int aRigidBodyId, 
                                   const std::string& aRigidBodyName);

    ~OptitrackStateFetcher_LibMocap() override; 

    bool init() override;
    Eigen::Matrix<float, 12, 1> fetchState() override;  

private:

    std::unique_ptr<libmotioncapture::MotionCapture> mMocap; 
    std::string mServerIp; 
    std::string mLocalIp; 
    Eigen::Matrix<float, 12, 1> mLatestState; 
    Eigen::Matrix<float, 12,1> mPrevState; 
    int mID; 
    std::string mRigidBodyName; 
    std::mutex mStateMutex; 
    std::chrono::steady_clock::time_point mPrevRecvdTime; 
    std::thread mListenThread; 
    
private: 

    void listen(); 
    void setLatestState(Eigen::Matrix<float, 12, 1> aLatestState);
   
};
#endif //OPTITRACKSTATEFETCHER_LIBMOCAP_H
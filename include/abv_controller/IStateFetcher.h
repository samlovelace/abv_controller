#ifndef ISTATEFETCHER_H
#define ISTATEFETCHER_H

#include <eigen3/Eigen/Dense>
#include <mutex>

class IStateFetcher
{
public:
    virtual ~IStateFetcher() = default; 
    virtual Eigen::Matrix<float, 13, 1> fetchState() = 0; 
    virtual bool init() = 0; 

protected: 
    Eigen::Matrix<float, 13, 1> mState; 

};
#endif // ISTATEFETCHER_H
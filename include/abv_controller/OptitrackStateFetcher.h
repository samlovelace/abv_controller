#ifndef OPTITRACKSTATEFETCHER_H
#define OPTITRACKSTATEFETCHER_H

#include "IStateFetcher.h"

class OptitrackStateFetcher : public IStateFetcher
{
public:
    OptitrackStateFetcher(/* args */);
    ~OptitrackStateFetcher() override; 

    Eigen::Matrix<double, 6, 1> fetchState() override; 

private:
    /* data */
};
#endif //OPTITRACKSTATEFETCHER_H
    
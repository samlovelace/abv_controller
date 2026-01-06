#ifndef DUMMYTHRUSTERDRIVER_H
#define DUMMYTHRUSTERDRIVER_H
 
#include "IThrusterDriver.hpp"
#include <vector>
 
class DummyThrusterDriver : public IThrusterDriver
{ 
public:
    DummyThrusterDriver(std::vector<int> aSet);
    ~DummyThrusterDriver() override; 

    bool init() override; 
    bool fini() override; 

    bool send(const std::string& aThrustCommand) override; 

private:
   
};
#endif //DUMMYTHRUSTERDRIVER_H
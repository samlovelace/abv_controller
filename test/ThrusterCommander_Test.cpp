
#include <gtest/gtest.h> 
#include "abv_controller/ThrusterCommander.h"
 
class ThrusterCommander_Test : public ::testing::Test, public ThrusterCommander
{ 
public:
    ThrusterCommander_Test() {
        mConfig.arduino.IP = "127.0.0.1"; 
        mConfig.arduino.CmdPort = 1000; 
        mConfig.arduino.DataPort = 1001; 
        mConfig.uOn = 0.2; 
        mConfig.uOff = 0.9; 
    }
    ~ThrusterCommander_Test() = default; 

protected:  
   
};


TEST_F(ThrusterCommander_Test, ConvertToThrustVector_Test)
{
    Eigen::Vector3d input = {2.5, 0, 0}; 
    Eigen::Vector3i thrustVector = convertToThrustVector(input); 
    Eigen::Vector3i correct = {1, 0, 0}; 
    ASSERT_EQ(correct, thrustVector); 

    input = {0, 2.5, 0}; 
    thrustVector = convertToThrustVector(input); 
    correct = {0, 1, 0}; 
    ASSERT_EQ(correct, thrustVector); 

    input = {0, 0, 2.5}; 
    thrustVector = convertToThrustVector(input); 
    correct = {0, 0, 1}; 
    ASSERT_EQ(correct, thrustVector); 
}




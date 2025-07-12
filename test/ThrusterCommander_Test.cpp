
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
    std::vector<Eigen::Vector3d> inputs = {
        {2.5, 0, 0}, 
        {0, 2.5, 0}, 
        {0, 0, 2.5}, 
        {0, 0, -2.5},
        {0, -2.5, 0},
        {-2.5, 0, 0} 
    }; 
    
    std::vector<Eigen::Vector3i> outputs = {
        {1, 0, 0}, 
        {0, 1, 0}, 
        {0, 0, 1}, 
        {0, 0, -1},
        {0, -1, 0},
        {-1, 0, 0} 
    };

    for (int i = 0; i < inputs.size(); i++)
    { 
        auto output = convertToThrustVector(inputs[i]); 

        ASSERT_EQ(output,outputs[i]); 
    }

}




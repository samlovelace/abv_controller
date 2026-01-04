
#include "abv_controller/RosTopicManager.h"

RosTopicManager::RosTopicManager(/* args */) : Node("ABV_Controller")
{
}

RosTopicManager::~RosTopicManager()
{
    rclcpp::shutdown();
}

void RosTopicManager::spinNode()
{
    std::thread([this]() {
        rclcpp::spin(this->get_node_base_interface());
    }).detach();
}
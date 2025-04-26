cmake_minimum_required(VERSION 3.8)
project(abv_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# use ORIGIN so executable looks for .so files in same folder
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "$ORIGIN")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(abv_idl)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_library(NATNET_LIB NatNet PATHS ${CMAKE_CURRENT_LIST_DIR}/dependencies/NatNetSDK4.1/lib)


set(ABV_CONTROLLER_SOURCES 
    ${CMAKE_CURRENT_LIST_DIR}/src/CommandHandler.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/ConfigurationManager.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/Controller.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp 
    ${CMAKE_CURRENT_LIST_DIR}/src/OptitrackStateFetcher.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/RosStatePublisher.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/RosTopicManager.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/SimulatedStateFetcher.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/StateMachine.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/ThrusterCommander.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/UdpClient.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/Vehicle.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/VehicleStateTracker.cpp
)

add_executable(${PROJECT_NAME} ${ABV_CONTROLLER_SOURCES})

include_directories(${PROJECT_NAME} 
  ${CMAKE_CURRENT_LIST_DIR}/include 
  ${CMAKE_CURRENT_LIST_DIR}/include/plog/plog-master/include
  ${CMAKE_CURRENT_LIST_DIR}/dependencies/NatNetSDK4.1/include
  /usr/include

)

ament_target_dependencies(${PROJECT_NAME} rclcpp abv_idl Eigen3)
target_link_libraries(${PROJECT_NAME} yaml-cpp ${NATNET_LIB}) 

# Install the executable
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY configuration/ DESTINATION share/${PROJECT_NAME}/configuration)
install(FILES dependencies/NatNetSDK4.1/lib/libNatNet.so DESTINATION lib/${PROJECT_NAME})

ament_package()
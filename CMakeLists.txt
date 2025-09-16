cmake_minimum_required(VERSION 3.8)
project(abv_controller)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(test)

# Shared library lookup behavior
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "$ORIGIN")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_idl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

option(ARCH_TEST "Force-disable NatNetSDK regardless of architecture" OFF)

# Architecture detection
set(USE_NATNET FALSE)
if(NOT ARCH_TEST AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(USE_NATNET TRUE)
  add_definitions(-DUSE_NATNET)
  message(STATUS "Architecture is x86_64, NatNet SDK will be included.")
else()
  message(STATUS "Using libmotioncapture for OptiTrack support")

  set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build static libmotioncapture")
  add_subdirectory(dependencies/libmotioncapture)
  set(USE_LIBMOCAP TRUE)
  add_definitions(-DUSE_LIBMOCAP)
endif()

# Source files (common)
set(ABV_CONTROLLER_SOURCES 
  src/CommandHandler.cpp
  src/ConfigurationManager.cpp
  src/Controller.cpp
  src/main.cpp 
  src/RosStatePublisher.cpp
  src/RosTopicManager.cpp
  src/SimulatedStateFetcher.cpp
  src/StateMachine.cpp
  src/ThrusterCommander.cpp
  src/UdpClient.cpp
  src/Vehicle.cpp
  src/VehicleStateTracker.cpp
)

# Conditionally include NatNet-dependent source
if(USE_NATNET)
  list(APPEND ABV_CONTROLLER_SOURCES src/OptitrackStateFetcher_NatNet.cpp)
endif()
if(USE_LIBMOCAP)
    list(APPEND ABV_CONTROLLER_SOURCES src/OptitrackStateFetcher_LibMocap.cpp)
endif()

# Create executable
add_executable(${PROJECT_NAME} ${ABV_CONTROLLER_SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${CMAKE_CURRENT_LIST_DIR}/include/plog/plog-master/include
  /usr/include
  ${EIGEN3_INCLUDE_DIRS}
)

if(USE_NATNET)
  target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/dependencies/NatNetSDK4.1/include
  )
endif()

if(USE_LIBMOCAP)
    target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/dependencies/libmotioncapture/include
    )
endif()

# Dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  robot_idl
  Eigen3
)

# Link libraries
target_link_libraries(${PROJECT_NAME} yaml-cpp)

if(USE_NATNET)
  find_library(NATNET_LIB NatNet PATHS ${CMAKE_CURRENT_LIST_DIR}/dependencies/NatNetSDK4.1/lib)
  if(NATNET_LIB)
    target_link_libraries(${PROJECT_NAME} ${NATNET_LIB})
    install(FILES dependencies/NatNetSDK4.1/lib/libNatNet.so DESTINATION lib/${PROJECT_NAME})
  else()
    message(FATAL_ERROR "NatNet library not found but USE_NATNET is TRUE.")
  endif()
endif()

if(USE_LIBMOCAP)
    target_link_libraries(${PROJECT_NAME} libmotioncapture)
endif()

# Install
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY configuration/ DESTINATION share/${PROJECT_NAME}/configuration)

install(
  PROGRAMS run.sh
  DESTINATION bin
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(abv_controller)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(ARCH_TEST "Force-disable NatNet regardless of architecture" OFF)

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

# Architecture detection
set(ARCH_X86 FALSE)
set(ARCH_ARM FALSE)

if(NOT ARCH_TEST AND CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  
  set(ARCH_X86 TRUE)
  add_definitions(-DARCH_X86)
  message(STATUS "Architecture is x86_64, NatNet SDK will be included.")

#elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
else()   
  message(STATUS "Using libmotioncapture for OptiTrack support")

  set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build static libmotioncapture")
  add_subdirectory(dependencies/libmotioncapture)
  set(ARCH_ARM TRUE)
  add_definitions(-DARCH_ARM)

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

# Conditionally include architecture dependent sources
if(ARCH_X86)
  list(APPEND ABV_CONTROLLER_SOURCES 
  	src/OptitrackStateFetcher_NatNet.cpp
	src/DummyThrusterDriver.cpp)
elseif(ARCH_ARM)
	list(APPEND ABV_CONTROLLER_SOURCES 
		src/OptitrackStateFetcher_LibMocap.cpp
		src/GpioThrusterDriver.cpp)
endif()

# Create executable
add_executable(${PROJECT_NAME} ${ABV_CONTROLLER_SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${CMAKE_CURRENT_LIST_DIR}/include/plog/plog-master/include
  ${EIGEN3_INCLUDE_DIRS}
)

if(ARCH_X86)
	target_include_directories(${PROJECT_NAME} PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/dependencies/NatNetSDK4.1/include)
elseif(ARCH_ARM)
	target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/dependencies/libmotioncapture/include
		${CMAKE_CURRENT_LIST_DIR}/dependencies/JETGPIO-1.2)
endif()

# Dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  robot_idl
  Eigen3
)

# Link libraries
target_link_libraries(${PROJECT_NAME} yaml-cpp)

if(ARCH_X86)

	find_library(NATNET_LIB NatNet PATHS ${CMAKE_CURRENT_LIST_DIR}/dependencies/NatNetSDK4.1/lib)
	
	if(NATNET_LIB)
		target_link_libraries(${PROJECT_NAME} ${NATNET_LIB})
		install(FILES dependencies/NatNetSDK4.1/lib/libNatNet.so DESTINATION lib/${PROJECT_NAME})
	else()
		message(FATAL_ERROR "NatNet library not found but USE_NATNET is TRUE.")
endif()

elseif(ARCH_ARM)
	# manually set because it doesnt use cmake 
	set(jetgpio_LIB ${CMAKE_CURRENT_LIST_DIR}/dependencies/JETGPIO-1.2/libjetgpio.so)
    target_link_libraries(${PROJECT_NAME} libmotioncapture ${jetgpio_LIB})
	install(FILES ${CMAKE_CURRENT_LIST_DIR}/dependencies/JETGPIO-1.2/libjetgpio.so DESTINATION lib/${PROJECT_NAME})
endif()

# Install
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY configuration/ DESTINATION share/${PROJECT_NAME}/configuration)

install(
  PROGRAMS run.sh
  DESTINATION bin
)

ament_package()

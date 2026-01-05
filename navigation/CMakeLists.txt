cmake_minimum_required(VERSION 3.8)
project(abv_navigation)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_idl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build static libmotioncapture")
add_subdirectory(
  ${CMAKE_CURRENT_LIST_DIR}/../dependencies/libmotioncapture libmotioncapture
)

set(SOURCES
    src/main.cpp
)

include(${CMAKE_CURRENT_LIST_DIR}/../common/CMakeLists.txt)

add_executable(${PROJECT_NAME} ${SOURCES} ${COMMON_SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE 
    include
    ${COMMON_INCLUDE_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../dependencies/plog/plog-master/include
)

# Dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  robot_idl
  Eigen3
)

target_link_libraries(${PROJECT_NAME} yaml-cpp)
# Install
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
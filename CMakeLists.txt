cmake_minimum_required(VERSION 3.8)
project(abv_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(abv_idl)

set(ABV_CONTROLLER_SOURCES 
  ${CMAKE_CURRENT_LIST_DIR}/src/StateMachine.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/CommandHandler.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/RosTopicManager.cpp 
  ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp 
)

add_executable(${PROJECT_NAME} ${ABV_CONTROLLER_SOURCES})

include_directories(${PROJECT_NAME} include)

ament_target_dependencies(${PROJECT_NAME} rclcpp abv_idl)

# Install the executable
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

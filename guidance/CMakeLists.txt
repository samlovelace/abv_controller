cmake_minimum_required(VERSION 3.8)
project(abv_guidance)

find_package(ament_cmake REQUIRED)

set(SOURCES
    main.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
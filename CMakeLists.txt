# requires minimum cmake version
cmake_minimum_required(VERSION 3.5)

# produce the cmake var PROJECT_NAME
project(assignment_grasp-it)

# mandatory use of these packages
find_package(YARP 3.5.1 REQUIRED)
find_package(ICUBcontrib REQUIRED)
find_package(Gazebo REQUIRED)

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

# helpers defining certain macros
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

add_subdirectory(gazebo)

# world
add_library(${PROJECT_NAME}-world SHARED ${CMAKE_SOURCE_DIR}/src/world.cpp)
target_compile_definitions(${PROJECT_NAME}-world PRIVATE _USE_MATH_DEFINES)
target_include_directories(${PROJECT_NAME}-world PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}-world PRIVATE ${GAZEBO_LIBRARIES} ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME}-world LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib)

# assignment
include_directories(${CMAKE_SOURCE_DIR}/src)
add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/main.cpp
                               ${CMAKE_SOURCE_DIR}/src/helpers.h
                               ${CMAKE_SOURCE_DIR}/src/helpers.cpp)
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# generate ad-hoc project to perform "make uninstall"
icubcontrib_add_uninstall_target()

file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})

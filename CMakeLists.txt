cmake_minimum_required(VERSION 3.10)
project(StateEstimatorTest)

# Add include directories
include_directories(
                    )

# Add include directories
link_directories("/usr/local/lib")
INCLUDE_DIRECTORIES("/usr/local/include")
link_directories("/usr/lib")
INCLUDE_DIRECTORIES("/usr/include")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/StateEstimator/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/eigen3
                    ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/rbdl/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/ruckig/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/rbdl/lib
                    ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/rbdl/include/addons/urdfreader
                    ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/nlohmann
                    ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty
                    ${Qt5Core_INCLUDE_DIRS})

# Add executable target
add_executable(test main.cpp)


# Link necessary libraries
target_link_libraries(test ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/rbdl/lib/librbdl.so)
target_link_libraries(test ${CMAKE_CURRENT_SOURCE_DIR}/StateEstimator/src/libStateEstimator.so)
target_link_libraries(test ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/rbdl/lib/librbdl_urdfreader.so)


# Set C++ standard
set_property(TARGET test PROPERTY CXX_STANDARD 11)

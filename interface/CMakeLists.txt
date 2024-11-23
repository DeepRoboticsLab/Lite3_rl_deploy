cmake_minimum_required(VERSION 3.10)

# project(interface)

set(CMAKE_BUILD_TYPE Release)
add_definitions(-w) # warning ignore
add_compile_options(-fPIC)
set(CMAKE_CXX_STANDARD 17)

include_directories(
    ./robot
    ./user_command
)
# file(GLOB_RECURSE SIMULATOIN_SRC "robot/simulation/*.hpp")
file(GLOB_RECURSE USER_COMMAND_SRC "user_command/*.cpp")

set(GAMEPAD_SDK_PATH "${THIRD_PARTY}/gamepad")
include_directories(${GAMEPAD_SDK_PATH}/include)
file(GLOB_RECURSE GAMEPAD_SDK_SRC "${GAMEPAD_SDK_PATH}/src/*.cpp")

set(LITE3_MOTION_SDK_PATH "${THIRD_PARTY}/Lite3_MotionSDK")
include_directories(${LITE3_MOTION_SDK_PATH}/include)
file(GLOB_RECURSE LITE3_MOTION_SDK_SRC "${LITE3_MOTION_SDK_PATH}/src/*.cpp")

add_library(interface STATIC ${USER_COMMAND_SRC} ${GAMEPAD_SDK_SRC} ${LITE3_MOTION_SDK_SRC} ${ECAN_SDK_SRC})
target_link_libraries(interface PUBLIC -lpthread -lm -lrt -ldl -lstdc++)

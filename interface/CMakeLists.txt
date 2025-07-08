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

# ==== Gamepad SDK ====
file(GLOB_RECURSE USER_COMMAND_SRC "user_command/*.cpp")

set(GAMEPAD_SDK_PATH "${THIRD_PARTY}/gamepad")
include_directories(${GAMEPAD_SDK_PATH}/include)
file(GLOB_RECURSE GAMEPAD_SDK_SRC "${GAMEPAD_SDK_PATH}/src/*.cpp")


# ==== Motion SDK ====
set(LITE3_MOTION_SDK_PATH "${THIRD_PARTY}/Lite3_MotionSDK")
include_directories(${LITE3_MOTION_SDK_PATH}/include)
file(GLOB_RECURSE LITE3_MOTION_SDK_SRC "${LITE3_MOTION_SDK_PATH}/src/*.cpp")

# ==== Mujoco Interface ====
if (USE_MJCPP)
  message(STATUS "✅ USE_MJCPP is ON. Including MujocoInterface headers.")
  set(MUJOCO_ROOT "${THIRD_PARTY}/mujoco/${BUILD_PLATFORM}")
  include_directories(${MUJOCO_ROOT}/include)
  link_directories(${MUJOCO_ROOT}/lib)
endif()



# ==== BUILD STATIC LIBRARY ====
add_library(interface STATIC ${USER_COMMAND_SRC} ${GAMEPAD_SDK_SRC} ${LITE3_MOTION_SDK_SRC})
target_link_libraries(interface PUBLIC -lpthread -lm -lrt -ldl -lstdc++)

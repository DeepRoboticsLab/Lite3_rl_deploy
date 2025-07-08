cmake_minimum_required(VERSION 3.10)

project(rl_controller)

set(CMAKE_BUILD_TYPE Release)
add_definitions(-w) # warning ignore
add_compile_options(-fPIC)

# ========== 控制选项 ==========
set(BUILD_PLATFORM "x86" CACHE STRING "select build cpu type")
option(HIMLOCO ON)
option(BUILD SIM OFF)
option(USE_RAISIM OFF)
set(USE_PYBULLET ON)
set(USE_RAISIM OFF)
option(SEND_REMOTE OFF)
# option(USE_ONNX "Use ONNXRuntime" ON)
option(USE_MJCPP "Use Mujoco C++" OFF)



if (BUILD_PLATFORM STREQUAL arm)
  message("this is arm platform")
  set(BUILD_SIM OFF)
  set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
  set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++") 
else()
  message("this is x86 platform")
endif()
set(CMAKE_CXX_STANDARD 17)

if (SEND_REMOTE)
  set(BUILD_SIM OFF)
endif()

# ========== 显示参数 ==========
message("BUILD_PLATFORM: ${BUILD_PLATFORM}")
message("BUILD_SIM:      ${BUILD_SIM}")
message("SEND_REMOTE:    ${SEND_REMOTE}")
# message("USE_ONNX:       ${USE_ONNX}")
message("USE_MJCPP:      ${USE_MJCPP}")

# 路径设置
get_filename_component(WORKSPACE_DIR ./ ABSOLUTE)
set(THIRD_PARTY ${WORKSPACE_DIR}/third_party)

# torch setting
# set(Torch_DIR "${THIRD_PARTY}/libtorch/${BUILD_PLATFORM}/share/cmake/Torch")
# find_package(Torch REQUIRED)
# message("Torch_DIR:  ${Torch_DIR}")

# ONNX
set(ONNX_RUNTIME_ROOT "${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}")
message("Using ONNXRuntime from: ${ONNX_RUNTIME_ROOT}")


# ONNX setting
# if (USE_ONNX)
#   add_definitions(-DUSE_ONNX)

#   include_directories(${ONNX_RUNTIME_ROOT}/include)
#   link_directories(${ONNX_RUNTIME_ROOT}/lib)
# endif()
include_directories(${ONNX_RUNTIME_ROOT}/include)
link_directories(${ONNX_RUNTIME_ROOT}/lib)


# MuJoCo
set(MUJOCO_ROOT "${THIRD_PARTY}/mujoco/${BUILD_PLATFORM}")
message("USE_MJCPP enabled, loading MuJoCo headers from ${MUJOCO_ROOT}")
if (USE_MJCPP)
  add_definitions(-DUSE_MJCPP)
  include_directories(${MUJOCO_ROOT}/include)
  link_directories(${MUJOCO_ROOT}/lib)
endif()

if (HIMLOCO)
  message("HIMLOCO enabled, adding HIMLOCO definitions")
  add_definitions(-DHIMLOCO)
endif()




# ========== Include 头文件路径 ==========
include_directories(
  ${THIRD_PARTY}/eigen/
  ${THIRD_PARTY}/gamepad/include
  ${THIRD_PARTY}/Lite3_MotionSDK/include/common
  ${THIRD_PARTY}/Lite3_MotionSDK/include
  types
  utils
  interface/robot
  interface/user_command
  state_machine
  run_policy
  # ${TORCH_INCLUDE_DIRS}
  ${ONNX_RUNTIME_ROOT}/include
  ${MUJOCO_ROOT}/include
)

# ========== 添加 interface 子模块 ==========
add_subdirectory(interface)

# ========== 仿真器宏定义 ==========
if (BUILD_SIM)
message("USE_PYBULLET:    ${USE_PYBULLET}")

  if(USE_RAISIM)
    add_definitions(-DUSE_RAISIM)
    set(raisim_DIR "your path to raisim")
    find_package(raisim CONFIG REQUIRED)
  elseif(USE_PYBULLET)
    add_definitions(-DUSE_PYBULLET)
  elseif(USE_MJCPP)
    set(MUJOCO_ROOT "${THIRD_PARTY}/mujoco/${BUILD_PLATFORM}")
    message("USE_MJCPP enabled, loading MuJoCo headers from ${MUJOCO_ROOT}")
    add_definitions(-DUSE_MJCPP)
    include_directories(${MUJOCO_ROOT}/include)
    link_directories(${MUJOCO_ROOT}/lib)
  else()
    message(FATAL_ERROR "No simulation option selected. Please select one of USE_RAISIM, USE_PYBULLET, or USE_MJCPP.")
  endif()
endif()

# ========== 设置部署路径 ==========
# 如果涉及到
if (SEND_REMOTE)
  set(PATH_LIB "../lib")
  set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) 
  set(CMAKE_INSTALL_RPATH "${PATH_LIB}/onnxruntime/${BUILD_PLATFORM}/lib")
  # set(CMAKE_INSTALL_RPATH ${PATH_LIB}/libtorch/${BUILD_PLATFORM}/lib)
endif()

# ========== 收集源文件 ==========
file(GLOB_RECURSE STATE_MACHINE_SRC "state_machine/*.c*")
file(GLOB_RECURSE RUN_POLICY_SRC "run_policy/*.cpp")

add_executable(rl_deploy main.cpp ${STATE_MACHINE_SRC} ${RUN_POLICY_SRC})

# ========== 链接基本库 ==========
target_link_libraries(rl_deploy interface -lpthread -lm -lrt -ldl -lstdc++ onnxruntime)
# if (USE_ONNX)
#   target_link_libraries(rl_deploy onnxruntime)
#   # target_link_libraries(rl_deploy torch)
# endif()

# MuJoCo

if (BUILD_SIM)
  if (USE_MJCPP)
    message("build simulation mujoco")
    target_link_libraries(rl_deploy mujoco glfw)
  endif()
endif()

if (BUILD_SIM)
  link_directories(${THIRD_PARTY}/Lite3_MotionSDK/lib)
  target_link_libraries(rl_deploy
      ${PROJECT_SOURCE_DIR}/third_party/Lite3_MotionSDK/lib/libdeeprobotics_legged_sdk_x86_64.so
      pthread
      rt
  )
else()
  target_link_libraries(rl_deploy
      ${PROJECT_SOURCE_DIR}/third_party/Lite3_MotionSDK/lib/libdeeprobotics_legged_sdk_aarch64.so
      pthread
      rt
  )
endif()

if (BUILD_SIM)
    target_compile_definitions(rl_deploy PRIVATE BUILD_SIMULATION)
endif()

# SDK动态链接库
# link_directories(${THIRD_PARTY}/Lite3_MotionSDK/lib)
# target_link_libraries(rl_deploy
#     ${PROJECT_SOURCE_DIR}/third_party/Lite3_MotionSDK/lib/libdeeprobotics_legged_sdk_x86_64.so
#     pthread
#     rt
# )

# for real
# target_link_libraries(rl_deploy
#     ${PROJECT_SOURCE_DIR}/third_party/Lite3_MotionSDK/lib/libdeeprobotics_legged_sdk_aarch64.so
#     pthread
#     rt
# )




# Raisim
if (BUILD_SIM)
  if(USE_RAISIM)
    message("build simulation raisim")
    target_link_libraries(rl_deploy raisim::raisim)
  endif()
endif()

if (SEND_REMOTE)
  add_custom_target(push_to_robot ALL DEPENDS rl_deploy)
  add_custom_command(
    TARGET push_to_robot POST_BUILD
    COMMAND ./scripts/sftp_to_remote.sh
    WORKING_DIRECTORY ..
    COMMENT "run sftp_to_remote.sh"
  )
endif()

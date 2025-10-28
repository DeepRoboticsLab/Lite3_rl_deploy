cmake_minimum_required(VERSION 3.10)
project(Lite3API)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Threads REQUIRED)

# Optional: For simulation (PyBullet integration)
option(BUILD_SIM "Build with simulation support" OFF)
if(BUILD_SIM)
    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
endif()

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/../types                   
    ${CMAKE_SOURCE_DIR}/../interface               
    ${CMAKE_SOURCE_DIR}/../interface/robot         
    ${CMAKE_SOURCE_DIR}/../interface/robot/simulation  
    ${CMAKE_SOURCE_DIR}/../interface/robot/hardware    
    ${CMAKE_SOURCE_DIR}/../interface/user_command      
    ${CMAKE_SOURCE_DIR}/../run_policy                 
)

# Core library: lite3_controller
add_library(lite3_controller STATIC
    src/lite3_controller.cpp
)

target_link_libraries(lite3_controller
    Eigen3::Eigen
    Threads::Threads
)

# Example executable
add_executable(simple_example
    examples/simple_example.cpp
)

target_link_libraries(simple_example
    lite3_controller
)

# Test executable
add_executable(test_controller
    src/tests/test_controller.cpp
)

target_link_libraries(test_controller
    lite3_controller
)

# Python bindings module
pybind11_add_module(pylite3
    python/python_bindings.cpp
)

target_link_libraries(pylite3 PRIVATE
    lite3_controller
    Eigen3::Eigen
)

# If BUILD_SIM=ON, add Python3 linking for simulation
if(BUILD_SIM)
    target_include_directories(lite3_controller PRIVATE ${Python3_INCLUDE_DIRS})
    target_link_libraries(lite3_controller ${Python3_LIBRARIES})
    target_link_libraries(simple_example ${Python3_LIBRARIES})
    target_link_libraries(test_controller ${Python3_LIBRARIES})
    add_definitions(-DBUILD_SIMULATION)  
endif()

# ONNX Runtime for RL_CONTROL
find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime PATHS /usr/lib /usr/local/lib)
if(ONNXRUNTIME_LIBRARY)
    target_link_libraries(lite3_controller ${ONNXRUNTIME_LIBRARY})
    target_link_libraries(simple_example ${ONNXRUNTIME_LIBRARY})
    target_link_libraries(test_controller ${ONNXRUNTIME_LIBRARY})
    target_link_libraries(pylite3 PRIVATE ${ONNXRUNTIME_LIBRARY})
else()
    message(WARNING "ONNX Runtime not found. RL_CONTROL may not work.")
endif()
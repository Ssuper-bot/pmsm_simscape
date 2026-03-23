cmake_minimum_required(VERSION 3.16)
project(pmsm_foc VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_PYTHON_BINDINGS "Build pybind11 Python bindings" OFF)

# ==== Core Library ====
add_library(pmsm_core STATIC
    src/transforms.cpp
    src/pid_controller.cpp
    src/svpwm.cpp
    src/foc_controller.cpp
)
target_include_directories(pmsm_core PUBLIC include)

# ==== Unit Tests ====
if(BUILD_TESTS)
    add_executable(test_foc tests/test_foc_controller.cpp)
    target_link_libraries(test_foc PRIVATE pmsm_core)
    
    enable_testing()
    add_test(NAME foc_tests COMMAND test_foc)
endif()

# ==== Python Bindings (pybind11) ====
if(BUILD_PYTHON_BINDINGS)
    find_package(pybind11 QUIET)
    if(pybind11_FOUND)
        pybind11_add_module(_pmsm_cpp src/pybind_module.cpp)
        target_link_libraries(_pmsm_cpp PRIVATE pmsm_core)
    else()
        message(WARNING "pybind11 not found. Python bindings will not be built.")
    endif()
endif()

# ==== ROS2 Integration (optional) ====
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    
    # ROS2 FOC controller node (C++ version)
    # add_executable(foc_node_cpp src/ros2/foc_node.cpp)
    # ament_target_dependencies(foc_node_cpp rclcpp std_msgs)
    # target_link_libraries(foc_node_cpp pmsm_core)
    
    ament_package()
endif()

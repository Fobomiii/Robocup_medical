# Standalone geometry tests can also run without a ROS installation.
cmake_minimum_required(VERSION 3.8)
project(medical_execution_path_tests LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()
add_executable(test_execution_path_selection test_execution_path_selection.cpp)
target_include_directories(test_execution_path_selection PRIVATE ../include)
add_test(NAME execution_path_selection COMMAND test_execution_path_selection)

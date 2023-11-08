cmake_minimum_required(VERSION 3.26)
project(Kalman_Test)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PythonLibs 3 REQUIRED)

# Make sure this points to the directory containing the NumPy headers
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories("C:/Python311/Lib/site-packages/numpy/core/include")

add_executable(Kalman_Test main.cpp)
target_link_libraries(Kalman_Test Eigen3::Eigen ${PYTHON_LIBRARIES})

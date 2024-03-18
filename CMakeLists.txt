cmake_minimum_required(VERSION 3.0)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Include OpenCV directories
include_directories(${OpenCV_INCLUDE_DIRS})

# Create an executable and link OpenCV and OpenMP if available
add_executable(pid_control main.cpp)
target_link_libraries(pid_control ${OpenCV_LIBS})

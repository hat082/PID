cmake_minimum_required(VERSION 3.0)
project(pid_control)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Include OpenCV directories
include_directories(${OpenCV_INCLUDE_DIRS})

# Find WiringPi library
find_library(WIRINGPI_LIB wiringPi)
if(WIRINGPI_LIB)
  message(STATUS "Found WiringPi library: ${WIRINGPI_LIB}")
else()
  message(FATAL_ERROR "WiringPi library not found")
endif()

# Add your source files
add_executable(pid_control main.cpp)

# Link OpenCV and WiringPi libraries
target_link_libraries(pid_control ${OpenCV_LIBS} ${WIRINGPI_LIB})


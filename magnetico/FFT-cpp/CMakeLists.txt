cmake_minimum_required(VERSION 3.10)
project(FFT-cpp VERSION 1.0.0)

# Set the source and include directories
set(SOURCES_DIR ${CMAKE_CURRENT_LIST_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/include)

# Collect source files
file(GLOB SOURCES ${SOURCES_DIR}/*.cpp)

# Create the executable
add_library(FFT-cpp ${SOURCES})

# Include directories
target_include_directories(FFT-cpp PUBLIC ${INCLUDE_DIR})

cmake_minimum_required(VERSION 3.3)

set(CMAKE_SYSROOT /Users/victor.lopez1/Projects/rpi-compilers/armv8-rpi3-linux-gnueabihf/sysroot)
set(TOOLCHAIN_PREFIX armv8-rpi3-linux-gnueabihf)
# Include directories


# Linker flags
set(CMAKE_EXE_LINKER_FLAGS_INIT "${CMAKE_EXE_LINKER_FLAGS_INIT} -L${CMAKE_SYSROOT}/lib/armv8-rpi3-linux-gnueabihf")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${CMAKE_EXE_LINKER_FLAGS_INIT} -L${CMAKE_SYSROOT}/usr/lib/armv8-rpi3-linux-gnueabihf")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${CMAKE_EXE_LINKER_FLAGS_INIT} -L${CMAKE_SYSROOT}/usr/lib/gcc/armv8-rpi3-linux-gnueabihf/12")

set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

set(CMAKE_SYSTEM_VERSION=1)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)
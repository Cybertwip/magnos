set(CMAKE_SYSTEM_NAME               Haiku)
set(CMAKE_SYSTEM_PROCESSOR          arm)

# Without that flag CMake is not able to pass test compilation check
set(CMAKE_TRY_COMPILE_TARGET_TYPE   STATIC_LIBRARY)
set(TOOLCHAIN_TRIPLET arm-unknown-linux)
set(TOOLCHAIN_PREFIX "${TOOLCHAIN_TRIPLET}-")

set(CMAKE_AR                        ${BAREMETAL_ARM_TOOLCHAIN_PATH}/arm-linux-gnueabihf-ar${CMAKE_EXECUTABLE_SUFFIX})
set(CMAKE_ASM_COMPILER              ${BAREMETAL_ARM_TOOLCHAIN_PATH}/arm-linux-gnueabihf-gcc${CMAKE_EXECUTABLE_SUFFIX})
set(CMAKE_C_COMPILER                ${BAREMETAL_ARM_TOOLCHAIN_PATH}/arm-linux-gnueabihf-gcc${CMAKE_EXECUTABLE_SUFFIX})
set(CMAKE_CXX_COMPILER              ${BAREMETAL_ARM_TOOLCHAIN_PATH}/arm-linux-gnueabihf-g++${CMAKE_EXECUTABLE_SUFFIX})
set(CMAKE_LINKER                    ${BAREMETAL_ARM_TOOLCHAIN_PATH}/arm-linux-gnueabihf-ld${CMAKE_EXECUTABLE_SUFFIX})
set(CMAKE_OBJCOPY                   ${BAREMETAL_ARM_TOOLCHAIN_PATH}/arm-linux-gnueabihf-objcopy${CMAKE_EXECUTABLE_SUFFIX} CACHE INTERNAL "")
set(CMAKE_RANLIB                    ${BAREMETAL_ARM_TOOLCHAIN_PATH}/arm-linux-gnueabihf-ranlib${CMAKE_EXECUTABLE_SUFFIX} CACHE INTERNAL "")
set(CMAKE_SIZE                      ${BAREMETAL_ARM_TOOLCHAIN_PATH}/arm-linux-gnueabihf-size${CMAKE_EXECUTABLE_SUFFIX} CACHE INTERNAL "")
set(CMAKE_STRIP                     ${BAREMETAL_ARM_TOOLCHAIN_PATH}/arm-linux-gnueabihf-strip${CMAKE_EXECUTABLE_SUFFIX} CACHE INTERNAL "")

set(CMAKE_C_FLAGS                   "-DOS_RASPBIAN=1 -Wno-unused-function -Wno-unused-variable -Wno-implicit-function-declaration -Wno-parentheses -Wno-maybe-uninitialized -Wno-unused-variable -Wl,--gc-sections" CACHE INTERNAL "")
set(CMAKE_CXX_FLAGS                 "${CMAKE_C_FLAGS}" CACHE INTERNAL "")

set(CMAKE_C_FLAGS_DEBUG             "-Os -g" CACHE INTERNAL "")
set(CMAKE_C_FLAGS_RELEASE           "-Os -DNDEBUG" CACHE INTERNAL "")
set(CMAKE_CXX_FLAGS_DEBUG           "${CMAKE_C_FLAGS_DEBUG}" CACHE INTERNAL "")
set(CMAKE_CXX_FLAGS_RELEASE         "${CMAKE_C_FLAGS_RELEASE}" CACHE INTERNAL "")


set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)

set(CMAKE_SYSROOT "${BAREMETAL_ARM_TOOLCHAIN_PATH}/../arm-unknown-linux-gnueabihf/sysroot")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I${BAREMETAL_ARM_TOOLCHAIN_PATH}/../include")
set(CMAKE_CXX_FLAGS "-fpermissive ${CMAKE_CXX_FLAGS} -I${BAREMETAL_ARM_TOOLCHAIN_PATH}/../include")

set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

set(RASPBERRY_PI True CACHE STRING "RASPBERRY_PI")
set(OS_RASPBIAN True CACHE STRING "OS_RASPBIAN")
cmake_minimum_required (VERSION 3.5)

project(whisper.cpp VERSION 1.5.2)

# Add path to modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(WHISPER_STANDALONE ON)
    include(GitVars)
    include(BuildTypes)

    # configure project version
    if (EXISTS "${CMAKE_SOURCE_DIR}/bindings/ios/Makefile-tmpl")
        configure_file(${CMAKE_SOURCE_DIR}/bindings/ios/Makefile-tmpl ${CMAKE_SOURCE_DIR}/bindings/ios/Makefile @ONLY)
    endif()
    configure_file(${CMAKE_SOURCE_DIR}/bindings/javascript/package-tmpl.json ${CMAKE_SOURCE_DIR}/bindings/javascript/package.json @ONLY)
else()
    set(WHISPER_STANDALONE OFF)
endif()

if (EMSCRIPTEN)
    set(BUILD_SHARED_LIBS_DEFAULT OFF)

    option(WHISPER_WASM_SINGLE_FILE "whisper: embed WASM inside the generated whisper.js" ON)
endif()

# options

option(WHISPER_ALL_WARNINGS           "whisper: enable all compiler warnings"                   ON)
option(WHISPER_ALL_WARNINGS_3RD_PARTY "whisper: enable all compiler warnings in 3rd party libs" OFF)

option(WHISPER_SANITIZE_THREAD        "whisper: enable thread sanitizer"    OFF)
option(WHISPER_SANITIZE_ADDRESS       "whisper: enable address sanitizer"   OFF)
option(WHISPER_SANITIZE_UNDEFINED     "whisper: enable undefined sanitizer" OFF)

option(WHISPER_SDL2                   "whisper: support for libSDL2" OFF)

option(WHISPER_NO_AVX                 "whisper: disable AVX"  OFF)
option(WHISPER_NO_AVX2                "whisper: disable AVX2" OFF)
option(WHISPER_NO_FMA                 "whisper: disable FMA"  OFF)
option(WHISPER_NO_F16C                "whisper: disable F16c" OFF)

option(WHISPER_PERF "whisper: enable perf timings" OFF)

# sanitizers

if (NOT MSVC)
    if (WHISPER_SANITIZE_THREAD)
        set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -fsanitize=thread")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread")
    endif()

    if (WHISPER_SANITIZE_ADDRESS)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}     -fsanitize=address -fno-omit-frame-pointer")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
    endif()

    if (WHISPER_SANITIZE_UNDEFINED)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}     -fsanitize=undefined")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined")
    endif()
endif()

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ffast-math")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")

# dependencies
find_package(Threads REQUIRED)

# compiler flags

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo")
endif ()

if (WHISPER_ALL_WARNINGS)
    if (NOT MSVC)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} \
            -Wall                           \
            -Wextra                         \
            -Wpedantic                      \
            -Wshadow                        \
            -Wcast-qual                     \
            -Wstrict-prototypes             \
            -Wpointer-arith                 \
            -Wno-unused-function            \
        ")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
            -Wall                           \
            -Wextra                         \
            -Wpedantic                      \
            -Wcast-qual                     \
        ")
    else()
        # todo : msvc
    endif()
endif()

if (NOT MSVC)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Werror=vla")
    #set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-math-errno -ffinite-math-only -funsafe-math-optimizations")
endif()

message(STATUS "CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")

if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    message(STATUS "ARM detected")
elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "ppc64le")
    message(STATUS "PowerPC detected")
else()
    message(STATUS "x86 detected")
    if (MSVC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /utf-8")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /utf-8")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /utf-8")
        if(NOT WHISPER_NO_AVX2)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX2")
            set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /arch:AVX2")
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /arch:AVX2")
        else()
            if(NOT WHISPER_NO_AVX)
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX")
                set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /arch:AVX")
                set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /arch:AVX")
            endif()
        endif()
    else()
        if (EMSCRIPTEN)
            set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -pthread")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
        else()
            if(NOT WHISPER_NO_AVX)
                set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mavx")
            endif()
            if(NOT WHISPER_NO_AVX2)
                set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mavx2")
            endif()
            if(NOT WHISPER_NO_FMA)
                set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfma")
            endif()
            if(NOT WHISPER_NO_F16C)
                set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mf16c")
            endif()
        endif()
    endif()
endif()

#
# POSIX conformance
#

# clock_gettime came in POSIX.1b (1993)
# CLOCK_MONOTONIC came in POSIX.1-2001 / SUSv3 as optional
# posix_memalign came in POSIX.1-2001 / SUSv3
# M_PI is an XSI extension since POSIX.1-2001 / SUSv3, came in XPG1 (1985)
add_compile_definitions(_XOPEN_SOURCE=600)

# Somehow in OpenBSD whenever POSIX conformance is specified
# some string functions rely on locale_t availability,
# which was introduced in POSIX.1-2008, forcing us to go higher
if (CMAKE_SYSTEM_NAME MATCHES "OpenBSD")
    remove_definitions(-D_XOPEN_SOURCE=600)
    add_compile_definitions(_XOPEN_SOURCE=700)
endif()

# Data types, macros and functions related to controlling CPU affinity
# are available on Linux through GNU extensions in libc
if (CMAKE_SYSTEM_NAME MATCHES "Linux")
    add_compile_definitions(_GNU_SOURCE)
endif()

# RLIMIT_MEMLOCK came in BSD, is not specified in POSIX.1,
# and on macOS its availability depends on enabling Darwin extensions
# similarly on DragonFly, enabling BSD extensions is necessary
if (CMAKE_SYSTEM_NAME MATCHES "Darwin")
    add_compile_definitions(_DARWIN_C_SOURCE)
endif()
if (CMAKE_SYSTEM_NAME MATCHES "DragonFly")
    add_compile_definitions(_DARWIN_C_SOURCE)
endif()

# alloca is a non-standard interface that is not visible on BSDs when
# POSIX conformance is specified, but not all of them provide a clean way
# to enable it in such cases
if (CMAKE_SYSTEM_NAME MATCHES "FreeBSD")
    add_compile_definitions(__BSD_VISIBLE)
endif()
if (CMAKE_SYSTEM_NAME MATCHES "NetBSD")
    add_compile_definitions(_NETBSD_SOURCE)
endif()
if (CMAKE_SYSTEM_NAME MATCHES "OpenBSD")
    add_compile_definitions(_BSD_SOURCE)
endif()

#
# whisper - this is the main library of the project
#

set(TARGET whisper)

add_library(${TARGET}
    whisper.h
    whisper.cpp
    )

include(DefaultTargetOptions)

target_include_directories(${TARGET} PUBLIC
    .
    )

if (MSVC)
    target_link_libraries(${TARGET} PRIVATE ${WHISPER_EXTRA_LIBS} ${CMAKE_THREAD_LIBS_INIT})

    set(WHISPER_EXTRA_FLAGS ${WHISPER_EXTRA_FLAGS} -D_CRT_SECURE_NO_WARNINGS)
else()
    target_link_libraries(${TARGET} PRIVATE m ${WHISPER_EXTRA_LIBS} ${CMAKE_THREAD_LIBS_INIT})
endif()

if (EMSCRIPTEN)
    set_target_properties(${TARGET} PROPERTIES COMPILE_FLAGS "-msimd128")
endif()

target_compile_definitions(${TARGET} PUBLIC
    ${WHISPER_EXTRA_FLAGS}
    )

set_target_properties(${TARGET} PROPERTIES PUBLIC_HEADER "whisper.h")

include(GNUInstallDirs)
#/****************************************************************************
# Copyright (c) 2013-2014 cocos2d-x.org
# Copyright (c) 2021-2022 Bytedance Inc.
#
# https://axmolengine.github.io/
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# ****************************************************************************/

cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS OFF)

if(APPLE)
    set(CMAKE_OSX_ARCHITECTURES arm64)
    
    set(OpenMP_CXX_INCLUDE_DIRS "/opt/homebrew/opt/libomp/include")
    set(OpenMP_CXX_LIBRARIES "/opt/homebrew/opt/libomp/lib/libomp.a")

    set(protobuf_DIR " /opt/homebrew/Cellar/protobuf@3/3.20.3")

    LIST(APPEND CMAKE_PREFIX_PATH "${protobuf_DIR}") 

endif()

set(USE_OPENMP OFF CACHE STRING "USE_OPENMP")

set(AIRLIB_CXX_STD ${CMAKE_CXX_STANDARD} CACHE STRING "AIRLIB_CXX_STD")
set(AIRSIM_ROOT ${CMAKE_CURRENT_LIST_DIR}/AirSim CACHE STRING "AIRSIM_ROOT")
set(AX_ENABLE_EXT_LUA NO CACHE INTERNAL "AX_ENABLE_EXT_LUA")


set(APP_NAME magnetico)

project(${APP_NAME})

set(MAGNOS_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR} CACHE STRING "MAGNOS_RUNTIME_OUTPUT_DIRECTORY")

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/AirSim/cmake/cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/AirSim/cmake/cmake-modules/CommonSetup.cmake")
CommonSetup()

if(EMSCRIPTEN)
    set(TARGET RISCV64_GENERIC CACHE INTERNAL "OPENBLAS_TARGET")
    set(ONLY_CBLAS NO CACHE INTERNAL "ONLY_CBLAS")
    set(NOFORTRAN 1 CACHE STRING "NOFORTRAN")
endif()

set(BUILD_STATIC_LIBS ON CACHE INTERNAL "BUILD_STATIC_LIBS")
set(BUILD_SHARED_LIBS OFF CACHE INTERNAL "BUILD_SHARED_LIBS")
set(WITH_WERROR NO CACHE INTERNAL "WITH_WERROR")

add_subdirectory(AirSim/cmake/AirLib)
add_subdirectory(AirSim/cmake/MavLinkCom)
add_subdirectory(AirSim/cmake/rpclib_wrapper)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_BINARY_DIR}/CMake")

find_package(Chrono COMPONENTS Vehicle)

set(_AX_ROOT "$ENV{AX_ROOT}" CACHE INTERNAL "_AX_ROOT")
set(AX_ROOT "$ENV{AX_ROOT}" CACHE INTERNAL "AX_ROOT")


if(NOT DEFINED BUILD_ENGINE_DONE) # to test magnetico into root project
    if(XCODE)
        set(CMAKE_XCODE_GENERATE_TOP_LEVEL_PROJECT_ONLY TRUE)
    endif()

    # config quick starter batch script run.bat for windows
    if(WIN32)
        file(RELATIVE_PATH CMAKE_BUILD_RELATIVE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
        configure_file("${CMAKE_CURRENT_SOURCE_DIR}/run.bat.in" "${CMAKE_CURRENT_SOURCE_DIR}/run.bat" @ONLY)
    endif()


    if(NOT (_AX_ROOT STREQUAL ""))
        file(TO_CMAKE_PATH ${_AX_ROOT} _AX_ROOT)
        message(STATUS "Using system env var _AX_ROOT=${_AX_ROOT}")
    else()
        message(FATAL_ERROR "Please run setup.ps1 add system env var 'AX_ROOT' to specific the engine root")
    endif()

    set(CMAKE_MODULE_PATH ${_AX_ROOT}/cmake/Modules/)

    include(AXBuildSet)
    set(_AX_USE_PREBUILT FALSE)
    if ((WIN32 OR LINUX) AND DEFINED AX_PREBUILT_DIR AND IS_DIRECTORY ${_AX_ROOT}/${AX_PREBUILT_DIR})
        set(_AX_USE_PREBUILT TRUE)
    endif()

    if (NOT _AX_USE_PREBUILT)
        add_subdirectory(${_AX_ROOT}/core ${ENGINE_BINARY_PATH}/axmol/core)
    endif()
endif()

# The common cross-platforms source files and header files
file(GLOB_RECURSE GAME_HEADER
    Source/*.h
    )
file(GLOB_RECURSE GAME_SOURCE
    Source/*.cpp
    )
    
set(GAME_INC_DIRS
  "${CMAKE_CURRENT_SOURCE_DIR}/Source"
)

set(content_folder
    "${CMAKE_CURRENT_SOURCE_DIR}/Content"
    )

if(APPLE)
    ax_mark_multi_resources(common_content_files RES_TO "Resources" FOLDERS ${content_folder})
elseif(WINDOWS)
    ax_mark_multi_resources(common_content_files RES_TO "Content" FOLDERS ${content_folder})
endif()

if(ANDROID)
    # the APP_NAME should match on AndroidManifest.xml
    list(APPEND GAME_SOURCE
         proj.android/app/jni/main.cpp
         )
elseif(LINUX)
    list(APPEND GAME_SOURCE
         proj.linux/main.cpp
         )
    list(APPEND GAME_SOURCE ${common_content_files})
elseif(WASM)
    list(APPEND GAME_SOURCE
         proj.wasm/main.cpp
         )
    list(APPEND GAME_SOURCE ${common_content_files})
elseif(WINDOWS)
    if(NOT WINRT)
        list(APPEND GAME_HEADER
            proj.win32/main.h
            proj.win32/resource.h
            )
        list(APPEND GAME_SOURCE
            proj.win32/main.cpp
            proj.win32/game.rc
            )
    else()
        ax_setup_winrt_sources()
    endif()
    list(APPEND GAME_SOURCE ${common_content_files})
elseif(APPLE)
    if(IOS)
        list(APPEND GAME_HEADER
             proj.ios_mac/ios/AppController.h
             proj.ios_mac/ios/RootViewController.h
             )

        if (TVOS)
            set(APP_UI_RES
                proj.ios_mac/ios/LaunchScreenBackground.png
                proj.ios_mac/ios/targets/tvos/LaunchScreen.storyboard
                proj.ios_mac/ios/targets/tvos/Images.xcassets
                )
        else()
            set(APP_UI_RES
                proj.ios_mac/ios/LaunchScreenBackground.png
                proj.ios_mac/ios/targets/ios/LaunchScreen.storyboard
                proj.ios_mac/ios/targets/ios/Images.xcassets
                )
        endif()

        list(APPEND GAME_SOURCE
             proj.ios_mac/ios/main.m
             proj.ios_mac/ios/AppController.mm
             proj.ios_mac/ios/RootViewController.mm
             proj.ios_mac/ios/Prefix.pch
             ${APP_UI_RES}
             )
    elseif(MACOSX)
        set(APP_UI_RES
            proj.ios_mac/mac/Icon.icns
            proj.ios_mac/mac/Info.plist
            )
        list(APPEND GAME_SOURCE
             proj.ios_mac/mac/main.cpp
             proj.ios_mac/mac/Prefix.pch
             ${APP_UI_RES}
             )
    endif()
    list(APPEND GAME_SOURCE ${common_content_files})
endif()

# mark app complie info and libs info
set(APP_SOURCES
    ${GAME_HEADER}
    ${GAME_SOURCE}
    )
if(NOT ANDROID)
    add_executable(${APP_NAME} ${APP_SOURCES})
else()
    add_library(${APP_NAME} SHARED ${APP_SOURCES})
    # whole archive for jni
    target_link_libraries(${APP_NAME} -Wl,--whole-archive cpp_android_spec -Wl,--no-whole-archive)
    config_android_shared_libs("org.axmol.lib" "${CMAKE_CURRENT_SOURCE_DIR}/proj.android/app/src")
endif()

if (NOT _AX_USE_PREBUILT)
    target_link_libraries(${APP_NAME} ${_AX_CORE_LIB})
endif()

CommonTargetLink()

target_link_libraries(${APP_NAME} AirLib)
target_link_libraries(${APP_NAME} MavLinkCom)
target_link_libraries(${APP_NAME} ${RPC_LIB})
target_link_libraries(${APP_NAME} ${CHRONO_LIBRARIES})  

if(APPLE)
    target_link_libraries(${APP_NAME} ${OpenMP_CXX_LIBRARIES})  
endif()

target_compile_definitions(${APP_NAME} PRIVATE "CHRONO_DATA_DIR=\"chrono/data/\"") 

if(EMSCRIPTEN)
    if("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
        target_compile_options(${APP_NAME} PRIVATE "-O2")
        target_compile_options(AirLib PRIVATE "-O2")
    else()
        target_compile_options(${APP_NAME} PRIVATE "-fsanitize=address")
        target_compile_options(AirLib PRIVATE "-fsanitize=address")
        target_compile_options(${APP_NAME} PRIVATE "-g3")
        target_compile_options(AirLib PRIVATE "-g3")
        target_compile_options(${APP_NAME} PRIVATE "-gseparate-dwarf")
        target_compile_options(AirLib PRIVATE "-gseparate-dwarf")

        target_link_options(${APP_NAME} PRIVATE "-fsanitize=address")
        target_link_options(${APP_NAME} PRIVATE "-fsanitize=address")
    endif()
    target_link_options(${APP_NAME} PRIVATE "-s ALLOW_MEMORY_GROWTH=1")
    target_link_options(${APP_NAME} PRIVATE "-s INITIAL_MEMORY=4gb")
endif()

#set(_geant4_lib_use_suffix -static)
#set(Geant4_LIBRARIES
# Geant4::G4Tree${_geant4_lib_use_suffix}
#   Geant4::G4FR${_geant4_lib_use_suffix}
#   Geant4::G4GMocren${_geant4_lib_use_suffix}
#   Geant4::G4visHepRep${_geant4_lib_use_suffix}
#   Geant4::G4RayTracer${_geant4_lib_use_suffix}
#   Geant4::G4VRML${_geant4_lib_use_suffix}
#   Geant4::G4ToolsSG${_geant4_lib_use_suffix})

# target_link_libraries(${APP_NAME} ${Geant4_LIBRARIES})



# The optional thirdparties(not dependent by engine)
if (AX_WITH_YAML_CPP)
    list(APPEND GAME_INC_DIRS "${_AX_ROOT}/thirdparty/yaml-cpp/include")
endif()


if(APPLE)
    target_include_directories(${APP_NAME} PRIVATE ${OpenMP_CXX_INCLUDE_DIRS})
endif()

target_include_directories(${APP_NAME} PRIVATE ${GAME_INC_DIRS})
target_include_directories(${APP_NAME} PRIVATE ${CHRONO_INCLUDE_DIRS})

# mark app resources, resource will be copy auto after mark
ax_setup_app_config(${APP_NAME})
if(APPLE)
    set_target_properties(${APP_NAME} PROPERTIES RESOURCE "${APP_UI_RES}")
    set_xcode_property(${APP_NAME} INSTALL_PATH "\$(LOCAL_APPS_DIR)")
    set_xcode_property(${APP_NAME} PRODUCT_BUNDLE_IDENTIFIER "org.axmol.hellocpp")

    if(MACOSX)
        set_target_properties(${APP_NAME} PROPERTIES MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/proj.ios_mac/mac/Info.plist")
    elseif(TVOS)
        set_target_properties(${APP_NAME} PROPERTIES MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/proj.ios_mac/ios/targets/tvos/Info.plist")
        set_xcode_property(${APP_NAME} ASSETCATALOG_COMPILER_APPICON_NAME "Brand Assets")
    elseif(IOS)
        set_target_properties(${APP_NAME} PROPERTIES MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/proj.ios_mac/ios/targets/ios/Info.plist")
        set_xcode_property(${APP_NAME} ASSETCATALOG_COMPILER_APPICON_NAME "AppIcon")
    endif()

    # For code-signing, set the DEVELOPMENT_TEAM:
    #set_xcode_property(${APP_NAME} DEVELOPMENT_TEAM "GRLXXXX2K9")
elseif(WINDOWS)
    if((NOT _AX_USE_PREBUILT) AND (NOT WINRT))
        ax_copy_target_dll(${APP_NAME})
    endif()
endif()

if ((NOT APPLE) AND (NOT WINRT))
    ax_get_resource_path(APP_RES_DIR ${APP_NAME})
    ax_sync_target_res(${APP_NAME} LINK_TO ${APP_RES_DIR} FOLDERS ${content_folder} SYM_LINK 1)

    if((WINDOWS AND (NOT (CMAKE_GENERATOR MATCHES "Ninja"))))
        set_property(TARGET ${APP_NAME} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${content_folder}")
        if(NOT DEFINED BUILD_ENGINE_DONE)
            set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT ${APP_NAME})
        endif()
    endif()
elseif(WINRT)
    if(NOT DEFINED BUILD_ENGINE_DONE)
        set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT ${APP_NAME})
    endif()
endif()

if (_AX_USE_PREBUILT) # support windows and linux
    use_ax_compile_define(${APP_NAME})

    include(AXLinkHelpers)
    ax_link_cxx_prebuilt(${APP_NAME} ${_AX_ROOT} ${AX_PREBUILT_DIR})
endif()

if (NOT DEFINED BUILD_ENGINE_DONE)
    ax_uwp_set_all_targets_deploy_min_version()
endif()


ax_setup_app_props(${APP_NAME})


#Translator


# Define the "translator" executable
add_executable(translator
    translator/src/main.cpp
    translator/src/layer_details.hpp
    translator/src/mlpack_to_torch.hpp
    translator/src/model_parser_impl.hpp
    translator/src/model_parser.hpp
    translator/src/onnx_pb.h
    translator/src/onnx_to_mlpack.hpp
)

# Set C++ standard and other compiler options if necessary
set_target_properties(translator PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

# Add any include directories if required

find_package(Boost REQUIRED)

if(Boost_FOUND)
    message(STATUS "Boost found: ${Boost_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "Boost not found. Please install Boost or provide the Boost path.")
endif()


find_package(onnxruntime REQUIRED)

if(onnxruntime_FOUND)
    message(STATUS "onnxruntime found")
else()
    message(FATAL_ERROR "onnxruntime not found. Please install onnxruntime or provide the onnxruntime path.")
endif()

# Find required protobuf package
find_package(protobuf REQUIRED)

# Check if protobuf was found
if(TARGET protobuf::libprotobuf)
    message(STATUS "protobuf found: ${PROTOBUF_LIBRARIES}")
else()
    message(FATAL_ERROR "Protobuf not found. Please install it or specify its location.")
endif()


target_include_directories(translator PRIVATE
   translator/src
   ${Boost_INCLUDE_DIRS}
   ${_AX_ROOT}/thirdparty/stb
)

add_subdirectory(onnx)

target_compile_definitions(onnx PRIVATE NDEBUG=1)
target_compile_definitions(onnx_proto PRIVATE NDEBUG=1)

target_compile_definitions(translator PRIVATE MLPACK_HAS_STB=1 MLPACK_ENABLE_ANN_SERIALIZATION=1 MODELS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/translator/models/")

target_link_libraries(translator PRIVATE AirLib ${Boost_LIBRARIES} protobuf::libprotobuf onnxruntime::onnxruntime onnx ${_AX_CORE_LIB})


# Extract include directories from the imported target.
get_target_property(ONNXRUNTIME_INCLUDE_DIRS onnxruntime::onnxruntime INTERFACE_INCLUDE_DIRECTORIES)

# Now ONNXRUNTIME_INCLUDE_DIRS contains the include directories.
# You can print or use them as needed.
message("ONNX Runtime Include Directories: ${ONNXRUNTIME_INCLUDE_DIRS}")

# Define the "translator" executable
add_executable(detector
translator/test/main.cpp
translator/test/MLUtils.cpp
)

target_include_directories(detector PRIVATE
   translator/src
   translator/test
   ${Boost_INCLUDE_DIRS}
   ${_AX_ROOT}/thirdparty/stb
   ${ONNXRUNTIME_INCLUDE_DIRS}
)

# Set C++ standard and other compiler options if necessary
set_target_properties(detector PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)


target_compile_definitions(detector PRIVATE MLPACK_HAS_STB=1 MLPACK_ENABLE_ANN_SERIALIZATION=1 MODELS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/translator/models/")

target_link_libraries(detector PRIVATE AirLib ${Boost_LIBRARIES} protobuf::libprotobuf onnxruntime::onnxruntime onnx ${_AX_CORE_LIB})


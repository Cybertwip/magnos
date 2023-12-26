cmake_minimum_required(VERSION 3.5.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS OFF)

cmake_policy(SET CMP0079 NEW)

set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)

set(PROJECT_NAME AirLib)

project(${PROJECT_NAME})

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

IncludeEigen()

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../AirLib/thirdparty/mlpack mlpack)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../AirLib/thirdparty/OpenBLAS OpenBLAS)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../AirLib/thirdparty/armadillo armadillo)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../AirLib/thirdparty/ensmallen ensmallen)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../AirLib/thirdparty/cereal cereal)

file(GLOB_RECURSE ${PROJECT_NAME}_sources 
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/api/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/common/common_utils/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/components/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/safety/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/vehicles/car/api/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/vehicles/multirotor/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/vehicles/multirotor/api/*.cpp
)


add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_sources})

CommonTargetLink()


target_include_directories(${PROJECT_NAME} PUBLIC 
${AIRSIM_ROOT}/AirLib/
${AIRSIM_ROOT}/AirLib/include
${AIRSIM_ROOT}/AirLib/thirdparty/eigen3
${AIRSIM_ROOT}/AirLib/thirdparty/mlpack/src
${AIRSIM_ROOT}/MavLinkCom/include
${RPC_LIB_INCLUDES}
)


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${MAGNOS_RUNTIME_OUTPUT_DIRECTORY}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${MAGNOS_RUNTIME_OUTPUT_DIRECTORY}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${MAGNOS_RUNTIME_OUTPUT_DIRECTORY}/bin)

# Find Armadillo and link it.
target_link_libraries(ensmallen INTERFACE cereal)
target_link_libraries(ensmallen INTERFACE armadillo)

target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
target_link_libraries(${PROJECT_NAME} MavLinkCom)
target_link_libraries(${PROJECT_NAME} ensmallen)
target_link_libraries(${PROJECT_NAME} openblas_static)

target_compile_definitions(${PROJECT_NAME} PRIVATE "ARMA_DONT_USE_STD_MUTEX=1")

if(MSVC)
  target_compile_options(${PROJECT_NAME} PRIVATE "/Zc:__cplusplus")
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

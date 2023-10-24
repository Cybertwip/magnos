cmake_minimum_required(VERSION 3.5.0)

cmake_policy(SET CMP0079 NEW)


project(AirLib)

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
${AIRSIM_ROOT}/MavLinkCom/include
${RPC_LIB_INCLUDES}
)


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${MAGNOS_RUNTIME_OUTPUT_DIRECTORY}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${MAGNOS_RUNTIME_OUTPUT_DIRECTORY}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${MAGNOS_RUNTIME_OUTPUT_DIRECTORY}/bin)

# Find Armadillo and link it.
target_link_libraries(ensmallen INTERFACE armadillo)

target_link_libraries(${PROJECT_NAME} PUBLIC ${RPC_LIB})
target_link_libraries(${PROJECT_NAME} PUBLIC MavLinkCom)
target_link_libraries(${PROJECT_NAME} PUBLIC ensmallen)
target_link_libraries(${PROJECT_NAME} PUBLIC cereal)
target_link_libraries(${PROJECT_NAME} PUBLIC openblas_static)



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

target_link_libraries(${PROJECT_NAME} PUBLIC ${RPC_LIB})
target_link_libraries(${PROJECT_NAME} PUBLIC MavLinkCom)

target_include_directories(${PROJECT_NAME} PUBLIC 
${AIRSIM_ROOT}/AirLib/
${AIRSIM_ROOT}/AirLib/include
${AIRSIM_ROOT}/MavLinkCom/include
${RPC_LIB_INCLUDES}
${ARMADILLO_INCLUDE_DIRS} 
${MLPACK_INCLUDE_DIRS}
)

target_link_libraries(armadillo PUBLIC openblas_static)
target_link_libraries(${PROJECT_NAME} PUBLIC ensmallen)
target_link_libraries(${PROJECT_NAME} PUBLIC cereal)

#string(SUBSTRING ${CMAKE_STATIC_LINKER_FLAGS} 9 -1 "BUILD_PLATFORM")
#find_package(Threads REQUIRED)
#target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
#target_link_libraries(${PROJECT_NAME} AirLib)
#target_link_libraries(${PROJECT_NAME} MavLinkCom)
#target_link_libraries(${PROJECT_NAME} ../AirLib/deps/rpclib/lib/${BUILD_PLATFORM}/$(Configuration)/rpc)

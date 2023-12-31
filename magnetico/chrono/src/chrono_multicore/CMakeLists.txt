#===============================================================================
# CMake configuration file for the ChronoEngine_Multicore library
#
# Invoked from the main CMakeLists.txt using ADD_SUBDIRECTORY()
#===============================================================================

# The Chrono::Multicore module will build even if OpenMP is not found,
# it just won't have parallel computation support.
# If TBB is defined, thrust will use TBB but the rest of the code will be single
# threaded

option(ENABLE_MODULE_MULTICORE "Enable the Chrono Multicore module" OFF)

# Return now if this module is not enabled

if(NOT ENABLE_MODULE_MULTICORE)
  mark_as_advanced(FORCE BLAZE_INSTALL_DIR)
  mark_as_advanced(FORCE USE_MULTICORE_DOUBLE)
  mark_as_advanced(FORCE USE_MULTICORE_SIMD)
  mark_as_advanced(FORCE USE_MULTICORE_CUDA)
  return()
endif()

message(STATUS "\n==== Chrono Multicore module ====\n")

# Return now if Thrust not available
if (NOT THRUST_FOUND)
  message("Chrono::Multicore requires Thrust, but Thrust was not found; disabling Chrono::Multicore")
  set(ENABLE_MODULE_MULTICORE OFF CACHE BOOL "Enable the Chrono Multicore module" FORCE)
  return()
endif()

mark_as_advanced(CLEAR BLAZE_INSTALL_DIR)
mark_as_advanced(CLEAR USE_MULTICORE_DOUBLE)
mark_as_advanced(CLEAR USE_MULTICORE_SIMD)
mark_as_advanced(CLEAR USE_MULTICORE_CUDA)

# ------------------------------------------------------------------------------
# Additional compiler flags
# ------------------------------------------------------------------------------

set(CH_MULTICORE_CXX_FLAGS "")
set(CH_MULTICORE_C_FLAGS "")

# ------------------------------------------------------------------------------
# Additional dependencies, specific to this module
# ------------------------------------------------------------------------------

# ----- CUDA support -----

cmake_dependent_option(USE_MULTICORE_CUDA "Enable CUDA support in Chrono::Multicore (if available)" OFF "CUDA_FOUND" OFF)

if(USE_MULTICORE_CUDA)
  set(CHRONO_MULTICORE_USE_CUDA "#define CHRONO_MULTICORE_USE_CUDA")
else()
  set(CHRONO_MULTICORE_USE_CUDA "#undef CHRONO_MULTICORE_USE_CUDA")
endif()

# ----- Double precision support -----

OPTION(USE_MULTICORE_DOUBLE "Compile Chrono::Multicore with double precision math" ON)

IF(USE_MULTICORE_DOUBLE)
  SET(CHRONO_MULTICORE_USE_DOUBLE "#define CHRONO_MULTICORE_USE_DOUBLE")
ENDIF()

# ----- Blaze library -----

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  SET(BLAZE_INSTALL_DIR ""  CACHE PATH   "Where is Blaze located?")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  FIND_PATH(BLAZE_INSTALL_DIR NAMES blaze/Blaze.h PATHS "/usr/include" "/usr/local/include")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  FIND_PATH(BLAZE_INSTALL_DIR NAMES blaze/Blaze.h PATHS "/usr/include" "/usr/local/include")
else()
  find_path(BLAZE_INSTALL_DIR NAMES blaze/Blaze.h PATHS "/usr/include" "/usr/local/include")
ENDIF()

# Extract Blaze version
find_file(BLAZE_VERSION_FILENAME "Version.h" PATHS "${BLAZE_INSTALL_DIR}/blaze/system")
mark_as_advanced(FORCE BLAZE_VERSION_FILENAME)
if(BLAZE_VERSION_FILENAME)
  file(READ ${BLAZE_VERSION_FILENAME} BLAZE_VERSION_FILE)
  message(STATUS "Blaze version file: ${BLAZE_VERSION_FILENAME}")
  string(REGEX MATCH "#define BLAZE_MAJOR_VERSION ([0-9]*)" _BLAZE_MAJOR_VERSION ${BLAZE_VERSION_FILE})
  set(BLAZE_MAJOR_VERSION ${CMAKE_MATCH_1})
  string(REGEX MATCH "#define BLAZE_MINOR_VERSION ([0-9]*)" _BLAZE_MINOR_VERSION ${BLAZE_VERSION_FILE})
  set(BLAZE_MINOR_VERSION ${CMAKE_MATCH_1})
  set(BLAZE_VERSION "${BLAZE_MAJOR_VERSION}.${BLAZE_MINOR_VERSION}")
  message(STATUS "Blaze version: ${BLAZE_VERSION}")
else()
  message(FATAL_ERROR "Cannot find blaze/system/Version.h.  Set BLAZE_INSTALL_DIR.")
endif()

# ----- Configure Blaze and Thrust -----

IF(ENABLE_OPENMP)
  add_definitions(-DTHRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_OMP)
  add_definitions(-DTHRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_OMP)
ELSEIF(ENABLE_TBB)
  add_definitions(-DBLAZE_USE_CPP_THREADS)
  add_definitions(-DTHRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_TBB)
  add_definitions(-DTHRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_TBB)
ELSE()
  add_definitions(-DBLAZE_USE_CPP_THREADS)
  add_definitions(-DTHRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_CPP)
  add_definitions(-DTHRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP)
ENDIF()

# ----- BOOST -- required only for older versions of Blaze -----

if (BLAZE_VERSION VERSION_LESS "3.2")
  set(BOOST_REQUIRED "TRUE")
else()
  set(BOOST_REQUIRED "FALSE")
endif()

if(BOOST_REQUIRED)
  set(BOOST_ROOT "" CACHE PATH "Where is Boost located?")
  find_package(Boost REQUIRED)

  if (Boost_FOUND)
    message(STATUS "Boost include dir: ${Boost_INCLUDE_DIRS}")
  else()
    mark_as_advanced(CLEAR BOOST_ROOT)
    message(FATAL_ERROR "Boost required for Blaze version ${BLAZE_VERSION}. Specify BOOST_ROOT or use Blaze 3.2 or newer.")
  endif()

endif()

# ----------------------------------------------------------------------------
# Generate and install configuration header file.
# ----------------------------------------------------------------------------

# Generate the configuration header file using substitution variables.
# Place the header file in the library output directory and make sure it can
# be found at compile time.

CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/ChConfigMulticore.h.in
  ${PROJECT_BINARY_DIR}/chrono_multicore/ChConfigMulticore.h
  )

install(FILES "${PROJECT_BINARY_DIR}/chrono_multicore/ChConfigMulticore.h"
        DESTINATION include/chrono_multicore)

# ------------------------------------------------------------------------------
# Collect all additional include directories necessary for the MULTICORE module
# ------------------------------------------------------------------------------

set(CH_MULTICORE_INCLUDES
    ${BLAZE_INSTALL_DIR}
    ${THRUST_INCLUDE_DIR}
)

if(BOOST_REQUIRED)
  set(CH_MULTICORE_INCLUDES "${CH_MULTICORE_INCLUDES};${Boost_INCLUDE_DIRS}")
endif()

INCLUDE_DIRECTORIES(${CH_MULTICORE_INCLUDES})

message(STATUS "Include dirs: ${CH_MULTICORE_INCLUDES}")

# ------------------------------------------------------------------------------
# Make some variables visible from parent directory
# ------------------------------------------------------------------------------

SET(CH_MULTICORE_CXX_FLAGS "${CH_MULTICORE_CXX_FLAGS}" PARENT_SCOPE)
SET(CH_MULTICORE_C_FLAGS "${CH_MULTICORE_C_FLAGS}" PARENT_SCOPE)
SET(CH_MULTICORE_INCLUDES "${CH_MULTICORE_INCLUDES}" PARENT_SCOPE)

# ------------------------------------------------------------------------------
# List the files in the Chrono::Multicore module
# ------------------------------------------------------------------------------

SET(ChronoEngine_Multicore_BASE
    ChApiMulticore.h
    ChMulticoreDefines.h
    ChSettings.h
    ChMeasures.h
    ChDataManager.h
    ChTimerMulticore.h
    ChDataManager.cpp
    )

SOURCE_GROUP("" FILES ${ChronoEngine_Multicore_BASE})

SET(ChronoEngine_Multicore_PHYSICS
    physics/ChSystemMulticore.h
    physics/ChSystemMulticore.cpp
    physics/ChSystemMulticoreNSC.cpp
    physics/ChSystemMulticoreSMC.cpp
    physics/Ch3DOFContainer.h
    physics/Ch3DOFContainer.cpp
    physics/ChFluidKernels.h
    physics/ChFluidContainer.cpp
    physics/ChParticleContainer.cpp
    physics/ChMPMSettings.h
    )

SOURCE_GROUP(physics FILES ${ChronoEngine_Multicore_PHYSICS})

SET(ChronoEngine_Multicore_CUDA
    cuda/matrixf.cuh
    cuda/svd.h
    cuda/ChCudaHelper.cuh
    cuda/ChGPUVector.cuh
    cuda/ChMPM.cu
    cuda/ChMPM.cuh
    cuda/ChMPMUtils.h
    )

SOURCE_GROUP(cuda FILES ${ChronoEngine_Multicore_CUDA})
    
SET(ChronoEngine_Multicore_SOLVER
    solver/ChSystemDescriptorMulticore.h
    solver/ChIterativeSolverMulticore.h
    solver/ChIterativeSolverMulticore.cpp
    solver/ChIterativeSolverMulticoreNSC.cpp
    solver/ChIterativeSolverMulticoreSMC.cpp
    solver/ChSolverMulticore.h
    solver/ChSolverMulticore.cpp
    solver/ChSolverMulticoreAPGD.cpp
    solver/ChSolverMulticoreAPGDREF.cpp
    solver/ChSolverMulticoreMINRES.cpp
    solver/ChSolverMulticoreBB.cpp
    solver/ChSolverMulticoreJacobi.cpp
    solver/ChSolverMulticoreCG.cpp
    solver/ChSolverMulticoreGS.cpp
    solver/ChSolverMulticoreSPGQP.cpp
    solver/ChShurProduct.cpp
    )

SOURCE_GROUP(solver FILES ${ChronoEngine_Multicore_SOLVER})

SET(ChronoEngine_Multicore_CONSTRAINTS
    constraints/ChConstraintRigidRigid.cpp
    constraints/ChConstraintRigidRigid.h
    constraints/ChConstraintBilateral.cpp
    constraints/ChConstraintBilateral.h
    constraints/ChConstraintUtils.cpp
    constraints/ChConstraintUtils.h
    )

SOURCE_GROUP(constraints FILES ${ChronoEngine_Multicore_CONSTRAINTS})

SET(ChronoEngine_Multicore_COLLISION
    collision/ChCollisionSystemChronoMulticore.h
    collision/ChCollisionSystemChronoMulticore.cpp
    collision/ChContactContainerMulticore.h
    collision/ChContactContainerMulticore.cpp
    collision/ChContactContainerMulticoreNSC.h
    collision/ChContactContainerMulticoreNSC.cpp
    collision/ChContactContainerMulticoreSMC.h
    collision/ChContactContainerMulticoreSMC.cpp
    )

SET(ChronoEngine_Multicore_COLLISION ${ChronoEngine_Multicore_COLLISION}
    collision/ChCollisionSystemBulletMulticore.h
    collision/ChCollisionSystemBulletMulticore.cpp
    )

SOURCE_GROUP(collision FILES ${ChronoEngine_Multicore_COLLISION})

# Windows doesn't automatically pick up Bullet sources from Chrono
IF(MSVC)
    SET(ChronoEngine_Multicore_COLLISION_BULLET
    # CollisionDispatch
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtUnionFind.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtCollisionDispatcher.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtCollisionDispatcherMt.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtSphereSphereCollisionAlgorithm.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtCollisionObject.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtSphereBoxCollisionAlgorithm.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtCollisionWorld.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtCollisionWorldImporter.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtSimulationIslandManager.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtCompoundCollisionAlgorithm.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtCompoundCompoundCollisionAlgorithm.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtManifoldResult.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtConvexConcaveCollisionAlgorithm.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtEmptyCollisionAlgorithm.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtSphereTriangleCollisionAlgorithm.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtConvexConvexAlgorithm.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtDefaultCollisionConfiguration.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtConvexPlaneCollisionAlgorithm.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtActivatingCollisionAlgorithm.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtBox2dBox2dCollisionAlgorithm.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtBoxBoxCollisionAlgorithm.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtBoxBoxDetector.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtGhostObject.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtConvex2dConvex2dAlgorithm.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtInternalEdgeUtility.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtHashedSimplePairCache.cpp
    # CollisionShapes
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtBarrelShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbt2DShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCEtriangleShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtBoxShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtTriangleMeshShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtBvhTriangleMeshShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtTriangleMesh.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtTriangleIndexVertexArray.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCollisionShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtTriangleCallback.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCompoundShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtTetrahedronShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConcaveShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtStridingMeshInterface.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConeShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtStaticPlaneShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexHullShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtSphereShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtPolyhedralConvexShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexTriangleMeshShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtOptimizedBvh.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCylinderShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCylindricalShellShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtMultiSphereShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtEmptyShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtMinkowskiSumShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtTriangleBuffer.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtHeightfieldTerrainShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCapsuleShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexInternalShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtUniformScalingShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtBox2dShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvex2dShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexPointCloudShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtMultimaterialTriangleMeshShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtScaledBvhTriangleMeshShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtShapeHull.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtTriangleIndexVertexMaterialArray.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtSdfCollisionShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtMiniSDF.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexPolyhedron.cpp
    # BroadphaseCollision
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/cbtAxisSweep3.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/cbtSimpleBroadphase.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/cbtOverlappingPairCache.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/cbtDispatcher.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/cbtCollisionAlgorithm.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/cbtDbvt.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/cbtDbvtBroadphase.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/cbtQuantizedBvh.cpp
    # NarrowPhaseCollision
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtContinuousConvexCollision.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtSubSimplexConvexCast.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtConvexCast.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtRaycastCallback.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtPersistentManifold.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtMinkowskiPenetrationDepthSolver.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtGjkConvexCast.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtGjkEpaPenetrationDepthSolver.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtVoronoiSimplexSolver.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtGjkPairDetector.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtGjkEpa2.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtPolyhedralContactClipping.cpp
    # LinearMath
        ${CHRONO_ROOT}/src/chrono/collision/bullet/LinearMath/cbtQuickprof.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/LinearMath/cbtAlignedAllocator.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/LinearMath/cbtGeometryUtil.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/LinearMath/cbtConvexHull.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/LinearMath/cbtConvexHullComputer.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/LinearMath/cbtSerializer.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/LinearMath/cbtSerializer64.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/LinearMath/cbtPolarDecomposition.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/LinearMath/cbtThreads.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/LinearMath/cbtVector3.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/LinearMath/TaskScheduler/cbtTaskScheduler.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/LinearMath/TaskScheduler/cbtThreadSupportPosix.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/LinearMath/TaskScheduler/cbtThreadSupportWin32.cpp
    # gimpact
        ${CHRONO_ROOT}/src/chrono/collision/gimpact/GIMPACT/Bullet/cbtGImpactCollisionAlgorithm.h
        ${CHRONO_ROOT}/src/chrono/collision/gimpact/GIMPACT/Bullet/cbtGImpactCollisionAlgorithm.cpp
        ${CHRONO_ROOT}/src/chrono/collision/gimpact/GIMPACT/Bullet/cbtGImpactQuantizedBvh.h
        ${CHRONO_ROOT}/src/chrono/collision/gimpact/GIMPACT/Bullet/cbtGImpactQuantizedBvh.cpp
        ${CHRONO_ROOT}/src/chrono/collision/gimpact/GIMPACT/Bullet/cbtTriangleShapeEx.h
        ${CHRONO_ROOT}/src/chrono/collision/gimpact/GIMPACT/Bullet/cbtTriangleShapeEx.cpp
        )

    SOURCE_GROUP(collision\\bullet FILES ${ChronoEngine_Multicore_COLLISION_BULLET})

    SET(ChronoEngine_Multicore_COLLISION
        ${ChronoEngine_Multicore_COLLISION}
        ${ChronoEngine_Multicore_COLLISION_BULLET}
        )
ENDIF()

# ------------------------------------------------------------------------------
# Add the ChronoEngine_multicore library
# ------------------------------------------------------------------------------

IF(USE_MULTICORE_CUDA)
    CUDA_ADD_LIBRARY(ChronoEngine_multicore
            ${ChronoEngine_Multicore_BASE}
            ${ChronoEngine_Multicore_PHYSICS}
            ${ChronoEngine_Multicore_CUDA}
            ${ChronoEngine_Multicore_COLLISION}
            ${ChronoEngine_Multicore_CONSTRAINTS}
            ${ChronoEngine_Multicore_SOLVER}
            ) 
    SET(CHRONO_MULTICORE_LINKED_LIBRARIES ChronoEngine ${CUDA_FRAMEWORK} ${OPENMP_LIBRARIES} ${TBB_LIBRARIES})
ELSE()
    ADD_LIBRARY(ChronoEngine_multicore
            ${ChronoEngine_Multicore_BASE}
            ${ChronoEngine_Multicore_PHYSICS}
            ${ChronoEngine_Multicore_COLLISION}
            ${ChronoEngine_Multicore_CONSTRAINTS}
            ${ChronoEngine_Multicore_SOLVER}
            )
    SET(CHRONO_MULTICORE_LINKED_LIBRARIES ChronoEngine ${OPENMP_LIBRARIES} ${TBB_LIBRARIES})
ENDIF()

# On Visual Studio, disable warning C4146 from Blaze
# ("unary minus operator applied to unsigned type, result still unsigned")
if(MSVC)
    target_compile_options(ChronoEngine_multicore PUBLIC "/wd4146" )
endif()

# Disable all warnings from external code
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set_source_files_properties(${ChronoEngine_Multicore_COLLISION_BULLET} PROPERTIES COMPILE_OPTIONS "-Wno-everything")
elseif(MSVC)
  set_source_files_properties(${ChronoEngine_Multicore_COLLISION_BULLET} PROPERTIES COMPILE_OPTIONS "/w")
else()
  set_source_files_properties(${ChronoEngine_Multicore_COLLISION_BULLET} PROPERTIES COMPILE_OPTIONS "-w")
endif()

SET_TARGET_PROPERTIES(ChronoEngine_multicore PROPERTIES
                      LINK_FLAGS "${CH_LINKERFLAG_LIB}")

target_compile_definitions(ChronoEngine_multicore PRIVATE "CH_API_COMPILE_MULTICORE")
target_compile_definitions(ChronoEngine_multicore PRIVATE "CH_IGNORE_DEPRECATED")

target_compile_definitions(ChronoEngine_multicore PRIVATE "BT_THREADSAFE")
if (USE_BULLET_DOUBLE)
    target_compile_definitions(ChronoEngine_multicore PRIVATE "BT_USE_DOUBLE_PRECISION")
endif()
if (USE_BULLET_OPENMP)
    target_compile_definitions(ChronoEngine_multicore PRIVATE "BT_USE_OPENMP")
endif()

TARGET_LINK_LIBRARIES(ChronoEngine_multicore ${CHRONO_MULTICORE_LINKED_LIBRARIES})

INSTALL(TARGETS ChronoEngine_multicore
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_multicore
        FILES_MATCHING PATTERN "*.h")

IF(USE_MULTICORE_CUDA)
  INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
          DESTINATION include/chrono_multicore
          FILES_MATCHING PATTERN "*.cuh")
ENDIF()

# Install required chrono_thirdparty headers
install(DIRECTORY ${CHRONO_ROOT}/src/chrono_thirdparty/easylogging
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

if(USE_MULTICORE_CUDA)
	if (DEFINED CUB_INCLUDE_DIR)
		install(DIRECTORY ${CHRONO_ROOT}/src/chrono_thirdparty/cub
			DESTINATION include/chrono_thirdparty
			FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")
	endif()
endif()

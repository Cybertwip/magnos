set(LIBRARIES ChronoEngine)
include_directories( ${CH_INCLUDES} )

set(TESTS
    utest_COLL_bullet_utils
)

if (${THRUST_FOUND})
   set(TESTS ${TESTS}
       utest_COLL_narrow_prims
       utest_COLL_narrow_mpr
   )
endif()

# Because Bullet classes are not DLL exported, the sources must be explicitly included.
IF (MSVC)
  SET(BULLET_FILES
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCollisionShape.h
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCollisionShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexInternalShape.h
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexInternalShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexShape.h
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtPolyhedralConvexShape.h
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtPolyhedralConvexShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexPolyhedron.h
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexPolyhedron.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtBoxShape.h
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtBoxShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConeShape.h
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConeShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCylinderShape.h
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCylinderShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCylindricalShellShape.h
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCylindricalShellShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtSphereShape.h
        ${CHRONO_ROOT}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtSphereShape.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/LinearMath/cbtAlignedAllocator.h
        ${CHRONO_ROOT}/src/chrono/collision/bullet/LinearMath/cbtAlignedAllocator.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/LinearMath/cbtGeometryUtil.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/LinearMath/cbtVector3.cpp
        ${CHRONO_ROOT}/src/chrono/collision/bullet/LinearMath/cbtConvexHullComputer.cpp
  )
  SOURCE_GROUP(bullet FILES ${BULLET_FILES})
ELSE()
  SET(BULLET_FILES "")
ENDIF()

message(STATUS "Unit test programs for COLLISION...")

foreach(PROGRAM ${TESTS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  ${PROGRAM}.cpp ${BULLET_FILES})
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBRARIES} gtest_main)

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    add_test(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
endforeach(PROGRAM)



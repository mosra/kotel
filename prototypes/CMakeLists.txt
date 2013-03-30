find_package(Magnum REQUIRED
    DebugTools
    Physics
    SceneGraph
    Sdl2Application)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CORRADE_CXX_FLAGS}")
include_directories(${MAGNUM_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/src)

set(PrototypeForces2d_SRCS
    forces-2d.cpp)

add_executable(prototype-forces-2d ${PrototypeForces2d_SRCS})
target_link_libraries(prototype-forces-2d
    ${MAGNUM_LIBRARIES}
    ${MAGNUM_DEBUGTOOLS_LIBRARIES}
    ${MAGNUM_PHYSICS_LIBRARIES}
    ${MAGNUM_SCENEGRAPH_LIBRARIES}
    ${MAGNUM_APPLICATION_LIBRARIES})

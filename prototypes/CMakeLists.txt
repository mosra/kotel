find_package(Magnum REQUIRED
    DebugTools
    MeshTools
    Physics
    Primitives
    SceneGraph
    Shaders)

if(NOT CORRADE_TARGET_NACL)
    find_package(Magnum REQUIRED Sdl2Application)
else()
    find_package(Magnum REQUIRED NaClApplication)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CORRADE_CXX_FLAGS}")
include_directories(${MAGNUM_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/src)

set(PrototypeForces2d_SRCS
    forces-2d.cpp)

add_executable(forces-2d ${PrototypeForces2d_SRCS})
target_link_libraries(forces-2d
    ${MAGNUM_LIBRARIES}
    ${MAGNUM_DEBUGTOOLS_LIBRARIES}
    ${MAGNUM_MESHTOOLS_LIBRARIES}
    ${MAGNUM_PHYSICS_LIBRARIES}
    ${MAGNUM_PRIMITIVES_LIBRARIES}
    ${MAGNUM_SCENEGRAPH_LIBRARIES}
    ${MAGNUM_SHADERS_LIBRARIES}
    ${MAGNUM_APPLICATION_LIBRARIES})

if(CORRADE_TARGET_NACL)
    install(TARGETS forces-2d DESTINATION ${CMAKE_INSTALL_PREFIX})
    install(FILES forces-2d.html forces-2d.nmf DESTINATION ${CMAKE_INSTALL_PREFIX})
endif()

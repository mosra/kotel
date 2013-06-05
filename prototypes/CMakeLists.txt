#
#   This file is part of Kotel.
#
#   Copyright © 2013 Vladimír Vondruš <mosra@centrum.cz>
#
#   Permission is hereby granted, free of charge, to any person obtaining a
#   copy of this software and associated documentation files (the "Software"),
#   to deal in the Software without restriction, including without limitation
#   the rights to use, copy, modify, merge, publish, distribute, sublicense,
#   and/or sell copies of the Software, and to permit persons to whom the
#   Software is furnished to do so, subject to the following conditions:
#
#   The above copyright notice and this permission notice shall be included
#   in all copies or substantial portions of the Software.
#
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
#   THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
#   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
#   DEALINGS IN THE SOFTWARE.
#

find_package(Magnum REQUIRED
    DebugTools
    MeshTools
    Primitives
    SceneGraph
    Shaders
    Shapes)

if(NOT CORRADE_TARGET_NACL)
    find_package(Magnum REQUIRED Sdl2Application)
else()
    find_package(Magnum REQUIRED NaClApplication)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CORRADE_CXX_FLAGS}")
include_directories(${MAGNUM_INCLUDE_DIRS}
    ${MAGNUM_APPLICATION_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/src)

set(PrototypeForces2d_SRCS
    forces-2d.cpp)

add_executable(forces-2d ${PrototypeForces2d_SRCS})
target_link_libraries(forces-2d
    ${MAGNUM_LIBRARIES}
    ${MAGNUM_DEBUGTOOLS_LIBRARIES}
    ${MAGNUM_MESHTOOLS_LIBRARIES}
    ${MAGNUM_PRIMITIVES_LIBRARIES}
    ${MAGNUM_SCENEGRAPH_LIBRARIES}
    ${MAGNUM_SHADERS_LIBRARIES}
    ${MAGNUM_SHAPES_LIBRARIES}
    ${MAGNUM_APPLICATION_LIBRARIES})

if(CORRADE_TARGET_NACL)
    install(TARGETS forces-2d DESTINATION ${CMAKE_INSTALL_PREFIX})
    install(FILES
        prototypes.css prototypes.js
        forces-2d.html forces-2d.nmf
        DESTINATION ${CMAKE_INSTALL_PREFIX})
endif()

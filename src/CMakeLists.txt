set(PEELMESH_HEADERS
    ${CMAKE_SOURCE_DIR}/include/peelmesh/geometry.hpp
    ${CMAKE_SOURCE_DIR}/include/peelmesh/kdtree.hpp
    ${CMAKE_SOURCE_DIR}/include/peelmesh/meshio.hpp
    ${CMAKE_SOURCE_DIR}/include/peelmesh/path.hpp
    ${CMAKE_SOURCE_DIR}/include/peelmesh/solver.hpp
    ${CMAKE_SOURCE_DIR}/include/peelmesh/pipeline.hpp)

set(PEELMESH_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/peelmesh/geometry.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/peelmesh/kdtree.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/peelmesh/meshio.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/peelmesh/path.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/peelmesh/solver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/peelmesh/pipeline.cpp
)

add_library(peelmesh)
target_sources(peelmesh
    PRIVATE ${PEELMESH_SOURCES}

    # PUBLIC ${PEELMESH_HEADERS}
)
target_include_directories(peelmesh
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        $<INSTALL_INTERFACE:include/deps/nanoflann>
        $<INSTALL_INTERFACE:include/deps/geometrycentral>
)

target_link_libraries(peelmesh PUBLIC eigen
    PRIVATE geometry-central)

find_package(Open3D CONFIG)

if(Open3D_FOUND)
    target_link_libraries(peelmesh PUBLIC Open3D::Open3D)
endif()

if(PEELMESH_BUILD_PYTHON_MODULE)
    add_subdirectory(bindings)
endif()

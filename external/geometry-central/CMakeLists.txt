include(FetchContent)

# Auto-detect pre-downloaded source
if(NOT PEELMESH_EXT_GEOMETRY_CENTRAL_SRC AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../_deps/geometry-central-src/CMakeLists.txt")
    set(PEELMESH_EXT_GEOMETRY_CENTRAL_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../../_deps/geometry-central-src")
endif()

# Auto-detect pre-downloaded Eigen (used by geometry-central)
if(NOT GC_EIGEN_LOCATION AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../_deps/eigen-src/signature_of_eigen3_matrix_library")
    set(GC_EIGEN_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/../../_deps/eigen-src")
    message(STATUS "Using local Eigen source from: ${GC_EIGEN_LOCATION}")
endif()

if(PEELMESH_EXT_GEOMETRY_CENTRAL_SRC AND EXISTS "${PEELMESH_EXT_GEOMETRY_CENTRAL_SRC}/CMakeLists.txt")
    message(STATUS "Using local geometry-central source from: ${PEELMESH_EXT_GEOMETRY_CENTRAL_SRC}")

    FetchContent_Declare(
        geometry-central
        SOURCE_DIR "${PEELMESH_EXT_GEOMETRY_CENTRAL_SRC}"
        GIT_SHALLOW FALSE
        GIT_PROGRESS FALSE

        CMAKE_ARGS
            -DCMAKE_CXX_STANDARD=${EXTERNAL_CXX_STANDARD}
            -DGC_ALWAYS_DOWNLOAD_EIGEN=OFF
            -DGC_EIGEN_LOCATION ${GC_EIGEN_LOCATION}

        EXCLUDE_FROM_ALL
    )
else()
    message(STATUS "Downloading and building geometry-central.")

    FetchContent_Declare(
        geometry-central
        GIT_REPOSITORY https://github.com/Nightinghost/geometry-central.git
        GIT_TAG master
        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE
        CMAKE_ARGS
            -DCMAKE_CXX_STANDARD=${EXTERNAL_CXX_STANDARD}
            -DGC_ALWAYS_DOWNLOAD_EIGEN=OFF
            -DGC_EIGEN_LOCATION ${GC_EIGEN_LOCATION}

        EXCLUDE_FROM_ALL
    )
endif()

FetchContent_MakeAvailable(geometry-central)

message(STATUS "Found geometry-central.")
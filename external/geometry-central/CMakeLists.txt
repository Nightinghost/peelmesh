include(FetchContent)

if(PEELMESH_EXT_GEOMETRY_CENTRAL_SRC AND EXISTS "${PEELMESH_EXT_GEOMETRY_CENTRAL_SRC}/CMakeLists.txt")
    message(STATUS "Using local geometry-central source from: ${PEELMESH_EXT_GEOMETRY_CENTRAL_SRC}")

    FetchContent_Declare(
        geometry-central
        SOURCE_DIR "${PEELMESH_EXT_GEOMETRY_CENTRAL_SRC}"
        GIT_SHALLOW FALSE
        GIT_PROGRESS FALSE

        CMAKE_ARGS
            -DCMAKE_CXX_STANDARD=${EXTERNAL_CXX_STANDARD}
            -DGC_ALWAYS_DOWNLOAD_EIGEN=ON
            -DGC_GC_EIGEN_LOCATION ${ext_eigen_SOURCE_DIR}

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
            -DGC_ALWAYS_DOWNLOAD_EIGEN=ON
            -DGC_GC_EIGEN_LOCATION ${ext_eigen_SOURCE_DIR}

        EXCLUDE_FROM_ALL
    )
endif()

FetchContent_MakeAvailable(geometry-central)

message(STATUS "Found geometry-central.")
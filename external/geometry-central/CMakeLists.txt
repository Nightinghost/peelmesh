include(FetchContent)

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

FetchContent_MakeAvailable(geometry-central)

message(STATUS "Found geometry-central.")
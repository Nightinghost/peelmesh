include(FetchContent)

# Auto-detect pre-downloaded source
if(NOT PEELMESH_EXT_FMT_SRC AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../fmt-src/CMakeLists.txt")
    set(PEELMESH_EXT_FMT_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../fmt-src")
endif()

if(PEELMESH_EXT_FMT_SRC AND EXISTS ${PEELMESH_EXT_FMT_SRC}/CMakeLists.txt)
    message(STATUS "Using external fmt source dir: ${PEELMESH_EXT_FMT_SRC}")

    FetchContent_Declare(
        fmtlib
        SOURCE_DIR "${PEELMESH_EXT_FMT_SRC}"

        CMAKE_ARGS
            -DCMAKE_CXX_STANDARD=${EXTERNAL_CXX_STANDARD}

        OVERRIDE_FIND_PACKAGE
    )

else()
    FetchContent_Declare(
        fmtlib
        GIT_REPOSITORY https://github.com/fmtlib/fmt.git
        GIT_TAG 12.0.0
        OVERRIDE_FIND_PACKAGE
    )
endif()

FetchContent_MakeAvailable(fmtlib)
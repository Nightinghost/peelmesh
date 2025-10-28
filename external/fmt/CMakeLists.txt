include(FetchContent)

if(PEELMESH_EXT_FMT_SOURCE_DIR AND EXISTS ${PEELMESH_EXT_FMT_SOURCE_DIR}/CMakeLists.txt)
    message(STATUS "Using external fmt source dir: ${PEELMESH_EXT_FMT_SOURCE_DIR}")

    FetchContent_Declare(
        fmtlib
        SOURCE_DIR "${PEELMESH_EXT_FMT_SOURCE_DIR}"

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
set(FORMAT_DEP fmt::fmt)
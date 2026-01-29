include(FetchContent)
set(FETCHCONTENT_QUIET FALSE)

if(PEELMESH_EXT_GOOGLETEST_SRC AND EXISTS ${PEELMESH_EXT_GOOGLETEST_SRC}/CMakeLists.txt)
    message(STATUS "Using local googletest source directory: ${PEELMESH_EXT_GOOGLE_TEST_SRC}")

    FetchContent_Declare(
        googletest
        SOURCE_DIR ${PEELMESH_EXT_GOOGLETEST_SRC}
        GIT_SHALLOW FALSE
        GIT_PROGRESS FALSE

        EXCLUDE_FROM_ALL
    )

else()
    FetchContent_Declare(
        googletest

        # PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../../.deps/googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.12.1
        GIT_PROGRESS TRUE
        GIT_SHALLOW TRUE

        EXCLUDE_FROM_ALL
    )
endif()

FetchContent_MakeAvailable(googletest)
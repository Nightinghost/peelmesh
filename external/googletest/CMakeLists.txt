include(FetchContent)

set(FETCHCONTENT_QUIET FALSE)
FetchContent_Declare(
    googletest

    # PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../../.deps/googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.12.1
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE

    EXCLUDE_FROM_ALL

    # CMAKE_ARGS
    # -DCMAKE_CXX_STANDARD=${EXTERNAL_CXX_STANDARD}
)

FetchContent_MakeAvailable(googletest)
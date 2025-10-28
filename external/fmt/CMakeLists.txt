include(FetchContent)

FetchContent_Declare(
    fmtlib
    GIT_REPOSITORY https://github.com/fmtlib/fmt.git
    GIT_TAG 12.0.0
    OVERRIDE_FIND_PACKAGE
)

FetchContent_MakeAvailable(fmtlib)

set(FORMAT_DEP fmt::fmt)

if(TARGET fmt::fmt)
    target_compile_features(fmt::fmt INTERFACE ${CMAKE_CXX_STANDARD})
endif()
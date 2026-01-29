include(FetchContent)

if(PEELMESH_EXT_PYBIND11_SRC AND EXISTS ${PEELMESH_EXT_PYBIND11_SRC}/CMakeLists.txt)
    message(STATUS "Using local pybind11 source from: ${PEELMESH_EXT_PYBIND11_SRC}")

    FetchContent_Declare(
        pybind11
        SOURCE_DIR ${PEELMESH_EXT_PYBIND11_SRC}
        GIT_SHALLOW FALSE
        GIT_PROGRESS FALSE
    )

else()
    message(STATUS "Downloading and build pybind11.")

    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG v2.13.5
    )
endif()

FetchContent_MakeAvailable(pybind11)
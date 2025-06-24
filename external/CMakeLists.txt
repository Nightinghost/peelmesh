add_subdirectory(geometry-central)

if(PEELMESH_BUILD_PYTHON_MODULE)
    add_subdirectory(pybind11)
endif()

if(PEELMESH_BUILD_EXAMPLES)
    add_subdirectory(open3d)
endif()

if(PEELMESH_BUILD_UNIT_TESTS)
    add_subdirectory(googletest)
endif()
find_package(Python REQUIRED COMPONENTS Interpreter Development)

if(Python_EXECUTABLE)
    message(STATUS "Using Python interpreter: ${Python_EXECUTABLE}")
else()
    message(FATAL_ERROR "Python interpreter not found!")
endif()

pybind11_add_module(peelmesh_module peelmesh_module.cpp)

target_link_libraries(peelmesh_module PRIVATE peelmesh geometry-central)

set_target_properties(peelmesh_module PROPERTIES
    OUTPUT_NAME "peelmesh"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/$<CONFIG>/python"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/$<CONFIG>/python"
)

install(TARGETS peelmesh_module
    COMPONENT python
    LIBRARY DESTINATION peelmesh
    RUNTIME DESTINATION peelmesh
    ARCHIVE DESTINATION peelmesh
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
    DESTINATION peelmesh
    COMPONENT python
)

if(SKBUILD)
    message(STATUS "Building via scikit-build-core, enabling Python module")
    add_custom_command(TARGET peelmesh_module POST_BUILD
        WORKING_DIRECTORY $<TARGET_FILE_DIR:peelmesh_module>
        COMMAND ${Python_EXECUTABLE} -m pybind11_stubgen peelmesh -o $<TARGET_FILE_DIR:peelmesh_module>/
        COMMENT "Generating .pyi stub for peelmesh"
    )
endif()

install(FILES $<TARGET_FILE_DIR:peelmesh_module>/peelmesh.pyi
    DESTINATION peelmesh
    COMPONENT python
)

# find_program(PYBIND11_STUBGEN pybind11-stubgen
# PATHS $ENV{CONDA_PREFIX}/Scripts
# $ENV{VIRTUAL_ENV}/Scripts
# ENV PATH
# )

# if(PYBIND11_STUBGEN)
# message(STATUS "Using pybind11-stubgen: ${PYBIND11_STUBGEN}")
# add_custom_command(TARGET peelmesh_module POST_BUILD
# COMMAND ${PYBIND11_STUBGEN} $<TARGET_FILE:peelmesh_module> -o $<TARGET_FILE_DIR:peelmesh_module>/peelmesh
# COMMENT "Generating .pyi stub for peelmesh"
# )

# install(FILES $<TARGET_FILE_DIR:peelmesh_module>/peelmesh.pyi
# DESTINATION peelmesh
# COMPONENT python
# )
# else()
# message(STATUS "pybind11-stubgen not found, stubs will not be generated")
# endif()
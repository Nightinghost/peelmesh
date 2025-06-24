find_package(Python REQUIRED COMPONENTS Interpreter Development)

if(Python_EXECUTABLE)
    message(STATUS "Using Python interpreter: ${Python_EXECUTABLE}")
else()
    message(FATAL_ERROR "Python interpreter not found!")
endif()

pybind11_add_module(peelmesh_module peelmesh_module.cpp)

target_link_libraries(peelmesh_module PRIVATE peelmesh geometry-central)

set_target_properties(peelmesh_module PROPERTIES OUTPUT_NAME "peelmesh" ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/$<CONFIG>/python" LIBRARY_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/$<CONFIG>/python")
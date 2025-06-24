cmake_minimum_required(VERSION 3.21 FATAL_ERROR)

find_program(SCCACHE sccache QUIET)

if(SCCACHE_FOUND)
    set(CMAKE_C_COMPILER_LAUNCHER ${SCCACHE})
    set(CMAKE_CXX_COMPILER_LAUNCHER ${SCCACHE})
    set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT Embedded)
    cmake_policy(SET CMP0141 NEW)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CONFIGURATION_TYPES "Release;RelWithDebInfo;MinSizeRel")
else()
    set(CMAKE_CONFIGURATION_TYPES "Debug")
endif()

project(peelmesh LANGUAGES CXX VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/utf-8 /wd4996 /wd4267 /wd4244)
else()
    add_compile_options(-fPIC)
endif()

include(GNUInstallDirs)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})

option(PEELMESH_BUILD_UNIT_TESTS "Build unit tests" OFF)
option(PEELMESH_BUILD_PYTHON_MODULE "Build Python module" OFF)
option(PEELMESH_BUILD_EXAMPLES "Build examples" OFF)

add_subdirectory(external)
add_subdirectory(src)

if(PEELMESH_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(PEELMESH_BUILD_UNIT_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

include(CMakePackageConfigHelpers)

install(TARGETS peelmesh geometry-central nanort nanoflann happly eigen
    EXPORT peelmeshTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(DIRECTORY ${geometry-central_SOURCE_DIR}/deps/nanoflann/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/deps/nanoflann
)
install(DIRECTORY ${geometry-central_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/deps/geometrycentral
)

if(PEELMESH_BUILD_EXAMPLES)
    install(DIRECTORY ${Open3D_INCLUDE_DIR}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/deps/Open3D)
endif()

install(EXPORT peelmeshTargets
    FILE peelmeshTargets.cmake
    NAMESPACE peelmesh::
    DESTINATION ${CMAKE_INSTALL_PREFIX}/CMake/
)

configure_package_config_file(
    ${CMAKE_SOURCE_DIR}/cmake/peelmeshConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/peelmeshConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/CMake/
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/peelmeshConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/peelmeshConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/peelmeshConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_PREFIX}/CMake/
)

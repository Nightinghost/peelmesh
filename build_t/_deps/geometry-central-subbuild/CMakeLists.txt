# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.30.4)

# Reject any attempt to use a toolchain file. We must not use one because
# we could be downloading it here. If the CMAKE_TOOLCHAIN_FILE environment
# variable is set, the cache variable will have been initialized from it.
unset(CMAKE_TOOLCHAIN_FILE CACHE)
unset(ENV{CMAKE_TOOLCHAIN_FILE})

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(geometry-central-populate NONE)



include(ExternalProject)
ExternalProject_Add(geometry-central-populate
                     "UPDATE_DISCONNECTED" "False" "EXTERNALPROJECT_INTERNAL_ARGUMENT_SEPARATOR" "GIT_SHALLOW" "FALSE" "GIT_PROGRESS" "FALSE" "CMAKE_ARGS" "-DCMAKE_CXX_STANDARD=" "-DGC_ALWAYS_DOWNLOAD_EIGEN=ON" "-DGC_GC_EIGEN_LOCATION"
                    SOURCE_DIR          "D:/Project/GitClone/geometry-central"
                    BINARY_DIR          "D:/Project/archive/peelmesh_archive/build_t/_deps/geometry-central-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
                    USES_TERMINAL_PATCH     YES
)



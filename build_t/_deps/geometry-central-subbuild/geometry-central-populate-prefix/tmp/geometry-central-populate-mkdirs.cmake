# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "D:/Project/GitClone/geometry-central")
  file(MAKE_DIRECTORY "D:/Project/GitClone/geometry-central")
endif()
file(MAKE_DIRECTORY
  "D:/Project/archive/peelmesh_archive/build_t/_deps/geometry-central-build"
  "D:/Project/archive/peelmesh_archive/build_t/_deps/geometry-central-subbuild/geometry-central-populate-prefix"
  "D:/Project/archive/peelmesh_archive/build_t/_deps/geometry-central-subbuild/geometry-central-populate-prefix/tmp"
  "D:/Project/archive/peelmesh_archive/build_t/_deps/geometry-central-subbuild/geometry-central-populate-prefix/src/geometry-central-populate-stamp"
  "D:/Project/archive/peelmesh_archive/build_t/_deps/geometry-central-subbuild/geometry-central-populate-prefix/src"
  "D:/Project/archive/peelmesh_archive/build_t/_deps/geometry-central-subbuild/geometry-central-populate-prefix/src/geometry-central-populate-stamp"
)

set(configSubDirs Debug)
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Project/archive/peelmesh_archive/build_t/_deps/geometry-central-subbuild/geometry-central-populate-prefix/src/geometry-central-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Project/archive/peelmesh_archive/build_t/_deps/geometry-central-subbuild/geometry-central-populate-prefix/src/geometry-central-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()

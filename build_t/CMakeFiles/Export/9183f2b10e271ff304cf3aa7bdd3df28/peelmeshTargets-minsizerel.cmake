#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "peelmesh::peelmesh" for configuration "MinSizeRel"
set_property(TARGET peelmesh::peelmesh APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(peelmesh::peelmesh PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_MINSIZEREL "CXX"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/lib/peelmesh.lib"
  )

list(APPEND _cmake_import_check_targets peelmesh::peelmesh )
list(APPEND _cmake_import_check_files_for_peelmesh::peelmesh "${_IMPORT_PREFIX}/lib/peelmesh.lib" )

# Import target "peelmesh::geometry-central" for configuration "MinSizeRel"
set_property(TARGET peelmesh::geometry-central APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(peelmesh::geometry-central PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_MINSIZEREL "CXX"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/lib/geometry-central.lib"
  )

list(APPEND _cmake_import_check_targets peelmesh::geometry-central )
list(APPEND _cmake_import_check_files_for_peelmesh::geometry-central "${_IMPORT_PREFIX}/lib/geometry-central.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

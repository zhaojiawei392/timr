#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "qpOASES::qpOASES" for configuration ""
set_property(TARGET qpOASES::qpOASES APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(qpOASES::qpOASES PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libqpOASES.a"
  )

list(APPEND _cmake_import_check_targets qpOASES::qpOASES )
list(APPEND _cmake_import_check_files_for_qpOASES::qpOASES "${_IMPORT_PREFIX}/lib/libqpOASES.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

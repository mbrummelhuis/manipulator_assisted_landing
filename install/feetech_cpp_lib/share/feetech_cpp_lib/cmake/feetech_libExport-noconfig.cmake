#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "feetech_cpp_lib::feetech_lib" for configuration ""
set_property(TARGET feetech_cpp_lib::feetech_lib APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(feetech_cpp_lib::feetech_lib PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libfeetech_lib.a"
  )

list(APPEND _cmake_import_check_targets feetech_cpp_lib::feetech_lib )
list(APPEND _cmake_import_check_files_for_feetech_cpp_lib::feetech_lib "${_IMPORT_PREFIX}/lib/libfeetech_lib.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

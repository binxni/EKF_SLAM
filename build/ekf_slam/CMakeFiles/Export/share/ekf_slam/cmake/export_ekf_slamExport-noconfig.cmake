#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ekf_slam::ekf_slam_lib" for configuration ""
set_property(TARGET ekf_slam::ekf_slam_lib APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ekf_slam::ekf_slam_lib PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libekf_slam_lib.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS ekf_slam::ekf_slam_lib )
list(APPEND _IMPORT_CHECK_FILES_FOR_ekf_slam::ekf_slam_lib "${_IMPORT_PREFIX}/lib/libekf_slam_lib.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

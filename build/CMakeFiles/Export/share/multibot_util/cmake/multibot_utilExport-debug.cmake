#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "multibot_util::multibot_util" for configuration "Debug"
set_property(TARGET multibot_util::multibot_util APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(multibot_util::multibot_util PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libmultibot_util.so"
  IMPORTED_SONAME_DEBUG "libmultibot_util.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS multibot_util::multibot_util )
list(APPEND _IMPORT_CHECK_FILES_FOR_multibot_util::multibot_util "${_IMPORT_PREFIX}/lib/libmultibot_util.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

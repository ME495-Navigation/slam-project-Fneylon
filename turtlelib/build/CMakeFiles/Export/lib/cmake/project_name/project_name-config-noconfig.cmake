#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "project_name::Name" for configuration ""
set_property(TARGET project_name::Name APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(project_name::Name PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/Name"
  )

list(APPEND _IMPORT_CHECK_TARGETS project_name::Name )
list(APPEND _IMPORT_CHECK_FILES_FOR_project_name::Name "${_IMPORT_PREFIX}/bin/Name" )

# Import target "project_name::libname" for configuration ""
set_property(TARGET project_name::libname APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(project_name::libname PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/liblibname.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS project_name::libname )
list(APPEND _IMPORT_CHECK_FILES_FOR_project_name::libname "${_IMPORT_PREFIX}/lib/liblibname.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

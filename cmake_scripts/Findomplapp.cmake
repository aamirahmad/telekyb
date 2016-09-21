# - Try to find OMPLAPP
# Once done this will define
#  OMPLAPP_FOUND - System has OMPLAPP
#  OMPLAPP_INCLUDE_DIRS - The OMPLAPP include directories
#  OMPLAPP_LIBRARIES - The libraries needed to use OMPLAPP
#  OMPLAPP_DEFINITIONS - Compiler switches required for using OMPLAPP

IF (DEFINED ENV{OMPLAPP_ROOT_DIR})
	SET(OMPLAPP_ROOT_DIR "$ENV{OMPLAPP_ROOT_DIR}")
ENDIF()

# Look for the header file.
FIND_PATH(OMPLAPP_INCLUDE_DIR NAMES omplapp/config.h
	HINTS ${OMPLAPP_ROOT_DIR} /usr/local )

UNSET(OMPLAPP_LIBRARY_APP)
UNSET(OMPLAPP_LIBRARY_BASE)

# Look for the library.
FIND_LIBRARY(OMPLAPP_LIBRARY_APP NAMES ompl_app
	HINTS ${OMPLAPP_ROOT_DIR} ${OMPLAPP_ROOT_DIR}/build/Release/lib
)
FIND_LIBRARY(OMPLAPP_LIBRARY_BASE NAMES ompl_app_base
	HINTS ${OMPLAPP_ROOT_DIR} ${OMPLAPP_ROOT_DIR}/build/Release/lib
)



set(OMPLAPP_LIBRARIES ${OMPLAPP_LIBRARY_BASE} ${OMPLAPP_LIBRARY_APP})


set(OMPLAPP_INCLUDE_DIRS ${OMPLAPP_ROOT_DIR}/src )


include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set OMPLAPP_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(OMPLAPP  DEFAULT_MSG
                                  OMPLAPP_LIBRARIES OMPLAPP_INCLUDE_DIRS)

mark_as_advanced(OMPLAPP_INCLUDE_DIRS OMPLAPP_LIBRARIES)

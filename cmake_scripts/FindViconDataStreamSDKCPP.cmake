# Locate ViconDataStreamSDK - CPP
#
# This module defines
#  ViconDataStreamSDKCPP_FOUND, if false, do not try to link to ViconDataStreamSDK
#  ViconDataStreamSDKCPP_LIBRARY, where to find ViconDataStreamSDK
#  ViconDataStreamSDKCPP_INCLUDE_DIR, where to find Client.h
#
# If ViconDataStreamSDK is not installed in a standard path, you can use the ViconDataStreamSDKCPP_DIR CMake variable
# to tell CMake where ViconDataStreamSDK is.

# find the ViconDataStreamSDK include directory
find_path(ViconDataStreamSDKCPP_INCLUDE_DIR ViconDataStreamSDK_CPP/Client.h
  PATHS "${ViconDataStreamSDKCPP_ROOT}/include/")

message(STATUS ${ViconDataStreamSDKCPP_INCLUDE_DIR})

# find the ViconDataStreamSDK library
find_library(ViconDataStreamSDKCPP_LIBRARY
  NAMES ViconDataStreamSDK_CPP
  PATH_SUFFIXES lib64 lib
  PATHS "${ViconDataStreamSDKCPP_ROOT}")

# handle the QUIETLY and REQUIRED arguments and set ViconDataStreamSDKCPP_FOUND to TRUE if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(ViconDataStreamSDKCPP DEFAULT_MSG ViconDataStreamSDKCPP_INCLUDE_DIR ViconDataStreamSDKCPP_LIBRARY)
mark_as_advanced(ViconDataStreamSDKCPP_INCLUDE_DIR ViconDataStreamSDKCPP_LIBRARY)

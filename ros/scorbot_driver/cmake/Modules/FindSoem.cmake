#  CMake script for finding SOEM (Simple Open Source EtherCAT Master)
# 
#  SOEM_FOUND - soem found
#  SOEM_INCLUDE_DIR - the soem include directory
#  SOEM_LIBRARIES - soem library
#

FIND_PATH(SOEM_INCLUDE_DIR NAMES ethercatmain.h
  PATHS
  $ENV{SOEM_PATH}/include/soem
  /usr/include/
  /usr/include/soem/src
  /usr/local/include/
  /usr/local/include/soem/src
  /opt/local/include/
  /opt/local/include/soem/src
)

FIND_LIBRARY(SOEM_LIBRARIES NAMES soem
  PATHS
  $ENV{SOEM_PATH}/lib/
  /usr/lib
  /usr/local/lib
  /opt/local/lib
)

include(FindPackageHandleStandardArgs)
# Soem is considered to be found (SOEM_FOUND is set to TRUE), if both SOEM_INCLUDE_DIR and SOEM_LIBRARIES are valid.

find_package_handle_standard_args(Soem  DEFAULT_MSG
                                  SOEM_INCLUDE_DIR SOEM_LIBRARIES)

# Show the SOEM_INCLUDE_DIR and SOEM_LIBRARY_DIR variables only in the advanced view
IF (SOEM_FOUND)
  MARK_AS_ADVANCED(SOEM_INCLUDE_DIR SOEM_LIBRARIES)
ENDIF (SOEM_FOUND)



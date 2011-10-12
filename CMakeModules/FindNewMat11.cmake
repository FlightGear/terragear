
FIND_PATH(NEWMAT_INCLUDE_DIR newmat/newmat.h
  HINTS $ENV{NEWMAT_DIR}
  PATH_SUFFIXES include
  PATHS
  /usr/local
  /usr
  /opt
)


FIND_LIBRARY(NEWMAT_LIBRARY
  NAMES newmat
  HINTS
  $ENV{NEWMAT_DIR}
  PATH_SUFFIXES lib64 lib libs64 libs libs/Win32 libs/Win64
  PATHS
  /usr/local
  /usr
  /opt
)

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(newmat DEFAULT_MSG
     NEWMAT_LIBRARY NEWMAT_INCLUDE_DIR)

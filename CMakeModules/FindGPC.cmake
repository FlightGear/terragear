
FIND_PATH(GPC_INCLUDE_DIR gpc.h
  HINTS $ENV{GPC_DIR}
  PATH_SUFFIXES include
  PATHS
  /usr/local
  /usr
  /opt
)


FIND_LIBRARY(GPC_LIBRARY
  NAMES genpolyclip gpc
  HINTS
  $ENV{GPC_DIR}
  PATH_SUFFIXES lib64 lib libs64 libs libs/Win32 libs/Win64
  PATHS
  /usr/local
  /usr
  /opt
)

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GPC DEFAULT_MSG
     GPC_LIBRARY GPC_INCLUDE_DIR)

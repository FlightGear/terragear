

include_directories(${PROJECT_SOURCE_DIR}/src/Lib)

# seems unfortunate that lib depends on other dirs
#include_directories(${PROJECT_SOURCE_DIR}/src/BuildTiles)

add_subdirectory(poly2ogr)

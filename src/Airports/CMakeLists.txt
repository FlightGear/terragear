include_directories(${PROJECT_SOURCE_DIR}/src/Lib)
include_directories(${PROJECT_SOURCE_DIR}/src/BuildTiles)

if (NEWMAT_FOUND)
add_subdirectory(GenAirports)
add_subdirectory(GenAirports850)
endif (NEWMAT_FOUND)

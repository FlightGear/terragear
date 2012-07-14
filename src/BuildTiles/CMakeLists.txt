

include_directories(${PROJECT_SOURCE_DIR}/src/Lib)
include_directories(${PROJECT_SOURCE_DIR}/src/BuildTiles)

add_subdirectory(Osgb36)
add_subdirectory(Parallel)      
add_subdirectory(Match)    
add_subdirectory(Main)    

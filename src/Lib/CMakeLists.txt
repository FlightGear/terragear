
include_directories(${PROJECT_SOURCE_DIR}/src/Lib)

# seems unfortunate that lib depends on other dirs
include_directories(${PROJECT_SOURCE_DIR}/src/BuildTiles)

add_subdirectory(Array)
add_subdirectory(DEM)
add_subdirectory(Geometry)
add_subdirectory(HGT)
add_subdirectory(Optimize)
add_subdirectory(Output)
add_subdirectory(Polygon)
add_subdirectory(TriangleJRS)
add_subdirectory(e00)
add_subdirectory(landcover)
add_subdirectory(poly2tri)
add_subdirectory(shapelib)
add_subdirectory(vpf)


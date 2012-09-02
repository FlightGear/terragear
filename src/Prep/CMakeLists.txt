include_directories(${PROJECT_SOURCE_DIR}/src/Lib)

# seems unfortunate that lib depends on other dirs
#include_directories(${PROJECT_SOURCE_DIR}/src/BuildTiles)

add_subdirectory(GDALChop)
add_subdirectory(GSHHS)
add_subdirectory(DemChop)
add_subdirectory(DemInfo)
add_subdirectory(DemRaw2ascii)
add_subdirectory(E00Lines)
add_subdirectory(Photo)
add_subdirectory(ShapeFile)
add_subdirectory(TGVPF)
add_subdirectory(Terra)
add_subdirectory(TerraFit)
add_subdirectory(OGRDecode)
add_subdirectory(UserDef)

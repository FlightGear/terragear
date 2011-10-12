

add_library(Clipper STATIC 
    clipper.cxx clipper.hxx priorities.cxx priorities.hxx
)

add_executable(testclipper testclipper.cxx)

target_link_libraries(testclipper
    Clipper Osgb36 Triangulate Polygon Geometry
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
	genpolyclip)

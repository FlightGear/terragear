
add_executable(e00lines 
    main.cxx)

target_link_libraries(e00lines 
    e00
    Polygon Geometry Output poly2tri TriangleJRS vpf
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
	genpolyclip)
	

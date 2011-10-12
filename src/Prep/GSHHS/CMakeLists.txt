
add_executable(gshhs 
    main.cxx
	gshhs.h
	gshhs_split.cxx gshhs_split.hxx)

target_link_libraries(gshhs 
    Polygon Geometry Output poly2tri
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
	genpolyclip)
	
add_executable(gshhs_debug 
    debug.cxx)

target_link_libraries(gshhs_debug 
    Polygon Geometry Output poly2tri
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
	genpolyclip)
    	

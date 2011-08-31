

add_executable(shape-decode shape-decode.cxx)

target_link_libraries(shape-decode 
    shape Polygon Geometry Output poly2tri TriangleJRS vpf
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
	${GPC_LIBRARY})
	
add_executable(noaa-decode noaa-decode.cxx)

target_link_libraries(noaa-decode 
    shape Polygon Geometry Output poly2tri TriangleJRS vpf
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
	${GPC_LIBRARY})
    	

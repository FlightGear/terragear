

add_executable(fgfs-construct 
    construct.cxx construct.hxx
	usgs.cxx main.cxx)

set_target_properties(fgfs-construct PROPERTIES
        COMPILE_DEFINITIONS 
        "DEFAULT_USGS_MAPFILE=\"${PKGDATADIR}/usgsmap.txt\";DEFAULT_PRIORITIES_FILE=\"${PKGDATADIR}/default_priorities.txt\"" )
            
target_link_libraries(fgfs-construct
    Clipper Osgb36 Triangulate GenOutput 
    Match
    Polygon Geometry
    Array Optimize Output landcover poly2tri
    TriangleJRS
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
	genpolyclip)


add_executable(fgfs-master 
    master.cxx)
    
target_link_libraries(fgfs-master
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
	)
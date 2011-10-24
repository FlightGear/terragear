

add_executable(fgfs-construct 
    construct.cxx construct.hxx
	usgs.cxx main.cxx)

set_target_properties(fgfs-construct PROPERTIES
        COMPILE_DEFINITIONS 
        "DEFAULT_USGS_MAPFILE=\"${PKGDATADIR}/usgsmap.txt\";DEFAULT_PRIORITIES_FILE=\"${PKGDATADIR}/default_priorities.txt\"" )
            
target_link_libraries(fgfs-construct
    Clipper GenOutput Osgb36 Triangulate  
    Match
    Polygon Geometry
    Array Optimize Output landcover poly2tri
    TriangleJRS
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
	${GPC_LIBRARY})

install(TARGETS fgfs-construct RUNTIME DESTINATION bin)
INSTALL(FILES usgsmap.txt DESTINATION ${PKGDATADIR} )


add_executable(fgfs-master 
    master.cxx)
    
target_link_libraries(fgfs-master
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
	)

install(TARGETS fgfs-master RUNTIME DESTINATION bin)

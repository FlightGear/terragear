
add_executable(demchop demchop.cxx point2d.hxx)

target_link_libraries(demchop 
    DEM
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})

add_executable(hgtchop hgtchop.cxx point2d.hxx)

target_link_libraries(hgtchop 
    HGT
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})
	
if(TIFF_FOUND)
add_executable(srtmchop srtmchop.cxx point2d.hxx)
target_link_libraries(srtmchop 
    HGT
    ${TIFF_LIBRARIES}
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})
	
endif(TIFF_FOUND)

add_executable(fillvoids fillvoids.cxx)
target_link_libraries(fillvoids 
    Array
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})

add_executable(testassem testassem.cxx)
target_link_libraries(testassem 
    Array
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})
	

add_executable(demchop demchop.cxx)

target_link_libraries(demchop 
    DEM
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})

install(TARGETS demchop RUNTIME DESTINATION bin)

add_executable(hgtchop hgtchop.cxx)

target_link_libraries(hgtchop 
    HGT
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})

install(TARGETS hgtchop RUNTIME DESTINATION bin)

if(TIFF_FOUND)
if(MSVC AND CMAKE_CL_64)
	set( SRTMCHOP_LIBRARIES ${JPEG_LIBRARY} )
endif(MSVC AND CMAKE_CL_64)
add_executable(srtmchop srtmchop.cxx)
target_link_libraries(srtmchop 
    HGT
    ${TIFF_LIBRARIES}
	${SRTMCHOP_LIBRARIES}
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})
	
install(TARGETS srtmchop RUNTIME DESTINATION bin)
endif(TIFF_FOUND)

add_executable(fillvoids fillvoids.cxx)
target_link_libraries(fillvoids 
    Array
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})

install(TARGETS fillvoids RUNTIME DESTINATION bin)

add_executable(testassem testassem.cxx)
target_link_libraries(testassem 
    Array
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})

install(TARGETS testassem RUNTIME DESTINATION bin)

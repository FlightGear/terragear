include_directories(${PROJECT_SOURCE_DIR}/src/Lib/terragear)

add_executable(hgtchop hgtchop.cxx)

target_link_libraries(hgtchop 
    HGT
	${ZLIB_LIBRARY}
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})

install(TARGETS hgtchop RUNTIME DESTINATION bin)

add_executable(dtedchop dtedchop.cxx)

target_link_libraries(dtedchop 
    HGT
	${ZLIB_LIBRARY}
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})

install(TARGETS dtedchop RUNTIME DESTINATION bin)


if(TIFF_FOUND)
if(MSVC AND CMAKE_CL_64)
	set( SRTMCHOP_LIBRARIES ${JPEG_LIBRARY} )
endif(MSVC AND CMAKE_CL_64)
add_executable(srtmchop srtmchop.cxx)
target_link_libraries(srtmchop 
    HGT
    ${ZLIB_LIBRARY}
    ${TIFF_LIBRARIES}
	${SRTMCHOP_LIBRARIES}
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})
	
install(TARGETS srtmchop RUNTIME DESTINATION bin)
endif(TIFF_FOUND)

add_executable(fillvoids fillvoids.cxx)
target_link_libraries(fillvoids 
    Array
    terragear
	${ZLIB_LIBRARY}
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})

install(TARGETS fillvoids RUNTIME DESTINATION bin)

add_executable(testassem testassem.cxx)
target_link_libraries(testassem 
    Array
    terragear
    ${ZLIB_LIBRARY}
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})

install(TARGETS testassem RUNTIME DESTINATION bin)

include_directories(${GPC_INCLUDE_DIR})

add_executable(photo
    photo.cxx)

target_link_libraries(photo
    Polygon Geometry Array Output poly2tri TriangleJRS
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
	${GPC_LIBRARY})

install(TARGETS photo RUNTIME DESTINATION bin)

add_executable(wgs84offset
    wgs84offset.cxx)

target_link_libraries(wgs84offset
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

install(TARGETS wgs84offset RUNTIME DESTINATION bin)


add_executable(findcorners
    findcorners.cxx)

target_link_libraries(findcorners
        ${SIMGEAR_CORE_LIBRARIES}
        ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

install(TARGETS findcorners RUNTIME DESTINATION bin)

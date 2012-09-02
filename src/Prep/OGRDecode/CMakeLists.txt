
if (GDAL_FOUND)

include_directories(
	${GDAL_INCLUDE_DIR}
	${GPC_INCLUDE_DIR}
)

add_executable(ogr-decode ogr-decode.cxx)

target_link_libraries(ogr-decode 
    ${GDAL_LIBRARY}
    Polygon Geometry  poly2tri TriangleJRS
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
	${GPC_LIBRARY})

install(TARGETS ogr-decode RUNTIME DESTINATION bin)
endif(GDAL_FOUND)

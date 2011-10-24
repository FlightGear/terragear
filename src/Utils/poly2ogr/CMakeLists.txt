
if (GDAL_FOUND)

include_directories(${GDAL_INCLUDE_DIR})
add_executable(poly2ogr poly2ogr.cxx)

target_link_libraries(poly2ogr
    ${GDAL_LIBRARY}
    Polygon
    ${SIMGEAR_CORE_LIBRARIES}
    ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

install(TARGETS poly2ogr RUNTIME DESTINATION bin)
endif(GDAL_FOUND)

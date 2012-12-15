include_directories(${GDAL_INCLUDE_DIR})
add_executable(ogr-decode ogr-decode.cxx)

target_link_libraries(ogr-decode 
    ${GDAL_LIBRARY}
    Polygon
    ${SIMGEAR_CORE_LIBRARIES}
    ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

install(TARGETS ogr-decode RUNTIME DESTINATION bin)


add_executable(tguserdef tguserdef.cxx)

target_link_libraries(tguserdef
    Polygon Geometry Output poly2tri
    ${GDAL_LIBRARY}
    ${SIMGEAR_CORE_LIBRARIES}
    ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

install(TARGETS tguserdef RUNTIME DESTINATION bin)

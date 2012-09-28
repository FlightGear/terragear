
add_executable(e00lines 
    main.cxx)

target_link_libraries(e00lines 
    e00
    Polygon Geometry Output poly2tri vpf
    ${GDAL_LIBRARY}
    ${SIMGEAR_CORE_LIBRARIES}
    ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

install(TARGETS e00lines RUNTIME DESTINATION bin)

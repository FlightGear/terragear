add_executable(tg-construct
    construct.cxx 
    construct.hxx
    priorities.cxx
    priorities.hxx
    usgs.cxx 
    main.cxx)

set_target_properties(tg-construct PROPERTIES
        COMPILE_DEFINITIONS 
        "DEFAULT_USGS_MAPFILE=\"${PKGDATADIR}/usgsmap.txt\";DEFAULT_PRIORITIES_FILE=\"${PKGDATADIR}/default_priorities.txt\"" )

target_link_libraries(tg-construct
    Osgb36  
    Match
    Polygon Geometry
    Array landcover poly2tri
    TriangleJRS
    ${GDAL_LIBRARY}
    ${SIMGEAR_CORE_LIBRARIES}
    ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

install(TARGETS tg-construct RUNTIME DESTINATION bin)

INSTALL(FILES usgsmap.txt DESTINATION ${PKGDATADIR} )
INSTALL(FILES default_priorities.txt DESTINATION ${PKGDATADIR} )

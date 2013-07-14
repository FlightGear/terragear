include_directories(${GDAL_INCLUDE_DIR})

add_executable(tg-construct
    tgconstruct.hxx
    tgconstruct.cxx
    tgconstruct_cleanup.cxx
    tgconstruct_clip.cxx
    tgconstruct_debug.cxx
    tgconstruct_elevation.cxx
    tgconstruct_landclass.cxx
    tgconstruct_lookup.cxx
    tgconstruct_math.cxx
    tgconstruct_output.cxx
    tgconstruct_poly.cxx
    tgconstruct_shared.cxx
    tgconstruct_tesselate.cxx
    tgconstruct_texture.cxx
    priorities.cxx
    priorities.hxx
    usgs.cxx 
    main.cxx)

set_target_properties(tg-construct PROPERTIES
        COMPILE_DEFINITIONS 
        "DEFAULT_USGS_MAPFILE=\"${PKGDATADIR}/usgsmap.txt\";DEFAULT_PRIORITIES_FILE=\"${PKGDATADIR}/default_priorities.txt\"" )

target_link_libraries(tg-construct
    terragear
    landcover
    ${Boost_LIBRARIES}
    ${GDAL_LIBRARY}
    ${ZLIB_LIBRARY}
    ${SIMGEAR_CORE_LIBRARIES}
    ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

install(TARGETS tg-construct RUNTIME DESTINATION bin)

INSTALL(FILES usgsmap.txt DESTINATION ${PKGDATADIR} )
INSTALL(FILES default_priorities.txt DESTINATION ${PKGDATADIR} )

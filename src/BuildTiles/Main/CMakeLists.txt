include_directories(${GDAL_INCLUDE_DIR})

add_executable(tg-construct
    tgconstruct_stage1.hxx
    tgconstruct_stage1.cxx
    tgconstruct_stage2.hxx
    tgconstruct_stage2.cxx
    tgconstruct_stage3.hxx
    tgconstruct_stage3.cxx    
    priorities.cxx
    priorities.hxx
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

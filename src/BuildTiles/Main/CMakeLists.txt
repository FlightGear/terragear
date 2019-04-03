include_directories(${GDAL_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/src/Lib/terragear)

add_executable(tg-construct
    tgconstruct.hxx
    tgconstruct.cxx
    tgconstruct_cleanup.cxx
    tgconstruct_clip.cxx
    tgconstruct_debug.cxx
    tgconstruct_elevation.cxx
    tgconstruct_lookup.cxx
    tgconstruct_math.cxx
    tgconstruct_output.cxx
    tgconstruct_poly.cxx
    tgconstruct_shared.cxx
    tgconstruct_tesselate.cxx
    tgconstruct_texture.cxx
    tglandclass.cxx
    tglandclass.hxx
    priorities.cxx
    priorities.hxx
    main.cxx)

set_target_properties(tg-construct PROPERTIES
        COMPILE_DEFINITIONS 
        "DEFAULT_PRIORITIES_FILE=\"${PKGDATADIR}/default_priorities.txt\"" )

target_link_libraries(tg-construct
    terragear
    Array landcover
    ${Boost_LIBRARIES}
    ${GDAL_LIBRARY}
    ${ZLIB_LIBRARY}
    ${SIMGEAR_CORE_LIBRARIES}
    ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

install(TARGETS tg-construct RUNTIME DESTINATION bin)

INSTALL(FILES default_priorities.txt DESTINATION ${PKGDATADIR} )

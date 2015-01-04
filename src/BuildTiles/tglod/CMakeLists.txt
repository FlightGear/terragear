include_directories(${GDAL_INCLUDE_DIR})

add_executable(tg-lod
    main.cxx
    tg_btg_mesh.hxx
    tg_btg_mesh.cxx
    tg_btg_mesh_simplify.cxx)

target_link_libraries(tg-lod
    terragear
    ${Boost_LIBRARIES}
    ${GDAL_LIBRARY}    
    ${ZLIB_LIBRARY}
    ${SIMGEAR_CORE_LIBRARIES}
    ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

install(TARGETS tg-lod RUNTIME DESTINATION bin)
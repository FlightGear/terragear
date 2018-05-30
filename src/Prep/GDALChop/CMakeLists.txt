include_directories(${GDAL_INCLUDE_DIR})
add_executable(gdalchop gdalchop.cxx)

target_link_libraries(gdalchop
        terragear ${GDAL_LIBRARY}
        ${ZLIB_LIBRARY}
        ${SIMGEAR_CORE_LIBRARIES}
        ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)
install(TARGETS gdalchop RUNTIME DESTINATION bin)
if (MSVC)
    set_target_properties( gdalchop PROPERTIES DEBUG_POSTFIX d )
endif ()

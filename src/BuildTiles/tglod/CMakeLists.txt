include_directories(${GDAL_INCLUDE_DIR})

add_executable(tg-lod
    main.cxx)

target_link_libraries(tg-lod
    terragear
    ${Boost_LIBRARIES}
    ${ZLIB_LIBRARY}
    ${SIMGEAR_CORE_LIBRARIES}
    ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

install(TARGETS tg-lod RUNTIME DESTINATION bin)
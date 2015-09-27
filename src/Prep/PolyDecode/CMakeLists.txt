include_directories(${GDAL_INCLUDE_DIR})
add_executable(poly-decode poly-decode.cxx)

target_link_libraries(poly-decode 
    ${GDAL_LIBRARY}
    terragear
    ${ZLIB_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT}
    ${SIMGEAR_CORE_LIBRARIES}
    ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

install(TARGETS poly-decode RUNTIME DESTINATION bin)

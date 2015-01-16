include_directories(${GDAL_INCLUDE_DIR})
add_executable(vector-decode vector-decode.cxx)

target_link_libraries(vector-decode 
    ${GDAL_LIBRARY}
    terragear
    ${ZLIB_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT}
    ${SIMGEAR_CORE_LIBRARIES}
    ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

install(TARGETS vector-decode RUNTIME DESTINATION bin)

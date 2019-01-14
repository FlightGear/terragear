include_directories(${GDAL_INCLUDE_DIR})
add_executable(cliff-decode cliff-decode.cxx)

target_link_libraries(cliff-decode 
    ${GDAL_LIBRARY}
    terragear
    ${ZLIB_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT}
    ${SIMGEAR_CORE_LIBRARIES}
    ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
    ${Boost_LIBRARIES}
)

install(TARGETS cliff-decode RUNTIME DESTINATION bin)

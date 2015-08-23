include_directories(${GDAL_INCLUDE_DIR})

add_executable(cgalcliptest
    cliptest.cxx
)

target_link_libraries(cgalcliptest
    terragear
    ${Boost_LIBRARIES}
    ${GDAL_LIBRARY}    
    ${ZLIB_LIBRARY}
    ${SIMGEAR_CORE_LIBRARIES}
    ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

add_executable(cgaltritest
    tritest.cxx
)

target_link_libraries(cgaltritest
    terragear
    ${Boost_LIBRARIES}
    ${GDAL_LIBRARY}    
    ${ZLIB_LIBRARY}
    ${SIMGEAR_CORE_LIBRARIES}
    ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

install(TARGETS cgalcliptest RUNTIME DESTINATION bin)
install(TARGETS cgaltritest RUNTIME DESTINATION bin)

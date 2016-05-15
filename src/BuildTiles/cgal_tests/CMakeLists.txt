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

add_executable(cgaljointest
    jointest.cxx
)

target_link_libraries(cgaljointest
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

add_executable(cgalarrtest
    arrtest.cxx
)

target_link_libraries(cgalarrtest
    terragear
    ${Boost_LIBRARIES}
    ${GDAL_LIBRARY}    
    ${ZLIB_LIBRARY}
    ${SIMGEAR_CORE_LIBRARIES}
    ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

add_executable(extended_kernel
    extended_kernel.cxx
)

target_link_libraries(extended_kernel
    terragear
    ${Boost_LIBRARIES}
    ${GDAL_LIBRARY}    
    ${ZLIB_LIBRARY}
    ${SIMGEAR_CORE_LIBRARIES}
    ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

install(TARGETS cgalcliptest RUNTIME DESTINATION bin)
install(TARGETS cgaljointest RUNTIME DESTINATION bin)
install(TARGETS cgaltritest RUNTIME DESTINATION bin)
install(TARGETS cgalarrtest RUNTIME DESTINATION bin)

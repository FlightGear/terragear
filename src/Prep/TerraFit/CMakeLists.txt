
add_executable(terrafit terrafit.cc)

include_directories(${PROJECT_SOURCE_DIR}/src/Lib/terragear)

target_link_libraries(terrafit 
    Array Terra
    terragear
    ${ZLIB_LIBRARY}
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})
	
install(TARGETS terrafit RUNTIME DESTINATION bin)

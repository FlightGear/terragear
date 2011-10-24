
add_executable(deminfo deminfo.cxx)

target_link_libraries(deminfo
    DEM
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})

install(TARGETS deminfo RUNTIME DESTINATION bin)



add_executable(fgfs-tools-server
    server.cxx)

target_link_libraries(fgfs-tools-server
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

install(TARGETS fgfs-tools-server RUNTIME DESTINATION bin)


add_executable(fgfs-tools-client
     client.cxx)

target_link_libraries(fgfs-tools-client
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
	)

install(TARGETS fgfs-tools-client RUNTIME DESTINATION bin)

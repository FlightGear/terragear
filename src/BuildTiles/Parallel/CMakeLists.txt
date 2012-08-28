add_executable(tg-construct-server
    server.cxx)

target_link_libraries(tg-construct-server
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

install(TARGETS tg-construct-server RUNTIME DESTINATION bin)


add_executable(tg-construct-client
     client.cxx)

target_link_libraries(tg-construct-client
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
	)

install(TARGETS tg-construct-client RUNTIME DESTINATION bin)

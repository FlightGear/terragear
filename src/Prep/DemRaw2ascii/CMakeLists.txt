
add_executable(raw2ascii
    main.cxx rawdem.c rawdem.h)

target_link_libraries(raw2ascii
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
)

install(TARGETS raw2ascii RUNTIME DESTINATION bin)


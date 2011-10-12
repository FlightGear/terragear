

add_library(e00 STATIC 
    e00.hxx e00.cxx
)

add_executable(teste00 teste00.cxx)

target_link_libraries(teste00 
    e00
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})

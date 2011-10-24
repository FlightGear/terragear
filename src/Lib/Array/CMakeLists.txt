
add_library(Array STATIC 
    array.cxx array.hxx
)

add_executable(test_array testarray.cxx)

target_link_libraries(test_array 
    Array
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})

install(TARGETS test_array RUNTIME DESTINATION bin)

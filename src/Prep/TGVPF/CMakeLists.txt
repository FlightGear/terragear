
add_executable(tgvpf tgvpf.cxx)

target_link_libraries(tgvpf 
    Polygon Geometry Output poly2tri TriangleJRS vpf
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
	${GPC_LIBRARY})
	
install(TARGETS tgvpf RUNTIME DESTINATION bin)

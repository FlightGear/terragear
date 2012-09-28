
add_executable(tgvpf tgvpf.cxx)

target_link_libraries(tgvpf 
    Polygon Geometry Output poly2tri vpf
    ${GDAL_LIBRARY}
    ${SIMGEAR_CORE_LIBRARIES}
    ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
    ${RT_LIBRARY})

install(TARGETS tgvpf RUNTIME DESTINATION bin)

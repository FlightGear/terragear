

add_library(HGT STATIC 
    hgt.cxx hgt.hxx
    srtmbase.cxx srtmbase.hxx
)

add_executable(test_hgt testhgt.cxx)

target_link_libraries(test_hgt 
    HGT
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES})

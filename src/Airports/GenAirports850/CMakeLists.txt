

add_executable(genapts850
    apt_surface.hxx apt_surface.cxx 
	build.cxx build.hxx 
	elevations.cxx elevations.hxx 
	global.hxx
        heli_gen.cxx heli_gen.hxx
	lights.hxx lights.cxx 
	main.cxx 
	point2d.cxx point2d.hxx 
	poly_extra.cxx poly_extra.hxx 
	runway.cxx runway.hxx 
	rwy_common.cxx rwy_common.hxx 
	rwy_gen.cxx rwy_gen.hxx
	rwy_simple.cxx rwy_simple.hxx 
	taxiway.cxx taxiway.hxx 
	texparams.hxx
	)
  
target_link_libraries(genapts850
    Polygon Geometry
    Array Optimize Output poly2tri
    TriangleJRS
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
	${GPC_LIBRARY}
	${NEWMAT_LIBRARY})

install(TARGETS genapts850 RUNTIME DESTINATION bin)



add_executable(genapts
    apt_surface.hxx apt_surface.cxx 
	build.cxx build.hxx 
	convex_hull.cxx convex_hull.hxx 
	elevations.cxx elevations.hxx 
	global.hxx 
	lights.hxx lights.cxx 
	main.cxx 
	point2d.cxx point2d.hxx  
	runway.cxx runway.hxx 
	rwy_common.cxx rwy_common.hxx 
	rwy_nonprec.cxx rwy_nonprec.hxx 
	rwy_prec.cxx rwy_prec.hxx 
	rwy_simple.cxx rwy_simple.hxx 
	rwy_visual.cxx rwy_visual.hxx 
	taxiway.cxx taxiway.hxx 
	)
  
target_link_libraries(genapts
    Polygon Geometry
    Array Optimize Output poly2tri
    TriangleJRS
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
    ${GDAL_LIBRARY}
	${GPC_LIBRARY}
	${NEWMAT_LIBRARY})

install(TARGETS genapts RUNTIME DESTINATION bin)

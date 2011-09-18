add_executable(genapts850
	airport.hxx airport.cxx
	apt_surface.hxx apt_surface.cxx
	beznode.hxx
	closedpoly.hxx closedpoly.cxx
	convex_hull.hxx convex_hull.cxx
	elevations.cxx elevations.hxx 
	global.hxx
	linearfeature.hxx linearfeature.cxx
	main.cxx
	parser.hxx parser.cxx
	point2d.cxx point2d.hxx 
	poly_extra.cxx poly_extra.hxx 
	runway.cxx runway.hxx
	texparams.hxx
	)
  
target_link_libraries(genapts850
    Polygon Geometry
    Array Optimize Output poly2tri
    TriangleJRS
	${SIMGEAR_CORE_LIBRARIES}
	${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
	${GPC_LIBRARY}
	${NEWMAT_LIBRARY}
        osg osgDB osgGA osgUtil osgViewer)

install(TARGETS genapts850 RUNTIME DESTINATION bin)

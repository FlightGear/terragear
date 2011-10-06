add_executable(genapts850
	airport.hxx airport.cxx
	apt_math.hxx apt_math.cxx
	apt_surface.hxx apt_surface.cxx
	beznode.hxx
	closedpoly.hxx closedpoly.cxx
	convex_hull.hxx convex_hull.cxx
	elevations.cxx elevations.hxx 
	global.hxx
	lights.cxx
	linearfeature.hxx linearfeature.cxx
	linked_objects.hxx linked_objects.cxx
	main.cxx
	object.hxx object.cxx
	parser.hxx parser.cxx
	point2d.cxx point2d.hxx 
	poly_extra.cxx poly_extra.hxx 
	runway.cxx runway.hxx
	rwy_simple.cxx
	rwy_gen.cxx
	rwy_common.cxx
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

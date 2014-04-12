include_directories(${GDAL_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/src/Lib)

add_executable(genapts850
    airport.hxx airport.cxx
    airport_base.cxx
    airport_features.cxx
    airport_lights.cxx
    apt_math.hxx apt_math.cxx
    beznode.hxx
    closedpoly.hxx closedpoly.cxx
    debug.hxx debug.cxx
    elevations.cxx elevations.hxx
    global.hxx
    helipad.hxx helipad.cxx
    lights.cxx
    linearfeature.hxx linearfeature.cxx
    linked_objects.hxx linked_objects.cxx
    main.cxx
    object.hxx object.cxx
    output.hxx output.cxx
    parser.hxx parser.cxx
    runway.cxx runway.hxx
    runway_simple.cxx
    runway_full.cxx
    scheduler.cxx scheduler.hxx
    taxiway.cxx taxiway.hxx
)

target_link_libraries(genapts850
    terragear
    ${GDAL_LIBRARY}
    ${ZLIB_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT}
    ${SIMGEAR_CORE_LIBRARIES}
    ${SIMGEAR_CORE_LIBRARY_DEPENDENCIES}
    ${RT_LIBRARY})

install(TARGETS genapts850 RUNTIME DESTINATION bin)

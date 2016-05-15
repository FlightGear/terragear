include_directories(${GDAL_INCLUDE_DIR})

include( ${CGAL_USE_FILE} )

add_subdirectory(polygon_set)
add_subdirectory(vector_intersections)
add_subdirectory(mesh)

# temp - root director may no longer exist
include (TerragearComponent)

# add more components - for now - just a root
set(HEADERS 
    clipper.hpp
    tg_areas.hxx
    tg_arrangement.hxx
    tg_array.hxx
    tg_cgal.hxx
    tg_cgal_epec.hxx
    tg_cluster.hxx
    tg_contour.hxx
    tg_dataset_protect.hxx
    tg_light.hxx
    tg_misc.hxx
    tg_mutex.hxx
    tg_nodes.hxx
    tg_polygon.hxx
    tg_rectangle.hxx
    tg_shapefile.hxx
    tg_surface.hxx
    tg_triangle.hxx
    tg_unique_geod.hxx
    tg_unique_tgnode.hxx
    tg_unique_vec2f.hxx
    tg_unique_vec3d.hxx
    tg_unique_vec3f.hxx
)

set(SOURCES 
    clipper.cpp
    tg_areas.cxx
    tg_arrangement.cxx
    tg_array.cxx
    tg_cgal.cxx
    tg_cluster.cxx
    tg_contour.cxx
    tg_misc.cxx
    tg_nodes.cxx
    tg_polygon.cxx
    tg_polygon_clean.cxx
    tg_polygon_clip.cxx
    tg_polygon_tesselate.cxx
    tg_rectangle.cxx
    tg_shapefile.cxx
    tg_sskel.cxx
    tg_surface.cxx
)

terragear_component(root ./ "${SOURCES}" "${HEADERS}")

get_property(coreSources  GLOBAL PROPERTY CORE_SOURCES)
get_property(includePaths GLOBAL PROPERTY TGLIB_INCLUDE_DIRS)

message(STATUS "coreSources is ${coreSources}")
message(STATUS "includePaths is ${includePaths}")

# 	get_property(FG_GROUPS_CORE_SOURCES_H GLOBAL PROPERTY FG_GROUPS_CORE_SOURCES_H)
#	string(REPLACE "@" ";" groups ${FG_GROUPS_CORE_SOURCES_H} )
#	foreach(g ${groups})
#		string(REPLACE "#" ";" g2 ${g})
#		list(GET g2 0 name)
#		list(REMOVE_AT g2 0)
#		source_group("${name}\\Headers" FILES ${g2})
#	endforeach()

    foreach(p ${includePaths})
	message(STATUS " add include dir ${p}")
        include_directories(${p})
    endforeach()

add_library(terragear STATIC ${coreSources} ) 

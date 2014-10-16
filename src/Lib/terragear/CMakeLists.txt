include_directories(${GDAL_INCLUDE_DIR})

include( ${CGAL_USE_FILE} )

add_library(terragear STATIC
    clipper.cpp
    clipper.hpp
    tg_accumulator.cxx
    tg_accumulator.hxx
    tg_areas.cxx
    tg_areas.hxx
    tg_arrangement.cxx
    tg_arrangement.hxx
    tg_array.cxx
    tg_array.hxx
    tg_cgal.cxx
    tg_cgal.hxx
    tg_chopper.cxx
    tg_chopper.hxx
    tg_contour.cxx
    tg_contour.hxx
    tg_graph.cxx
    tg_graph.hxx
    tg_light.hxx
    tg_misc.cxx
    tg_misc.hxx
    tg_nodes.cxx
    tg_nodes.hxx
    tg_polygon.cxx
    tg_polygon.hxx
    tg_polygon_clean.cxx
    tg_polygon_clip.cxx
    tg_polygon_tesselate.cxx
    tg_rectangle.cxx
    tg_rectangle.hxx
    tg_shapefile.cxx
    tg_shapefile.hxx
    tg_sskel.cxx
    tg_surface.cxx
    tg_surface.hxx
    tg_triangle.hxx
    tg_unique_geod.hxx
    tg_unique_tgnode.hxx
    tg_unique_vec2f.hxx
    tg_unique_vec3d.hxx
    tg_unique_vec3f.hxx
)
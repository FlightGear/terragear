include_directories(${GDAL_INCLUDE_DIR})

include( ${CGAL_USE_FILE} )

add_library(Polygon STATIC 
    clipper.cpp
    clipper.hpp
    polygon.cxx
    polygon.hxx
    rectangle.cxx
    rectangle.hxx
    surface.cxx
    surface.hxx
    tg_nodes.cxx
    tg_nodes.hxx
    tg_unique_geod.hxx
    tg_unique_tgnode.hxx
    tg_unique_vec2f.hxx
    tg_unique_vec3d.hxx
    tg_unique_vec3f.hxx
)
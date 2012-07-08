if (GDAL_FOUND)
include_directories(${GDAL_INCLUDE_DIR})
endif (GDAL_FOUND)

add_library(Geometry STATIC 
    contour_tree.cxx
    line.cxx
    poly_support.cxx
    poly_extra.cxx
    rectangle.cxx
    tg_nodes.cxx
    trinodes.cxx
    trisegs.cxx
    util.cxx
    contour_tree.hxx
    line.hxx
    point3d.hxx
    poly_support.hxx
    poly_extra.hxx
    rectangle.hxx
    tg_nodes.hxx
    trinodes.hxx
    trisegs.hxx
    util.hxx
)

include_directories(${GDAL_INCLUDE_DIR})

include( ${CGAL_USE_FILE} )

add_library(Polygon STATIC 
    chop-bin.cxx
    index.cxx
    polygon.cxx
    simple_clip.cxx
    superpoly.cxx
    chop.hxx
    index.hxx
    names.hxx
    polygon.hxx
    simple_clip.hxx
    superpoly.hxx
    texparams.cxx
    texparams.hxx
    clipper.cpp
    clipper.hpp
)

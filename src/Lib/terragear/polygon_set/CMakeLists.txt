
include (TerragearComponent)

set(HEADERS 
    tg_polygon_set.hxx
    tg_polygon_set_paths.hxx
    tg_polygon_def.hxx

    tg_polygon_accumulator.hxx
    tg_polygon_chop.hxx
)

set(SOURCES 
    tg_polygon_set.cxx
    tg_polygon_set_io.cxx
    tg_polygon_set_io_debug.cxx
    tg_polygon_set_meta.cxx
    tg_polygon_set_offset_clipper.cxx
    tg_polygon_set_paths.cxx
    tg_polygon_set_sle.cxx

    tg_polygon_accumulator.cxx
    tg_polygon_chop.cxx
)

terragear_component(polygon_set polygon_set "${SOURCES}" "${HEADERS}")

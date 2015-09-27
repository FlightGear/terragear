
include (TerragearComponent)

set(HEADERS 
    tg_constraint.hxx
    tg_intersection_generator.hxx
    tg_intersection_edge.hxx
    tg_intersection_node.hxx
    tg_segmentnetwork.hxx
)

set(SOURCES 
    tg_constraint.cxx
    tg_intersection_generator.cxx
    tg_intersection_edge.cxx
    tg_intersection_node.cxx
    tg_segmentnetwork.cxx
    tg_segmentnetwork_ars.cxx
)

terragear_component(vector_intersections vector_intersections "${SOURCES}" "${HEADERS}")

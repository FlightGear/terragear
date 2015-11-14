
include (TerragearComponent)

set(HEADERS 
    tg_mesh_def.hxx
    tg_mesh.hxx
)

set(SOURCES 
    tg_mesh.cxx
    tg_mesh_arrangement.cxx
    tg_mesh_triangulation.cxx
    tg_mesh_io.cxx
)

terragear_component(mesh mesh "${SOURCES}" "${HEADERS}")

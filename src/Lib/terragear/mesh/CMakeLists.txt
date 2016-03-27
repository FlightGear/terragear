
include (TerragearComponent)

set(HEADERS 
    tg_mesh_def.hxx
    tg_mesh.hxx
)

set(SOURCES 
    tg_mesh.cxx
    tg_mesh_arrangement.cxx
    tg_mesh_arrangement_cleaning.cxx
    tg_mesh_arrangement_io.cxx
    tg_mesh_polyhedral_surface.hxx
    tg_mesh_polyhedral_surface.cxx
    tg_mesh_triangulation.cxx
    tg_mesh_triangulation_io.cxx
    tg_mesh_triangulation_shared_edges.cxx
    tg_mesh_io.cxx
)

terragear_component(mesh mesh "${SOURCES}" "${HEADERS}")

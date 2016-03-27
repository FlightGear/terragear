#include <simgear/debug/logstream.hxx>

#include "tg_mesh.hxx"

#define DEBUG_MESH_SURFACE                  (1)     // Generate intermediate shapefiles during surface parameterization

void tgMeshPolyhedralSurface::loadTriangulation( const std::string& basePath, const SGBucket& bucket )
{
    std::string bucketPath = basePath + "/" + bucket.gen_base_path() + "/" + bucket.gen_index_str();
    
    std::vector<meshTriPoint>   points;
    std::string filePath;
    
    filePath = bucketPath + "/stage1_triangles_points.shp"; 
    //fromShapefile( filePath, points );
    
    // use incremental builder to load the triangulation, and the neighboring facets
    
}
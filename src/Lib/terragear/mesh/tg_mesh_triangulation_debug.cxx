#include <simgear/debug/logstream.hxx>

#include "tg_mesh.hxx"

void tgMeshTriangulation::writeCdtFile( const char* filename, std::vector<meshTriPoint>& points,  std::vector<meshTriSegment>& constraints ) const
{
    std::ofstream output_file(filename);
    if (!output_file.is_open()) {
        std::cerr << "Failed to open " << filename << std::endl;
    } else {
        output_file << std::setprecision(64);

        output_file << points.size() << "\n";
        for ( unsigned int i = 0; i < points.size(); i++ ) {
            output_file << points[i] << "\n";
        }
    }
    output_file << "\n";

    output_file << constraints.size() << "\n";
    for ( unsigned int i = 0; i < constraints.size(); i++ ) {        
        output_file << constraints[i].source() << "\t" << constraints[i].target() << "\n";            
    }
    output_file.close();
}

void tgMeshTriangulation::writeCdtFile2( const char* filename, const meshTriCDT& cdt) const
{
    std::ofstream output_file(filename);
    if (!output_file.is_open()) {
        std::cerr << "Failed to open " << filename << std::endl;
        exit(0);
    }

    output_file << std::setprecision(64);
    output_file << cdt;
    output_file.close();    
}

void tgMeshTriangulation::saveCDTAscii( const std::string& datasource, const char* file ) const
{
    std::string filename = datasource + "/" + file;
    std::ofstream output_file(filename.c_str());

    CGAL::set_ascii_mode(output_file);
    output_file << std::setprecision(64);
    output_file << meshTriangulation;
    output_file.close();
}

void tgMeshTriangulation::saveTdsAscii( const std::string& datasource, const char* layer ) const
{
    std::string filename = datasource + "/" + layer + "_tds.txt";
    std::ofstream output_file(filename.c_str());
    output_file << std::setprecision(64);

    output_file << vertexInfo.size() << " " << faceInfo.size() << " " << "2" << std::endl;

    // don't save first point : it's infinite vertex
    for (unsigned int i=1; i<vertexInfo.size(); i++) {
        output_file << vertexInfo[i].getPoint() << std::endl;
    }
    output_file << std::endl;

    char indices[32];
    for (unsigned int i=0; i<faceInfo.size(); i++) {
        faceInfo[i].getVidxField2( indices );
        output_file << indices << std::endl;
    }
    output_file << std::endl;

    for (unsigned int i=0; i<faceInfo.size(); i++) {                
        faceInfo[i].getNIdxField2( indices );
        output_file << indices << std::endl;
    }

    for (unsigned int i=0; i<faceInfo.size(); i++) {                
        faceInfo[i].getConstrainedField2( indices );
        output_file << indices << std::endl;
    }

    output_file.close();
}

void tgMeshTriangulation::saveFlippable( const char* layer ) const
{
    // open debug layer
    GDALDataset*  poDS = NULL;
    OGRLayer*     poEdgeLayer = NULL;

    poDS = mesh->openDatasource( mesh->getDebugPath() );
    if ( poDS ) {
        poEdgeLayer = mesh->openLayer( poDS, wkbLineString25D, tgMesh::LAYER_FIELDS_NONE, layer );
    }

    meshTriCDT::Finite_faces_iterator fit2 = meshTriangulation.finite_faces_begin();
    char desc[64];
    int idx = 0;
    for (; fit2 != meshTriangulation.finite_faces_end(); fit2++, idx++) {
        for(int i=0;i<3;i++) {
            if ( meshTriangulation.is_flipable( fit2, i, false) ) {
                // edge is flippable - should not be the case
                // create a segment from the two vertexes
                meshTriPoint s = fit2->vertex( meshTriangulation.cw(i) )->point();
                meshTriPoint t = fit2->vertex( meshTriangulation.ccw(i) )->point();
                meshTriSegment seg( s, t );

                sprintf( desc, "face_%d_edge_%d", idx, i );
                toShapefile( poEdgeLayer, seg, desc );

                SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - face " << idx << " edge " << i << " flippable!" );
            }
        }
    }

    // close datasource
    GDALClose( poDS );
}

void tgMeshTriangulation::saveConstrained( const char* layer ) const
{
    // open debug layer
    GDALDataset*  poDS = NULL;
    OGRLayer*     poEdgeLayer = NULL;

    poDS = mesh->openDatasource( mesh->getDebugPath() );
    if ( poDS ) {
        poEdgeLayer = mesh->openLayer( poDS, wkbLineString25D, tgMesh::LAYER_FIELDS_NONE, layer );
    }

    meshTriCDT::All_faces_iterator fit2 = meshTriangulation.all_faces_begin();
    char desc[64];
    int idx = 0;
    for (; fit2 != meshTriangulation.all_faces_end(); fit2++, idx++) {
        for(int i=0;i<3;i++) {
            meshTriEdge edge( fit2, i );
            if ( meshTriangulation.is_constrained(edge) ) {
                // edge is constrained
                // create a segment from the two vertexes
                meshTriPoint s = fit2->vertex( meshTriangulation.cw(i) )->point();
                meshTriPoint t = fit2->vertex( meshTriangulation.ccw(i) )->point();
                meshTriSegment seg( s, t );

                sprintf( desc, "face_%d_edge_%d", idx, i );
                toShapefile( poEdgeLayer, seg, desc );
            }
        }
    }

    // close datasource
    GDALClose( poDS );
}

void tgMeshTriangulation::saveEdgeBoundingBox( const meshTriPoint& lr, const meshTriPoint& ll, const meshTriPoint& ul, const meshTriPoint& ur, const char* name ) const
{
    GDALDataset*  poDS = NULL;
    OGRLayer*     poLayer    = NULL;

    poDS = mesh->openDatasource( mesh->getDebugPath() );
    if ( poDS ) {
        poLayer = mesh->openLayer( poDS, wkbLineString25D, tgMesh::LAYER_FIELDS_NONE, name );

        SG_LOG(SG_GENERAL, SG_DEBUG, " Opened datasource " << mesh->getDebugPath() );

        meshTriSegment north( ul, ur );
        toShapefile( poLayer, north, "north" );

        meshTriSegment south( ll, lr );
        toShapefile( poLayer, south, "south" );

        meshTriSegment east( ur, lr );
        toShapefile( poLayer, east, "east" );

        meshTriSegment west( ul, ll );
        toShapefile( poLayer, west, "west" );

        GDALClose( poDS );
    }
}
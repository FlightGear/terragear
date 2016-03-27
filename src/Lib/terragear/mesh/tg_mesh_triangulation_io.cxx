#include <simgear/debug/logstream.hxx>

#include <CGAL/Bbox_2.h>

#include "tg_mesh.hxx"


// Save a single meshTriPoint
void tgMeshTriangulation::toShapefile( OGRLayer* poLayer, const meshTriPoint& pt, const char* desc ) const
{    
    OGRPoint      point;
    
    point.setZ( 0.0 );    
    point.setX( pt.x() );
    point.setY( pt.y() );
    
    OGRFeature* poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
    poFeature->SetGeometry(&point);    
    poFeature->SetField("tg_desc", desc );
    
    if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(poFeature);        
}

#if 0
// Save a single meshTriPoint with elevation
void tgMeshTriangulation::toShapefile( OGRLayer* poLayer, const meshTriPoint& pt, double elv, unsigned int idx ) const
{    
    OGRPoint      point;
    
    point.setX( pt.x() );
    point.setY( pt.y() );
    point.setZ( elv );
    
    OGRFeature* poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
    poFeature->SetGeometry(&point);    
    poFeature->SetField("tg_vert", (int)idx);
    
    if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(poFeature);        
}
#endif

// Save a vector of meshTriPoints ( for shared edges )
#if 0
void tgMeshTriangulation::toShapefile( const std::string& datasource, const char* layer, const std::vector<meshTriPoint>& points ) const
{
    GDALDataset*  poDS = NULL;
    OGRLayer*     poPointLayer = NULL;
    
    poDS = mesh->openDatasource( datasource );
    if ( poDS ) {
        poPointLayer = mesh->openLayer( poDS, wkbPoint25D, tgMesh::LAYER_FIELDS_NONE, layer );
        
        if ( poPointLayer ) {            
            for ( unsigned int i=0; i < points.size(); i++ ) {
                toShapefile( poPointLayer, points[i], "pt" );
            }
        }
    }
    
    // close datasource
    GDALClose( poDS );        
}
#endif

// Load a single vector info from a shapefile, and push into given vector
void tgMeshTriangulation::fromShapefile( const OGRFeatureDefn* poFDefn, OGRCoordinateTransformation* poCT, OGRFeature* poFeature, std::vector<meshVertexInfo>& points ) const
{
    OGRGeometry *poGeometry = poFeature->GetGeometryRef();
    if (poGeometry == NULL) {
        SG_LOG( SG_GENERAL, SG_INFO, "Found feature without geometry!" );
        return;
    }
    
    OGRwkbGeometryType geoType = wkbFlatten(poGeometry->getGeometryType());
    if (geoType != wkbPoint) {
        SG_LOG( SG_GENERAL, SG_INFO, "Unknown feature " << geoType );
        return;
    }
    
    // now the geometry(s)
    poGeometry->transform( poCT );

    int index = 0;
    int fieldIdx = poFeature->GetFieldIndex( "tg_vert" );    
    if ( fieldIdx >= 0 ) {
        index = poFeature->GetFieldAsInteger(fieldIdx);
    }

    switch( geoType ) {
        case wkbPoint:
        {
            const OGRPoint* ogr_point = (const OGRPoint*)poGeometry;
            points.push_back( meshVertexInfo( index, ogr_point ) );
            break;
        }
        
        default:
            break;
    }
    
    return;
}

// Load a single meshFaceInfo from a shapefile, and push into given vector
void tgMeshTriangulation::fromShapefile( const OGRFeatureDefn* poFDefn, OGRCoordinateTransformation* poCT, OGRFeature* poFeature, std::vector<meshFaceInfo>& faces ) const
{
    OGRGeometry *poGeometry = poFeature->GetGeometryRef();
    if (poGeometry == NULL) {
        SG_LOG( SG_GENERAL, SG_INFO, "Found feature without geometry!" );
        return;
    }
    
    OGRwkbGeometryType geoType = wkbFlatten(poGeometry->getGeometryType());
    if ( geoType != wkbLineString ) {
        SG_LOG( SG_GENERAL, SG_INFO, "Unknown feature " << geoType );
        return;
    }
    
    
#if 0
poFeature->SetField("tds_cons", constrained );

sprintf(indices, "%08d,%08d,%08d", 
V[tri->vertex(0)], 
V[tri->vertex(1)],
V[tri->vertex(2)]
);
poFeature->SetField("tds_vidx", indices );

sprintf(indices, "%08d,%08d,%08d", 
F[tri->neighbor(0)], 
F[tri->neighbor(1)],
F[tri->neighbor(2)]
);
poFeature->SetField("tds_nidx", indices );
#endif
    
    int  fieldIdx;
    char valueStr[128];
    int  fid = 0;
    int  vIdx[3];
    int  nIdx[3];
    int  cons[3];
    
    switch( geoType ) {
        case wkbLineString:
        {
            // we don't need the geometry - just the feature fields...
            fieldIdx = poFeature->GetFieldIndex( "tds_fid" );
            if ( fieldIdx >= 0 ) {
                fid = poFeature->GetFieldAsInteger(fieldIdx);
            }
            
            fieldIdx = poFeature->GetFieldIndex( "tds_vidx" );
            if ( fieldIdx >= 0 ) {
                strncpy( valueStr, poFeature->GetFieldAsString(fieldIdx), 128 );
                sscanf( valueStr, "%08d,%08d,%08d", &vIdx[0], &vIdx[1], &vIdx[2] );
            } else {
                SG_LOG( SG_GENERAL, SG_INFO, "tds_vidx NOT SET" );
            }

            fieldIdx = poFeature->GetFieldIndex( "tds_nidx" );
            if ( fieldIdx >= 0 ) {
                strncpy( valueStr, poFeature->GetFieldAsString(fieldIdx), 128 );
                sscanf( valueStr, "%08d,%08d,%08d",  &nIdx[0], &nIdx[1], &nIdx[2] );
            } else {
                SG_LOG( SG_GENERAL, SG_INFO, "tds_nidx NOT SET" );
            }
            
            fieldIdx = poFeature->GetFieldIndex( "tds_cons" );
            if ( fieldIdx >= 0 ) {
                strncpy( valueStr, poFeature->GetFieldAsString(fieldIdx), 128 );
                sscanf( valueStr, "%01d,%01d,%01d",   &cons[0], &cons[1], &cons[2] );
            } else {
                SG_LOG( SG_GENERAL, SG_INFO, "tds_nidx NOT SET" );
            }
         
            faces.push_back( meshFaceInfo(fid, vIdx, nIdx, cons) );
            break;
        }
        
        default:
            break;
    }
    
    return;
}

// load all meshTriPoints from all layers of a shapefile
void tgMeshTriangulation::fromShapefile( const std::string& filename, std::vector<meshVertexInfo>& points ) const
{
    GDALAllRegister();
    
    GDALDataset* poDS = (GDALDataset*)GDALOpenEx( filename.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
    if( poDS == NULL )
    {
        SG_LOG( SG_GENERAL, SG_DEBUG, "Failed opening datasource " << filename.c_str() );
        return;
    }
    
    SG_LOG( SG_GENERAL, SG_DEBUG, "Processing datasource " << filename.c_str() << " with " << poDS->GetLayerCount() << " layers " );
    for (int i=0; i<poDS->GetLayerCount(); i++) {
        OGRLayer* poLayer = poDS->GetLayer(i);
        assert(poLayer != NULL);

        OGRFeatureDefn*         poFDefn = poLayer->GetLayerDefn();
        OGRSpatialReference*    oSourceSRS = poLayer->GetSpatialRef();
        std::string             layername = poFDefn->GetName();
        
        /* setup a transformation to WGS84 */
        if (oSourceSRS == NULL) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Layer " << layername << " has no defined spatial reference system" );
            exit( 1 );
        }
        
        OGRSpatialReference oTargetSRS;
        oTargetSRS.SetWellKnownGeogCS( "WGS84" );
        OGRCoordinateTransformation* poCT = OGRCreateCoordinateTransformation(oSourceSRS, &oTargetSRS);
        
        OGRFeature* poFeature = NULL;
        while ( ( poFeature = poLayer->GetNextFeature()) != NULL )
        {
            fromShapefile( poFDefn, poCT, poFeature, points );
            OGRFeature::DestroyFeature( poFeature );
        }
        
        OCTDestroyCoordinateTransformation ( poCT );
    }
    
    GDALClose( poDS );    
}

void tgMeshTriangulation::fromShapefile( const std::string& filename, std::vector<meshFaceInfo>& faces ) const
{
    GDALAllRegister();
    
    GDALDataset* poDS = (GDALDataset*)GDALOpenEx( filename.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
    if( poDS == NULL )
    {
        SG_LOG( SG_GENERAL, SG_ALERT, "Failed opening datasource " << filename.c_str() );
        return;
    }
    
    SG_LOG( SG_GENERAL, SG_ALERT, "Processing datasource " << filename.c_str() << " with " << poDS->GetLayerCount() << " layers " );
    for (int i=0; i<poDS->GetLayerCount(); i++) {
        OGRLayer* poLayer = poDS->GetLayer(i);
        assert(poLayer != NULL);
        
        OGRFeatureDefn*         poFDefn = poLayer->GetLayerDefn();
        OGRSpatialReference*    oSourceSRS = poLayer->GetSpatialRef();
        std::string             layername = poFDefn->GetName();
        
        /* setup a transformation to WGS84 */
        if (oSourceSRS == NULL) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Layer " << layername << " has no defined spatial reference system" );
            exit( 1 );
        }
        
        OGRSpatialReference oTargetSRS;
        oTargetSRS.SetWellKnownGeogCS( "WGS84" );
        OGRCoordinateTransformation* poCT = OGRCreateCoordinateTransformation(oSourceSRS, &oTargetSRS);
        
        SG_LOG( SG_GENERAL, SG_ALERT, "Layer " << layername << " has " << poLayer->GetFeatureCount() );
        
        OGRFeature* poFeature = NULL;
        while ( ( poFeature = poLayer->GetNextFeature()) != NULL )
        {
            fromShapefile( poFDefn, poCT, poFeature, faces );
            OGRFeature::DestroyFeature( poFeature );
        }
        
        OCTDestroyCoordinateTransformation ( poCT );
    }
    
    GDALClose( poDS );        
}

#if 0
void tgMeshTriangulation::fromShapefile( const std::string& filename, std::vector<meshTriangle>& triangles ) const
{
    GDALAllRegister();
    
    GDALDataset* poDS = (GDALDataset*)GDALOpenEx( filename.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
    if( poDS == NULL )
    {
        SG_LOG( SG_GENERAL, SG_DEBUG, "Failed opening datasource " << filename.c_str() );
        return;
    }
    
    SG_LOG( SG_GENERAL, SG_DEBUG, "Processing datasource " << filename.c_str() << " with " << poDS->GetLayerCount() << " layers " );
    for (int i=0; i<poDS->GetLayerCount(); i++) {
        OGRLayer* poLayer = poDS->GetLayer(i);
        assert(poLayer != NULL);
        
        OGRFeatureDefn*         poFDefn = poLayer->GetLayerDefn();
        OGRSpatialReference*    oSourceSRS = poLayer->GetSpatialRef();
        std::string             layername = poFDefn->GetName();
        
        /* setup a transformation to WGS84 */
        if (oSourceSRS == NULL) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Layer " << layername << " has no defined spatial reference system" );
            exit( 1 );
        }
        
        OGRSpatialReference oTargetSRS;
        oTargetSRS.SetWellKnownGeogCS( "WGS84" );
        OGRCoordinateTransformation* poCT = OGRCreateCoordinateTransformation(oSourceSRS, &oTargetSRS);
        
        OGRFeature* poFeature = NULL;
        while ( ( poFeature = poLayer->GetNextFeature()) != NULL )
        {
            fromShapefile( poFDefn, poCT, poFeature, triangles );
            OGRFeature::DestroyFeature( poFeature );
        }
        
        OCTDestroyCoordinateTransformation ( poCT );
    }
    
    GDALClose( poDS );    
}
#endif

#if 0
// load a single meshTriSegment from a shapefile
void tgMeshTriangulation::fromShapefile( const OGRFeatureDefn* poFDefn, OGRCoordinateTransformation* poCT, OGRFeature* poFeature, std::vector<meshTriSegment>& segments ) const
{
    OGRGeometry *poGeometry = poFeature->GetGeometryRef();
    if (poGeometry == NULL) {
        SG_LOG( SG_GENERAL, SG_INFO, "Found feature without geometry!" );
        return;
    }
    
    OGRwkbGeometryType geoType = wkbFlatten(poGeometry->getGeometryType());
    if (geoType != wkbLineString && geoType != wkbMultiLineString) {
        SG_LOG( SG_GENERAL, SG_INFO, "Unknown feature" << geoType );
        return;
    }
    
    // now the geometry(s)
    poGeometry->transform( poCT );
    
    switch( geoType ) {
        case wkbLineString:
        {
            const OGRLineString* ogr_line = (const OGRLineString*)poGeometry;
            int numPoints = ogr_line->getNumPoints();
            if ( numPoints == 2 ) {
                OGRPoint oSrc, oTrg;
                meshTriPoint src, trg;
                
                ogr_line->getPoint( 0, &oSrc );
                ogr_line->getPoint( 1, &oTrg );
                
                src = meshTriPoint( oSrc.getX(), oSrc.getY() );
                trg = meshTriPoint( oTrg.getX(), oTrg.getY() );

                segments.push_back( meshTriSegment( src, trg ) ); 
            } else {
                SG_LOG( SG_GENERAL, SG_ALERT, " line with more than 2 points not supported " );                
            }
            break;
        }
        
        case wkbMultiLineString:
        {
            SG_LOG( SG_GENERAL, SG_ALERT, " multi line not supported " );
            break;
        }
        
        default:
            break;
    }
    
    return;
}
#endif

#if 0
void tgMeshTriangulation::fromShapefile( const std::string& filename, std::vector<meshTriSegment>& segments ) const
{
    GDALAllRegister();
    
    GDALDataset* poDS = (GDALDataset*)GDALOpenEx( filename.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
    if( poDS == NULL )
    {
        SG_LOG( SG_GENERAL, SG_DEBUG, "Failed opening datasource " << filename.c_str() );
        return;
    }
    
    SG_LOG( SG_GENERAL, SG_DEBUG, "Processing datasource " << filename.c_str() << " with " << poDS->GetLayerCount() << " layers " );
    for (int i=0; i<poDS->GetLayerCount(); i++) {
        OGRLayer* poLayer = poDS->GetLayer(i);
        assert(poLayer != NULL);
        
        OGRFeatureDefn*         poFDefn = poLayer->GetLayerDefn();
        OGRSpatialReference*    oSourceSRS = poLayer->GetSpatialRef();
        std::string             layername = poFDefn->GetName();
        
        /* setup a transformation to WGS84 */
        if (oSourceSRS == NULL) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Layer " << layername << " has no defined spatial reference system" );
            exit( 1 );
        }
        
        OGRSpatialReference oTargetSRS;
        oTargetSRS.SetWellKnownGeogCS( "WGS84" );
        OGRCoordinateTransformation* poCT = OGRCreateCoordinateTransformation(oSourceSRS, &oTargetSRS);
        
        OGRFeature* poFeature = NULL;
        while ( ( poFeature = poLayer->GetNextFeature()) != NULL )
        {
            fromShapefile( poFDefn, poCT, poFeature, segments );
            OGRFeature::DestroyFeature( poFeature );
        }
        
        OCTDestroyCoordinateTransformation ( poCT );
    }
    
    GDALClose( poDS );    
}
#endif


// TODO: instead of saving triangles, we should save constraints and points, so we can easily retriangulate.
#if 0
void tgMeshTriangulation::toShapefile( OGRLayer* poLayer, const meshTriCDTPlus::Subconstraint_iterator cit ) const
{
    OGRPoint      point;
    OGRLineString line;
    
    point.setZ( 0.0 );
    
    point.setX( cit->first.first->point().x() );
    point.setY( cit->first.first->point().y() );
    line.addPoint(&point);
    
    point.setX( cit->first.second->point().x() );
    point.setY( cit->first.second->point().y() );
    line.addPoint(&point);
    
    OGRFeature* poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
    poFeature->SetGeometry(&line);    
    
    if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(poFeature);    
}
#endif

void tgMeshTriangulation::toShapefile( const std::string& datasource, const char* layer, bool marked ) const
{
#if 0    
    GDALDataset*  poDS = NULL;
    OGRLayer*     poPointLayer = NULL;
//    OGRLayer*     poConstraintLayer = NULL;
    OGRLayer*     poTriangleLayer = NULL;
    char          layerName[64];
    //SGBucket      b = mesh->getBucket();
    
    // for stage2, we want to save the triangulation as indicies and faces so we can easily load into a
    // polyhedral surface.
    // generate the vertex handle to index map
    // points will be saved with their index;
    CGAL::Unique_hash_map<meshTriVertexHandle, unsigned int>    V;
    CGAL::Unique_hash_map<meshTriFaceHandle, unsigned int>      F;
    unsigned int                                                vNum = 0;
    unsigned int                                                fNum = 0;
    
//    double north_edge = b.get_center_lat() + 0.5 * b.get_height();
//    double south_edge = b.get_center_lat() - 0.5 * b.get_height();
//    double east_edge  = b.get_center_lon() + 0.5 * b.get_width();
//    double west_edge  = b.get_center_lon() - 0.5 * b.get_width();
//    CGAL::Bbox_2  tileBbox( west_edge, south_edge, east_edge, north_edge );
    
    poDS = mesh->openDatasource( datasource );
    if ( poDS ) {
        //sprintf( layerName, "%s_constraints", layer );
        //poConstraintLayer = mesh->openLayer( poDS, wkbLineString25D, tgMesh::LAYER_FIELDS_NONE, layerName );

        sprintf( layerName, "%s_points", layer );
        poPointLayer = mesh->openLayer( poDS, wkbPoint25D, tgMesh::LAYER_FIELDS_TDS_VERTEX, layerName );

        sprintf( layerName, "%s_triangles", layer );
        poTriangleLayer = mesh->openLayer( poDS, wkbLineString25D, tgMesh::LAYER_FIELDS_TDS_FACE, layerName );
        
        // create V and F
        for (meshTriCDTPlus::Finite_vertices_iterator vit = meshTriangulation.finite_vertices_begin(); vit != meshTriangulation.finite_vertices_end(); vit++ ) {
            V[vit] = vNum++;
        }            
        
        for (meshTriCDTPlus::Finite_faces_iterator   fit = meshTriangulation.finite_faces_begin();     fit != meshTriangulation.finite_faces_end();    fit++ ) {
            F[fit] = fNum++;
        }            
        
#if 0        
        if ( poConstraintLayer ) {
            for (meshTriCDTPlus::Subconstraint_iterator cit = meshTriangulation.subconstraints_begin(); cit != meshTriangulation.subconstraints_end(); cit++ ) {
                toShapefile( poConstraintLayer, cit );
            }
        }
#endif

        if ( poPointLayer ) {
            for (meshTriCDTPlus::Finite_vertices_iterator vit = meshTriangulation.finite_vertices_begin(); vit != meshTriangulation.finite_vertices_end(); vit++ ) {
                toShapefile( poPointLayer, vit, V );
            }
        }
            
        if ( poTriangleLayer ) {
            for (meshTriCDTPlus::Finite_faces_iterator fit=meshTriangulation.finite_faces_begin(); fit!=meshTriangulation.finite_faces_end(); ++fit) {
                if ( marked ) {                    
//                    if ( fit->info().hasFace() ) {
//                        meshTriangle tri = meshTriangulation.triangle(fit);
                        toShapefile( poTriangleLayer, fit, V, F );
//                    }
                } else {
//                    meshTriangle tri = meshTriangulation.triangle(fit);
                    toShapefile( poTriangleLayer, fit, V, F );
                }
            }
        }
    }
    
    // close datasource
    GDALClose( poDS );    
#endif    
}

#if 0
// using meshTriangle instead of face loses z values...
void tgMeshTriangulation::toShapefile( OGRLayer* poLayer, const meshTriangle& tri ) const
{    
    OGRPolygon    polygon;
    OGRPoint      point;
    OGRLinearRing ring;
    
    for ( unsigned int i=0; i<3; i++ ) {
        point.setX( CGAL::to_double( tri[i].x() ) );
        point.setY( CGAL::to_double( tri[i].y() ) );
        point.setZ( 0.0 );
        
        ring.addPoint(&point);
    }
    ring.closeRings();
    polygon.addRing(&ring);
    
    OGRFeature* poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
    poFeature->SetGeometry(&polygon);    
    if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(poFeature);        
}
#endif

void tgMeshTriangulation::toShapefile( const std::string& datasource, const char* layer, std::vector<meshVertexInfo>& points ) const
{
    GDALDataset*  poDS = NULL;
    OGRLayer*     poLayer = NULL;
    
    poDS = mesh->openDatasource( datasource );
    if ( poDS ) {
        poLayer = mesh->openLayer( poDS, wkbPoint25D, tgMesh::LAYER_FIELDS_TDS_VERTEX, layer );
        if ( poLayer ) {
            for ( unsigned int i=0; i<points.size(); i++ ) {
                
            }
        }

        GDALClose( poDS );    
    }    
}

void tgMeshTriangulation::toShapefile( OGRLayer* poLayer, const meshVertexInfo& vi) const
{
    OGRPoint      point;
    
    point.setX( vi.getX() );
    point.setY( vi.getY() );
    point.setZ( vi.getZ() );
    
    OGRFeature* poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
    poFeature->SetGeometry( &point );    
    poFeature->SetField( "tg_vert", vi.getId() );
    
    if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    
    OGRFeature::DestroyFeature(poFeature);    
}

const meshVertexInfo* tgMeshTriangulation::findVertex( int idx ) const
{
    // create a map...
    meshVertexInfo const* cur = NULL;
    
    for ( unsigned int i=0; i<vertexInfo.size(); i++ ) {
        if ( vertexInfo[i].getId() == idx ) {
            cur = &vertexInfo[i];
            break;
        }
    }
    
    return cur;
}

void tgMeshTriangulation::toShapefile( OGRLayer* poLayer, const meshFaceInfo& fi) const
{
    OGRPolygon    polygon;
    OGRPoint      point;
    OGRLinearRing ring;
    char          constrained[8];
    char          indices[32];
    bool          isFinite = true;
    
    // we really don't need to save the face geometry - it is determined by the vertex
    // indices saved in the point layer, along with the metadata.  But it helps to
    // visualize them in QGIS - so save it...
    for ( unsigned int i=0; i<3; i++ ) {
        if ( fi.getVid(i) != 0 ) {
            // finite face - save geometry
            const meshVertexInfo* vi = findVertex( fi.getVid(i) );
            if ( vi ) {
                point.setX( vi->getX() );
                point.setY( vi->getY() );
                point.setZ( vi->getZ() );

                ring.addPoint(&point);
            } else {
                SG_LOG(SG_GENERAL, SG_ALERT, "infinite face");
                isFinite = false;
            }
        }
    }
    ring.closeRings();
    polygon.addRing(&ring);
    
    OGRFeature* poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
    
//    if ( isFinite ) {
        poFeature->SetGeometry(&polygon);
//    }

    // save meta data
    poFeature->SetField("tds_fid", fi.getFid() );
        
    fi.getConstrainedField(constrained);
    poFeature->SetField("tds_cons", constrained );
    
    fi.getVIdxField(indices);
    poFeature->SetField("tds_vidx", indices );
    
    fi.getNIdxField(indices);
    poFeature->SetField("tds_nidx", indices );
    
    if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(poFeature);
}

#if 0
void tgMeshTriangulation::toShapefile( OGRLayer* poLayer, const meshTriVertexHandle vit, 
                                       CGAL::Unique_hash_map<meshTriVertexHandle, unsigned int>& V) const
{
    OGRPoint      point;
        
    point.setX( vit->point().x() );
    point.setY( vit->point().y() );
    point.setZ( vit->info().getElevation() );
        
    OGRFeature* poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
    poFeature->SetGeometry(&point);    
    poFeature->SetField("tg_vert", (int)V[vit]);

    if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(poFeature);    
}
                  
void tgMeshTriangulation::toShapefile( OGRLayer* poLayer, const meshTriFaceHandle tri, 
                                       CGAL::Unique_hash_map<meshTriVertexHandle, unsigned int>& V, 
                                       CGAL::Unique_hash_map<meshTriFaceHandle, unsigned int>& F ) const
{
    OGRPolygon    polygon;
    OGRPoint      point;
    OGRLinearRing ring;
    char          constrained[8];
    char          indices[32];
    
    for ( unsigned int i=0; i<3; i++ ) {
        point.setX( CGAL::to_double( tri->vertex(i)->point().x() ) );
        point.setY( CGAL::to_double( tri->vertex(i)->point().y() ) );
        point.setZ( tri->vertex(i)->info().getElevation() );
        
        ring.addPoint(&point);
    }
    ring.closeRings();
    polygon.addRing(&ring);
    
    OGRFeature* poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
    poFeature->SetGeometry(&polygon);
    
    sprintf(constrained, "%01d,%01d,01%d", 
            tri->is_constrained(0)?1:0, 
            tri->is_constrained(1)?1:0, 
            tri->is_constrained(2)?1:0 
           );
    poFeature->SetField("tds_cons", constrained );
    
    sprintf(indices, "%08d,%08d,%08d", 
            V[tri->vertex(0)], 
            V[tri->vertex(1)],
            V[tri->vertex(2)]
           );
    poFeature->SetField("tds_vidx", indices );
    
    sprintf(indices, "%08d,%08d,%08d", 
            F[tri->neighbor(0)], 
            F[tri->neighbor(1)],
            F[tri->neighbor(2)]
    );
    poFeature->SetField("tds_nidx", indices );
    
    if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(poFeature);        
}
#endif

#if 0
void tgMeshTriangulation::toShapefile( OGRLayer* poLayer, const meshTriFaceHandle tri ) const
{
    OGRPolygon    polygon;
    OGRPoint      point;
    OGRLinearRing ring;
    char          constrained[8];
    char          v_indices[32];
    
    for ( unsigned int i=0; i<3; i++ ) {
        point.setX( CGAL::to_double( tri->vertex(i)->point().x() ) );
        point.setY( CGAL::to_double( tri->vertex(i)->point().y() ) );
        point.setZ( tri->vertex(i)->info().getElevation() );
        
        ring.addPoint(&point);
    }
    ring.closeRings();
    polygon.addRing(&ring);
    
    OGRFeature* poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
    poFeature->SetGeometry(&polygon);    
    if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(poFeature);        
}
#endif

void tgMeshTriangulation::toShapefile( const std::string& datasource, const char* layer, const moveNodeTree& tree )
{
    GDALDataset*  poDS = NULL;
    OGRLayer*     poLayer = NULL;

    poDS = mesh->openDatasource( datasource );
    if ( poDS ) {
        poLayer = mesh->openLayer( poDS, wkbPoint25D, tgMesh::LAYER_FIELDS_NONE, layer );
        if ( poLayer ) {
            moveNodeTree::iterator mnIt = tree.begin();
            unsigned int ptNum = 1;
            while ( mnIt != tree.end() ) {
                char desc[16];
                
                moveNodeData mnData = *mnIt;
                meshTriPoint fromPt = boost::get<0>(mnData);
                meshTriPoint toPt   = boost::get<1>(mnData);
                
                sprintf( desc, "%04d_from", ptNum );
                toShapefile( poLayer, fromPt, desc );
                
                toShapefile( poLayer, toPt, desc );
                sprintf( desc, "%04d_to", ptNum );
                
                ptNum++;
                mnIt++;
            }
        }
        GDALClose( poDS );    
    }   
}

void tgMeshTriangulation::toShapefile( const std::string& datasource, const char* layer, const nodeMembershipTree& tree )
{
    GDALDataset*  poDS = NULL;
    OGRLayer*     poLayer = NULL;
    
    poDS = mesh->openDatasource( datasource );
    if ( poDS ) {
        poLayer = mesh->openLayer( poDS, wkbPoint25D, tgMesh::LAYER_FIELDS_NONE, layer );
        if ( poLayer ) {
            nodeMembershipTree::iterator nmIt = tree.begin();
            while ( nmIt != tree.end() ) {
                char desc[16];
                
                nodeMembershipData  nmData = *nmIt;
                meshTriPoint        pt = boost::get<0>(nmData);
                
                switch( boost::get<1>(nmData) ) {
                    case NODE_CURRENT:
                        sprintf( desc, "%04d_CURR", boost::get<2>(nmData) );
                        break;
                        
                    case NODE_NEIGHBOR:
                        sprintf( desc, "%04d_NEIGH", boost::get<2>(nmData) );
                        break;

                    case NODE_BOTH:
                        sprintf( desc, "%04d_BOTH", boost::get<2>(nmData) );
                        break;                        
                }
                toShapefile( poLayer, pt, desc );
                
                nmIt++;
            }
        }
        GDALClose( poDS );    
    }   
}

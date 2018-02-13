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

// Save a vector of meshTriPoints ( for shared edges )
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

    switch( geoType ) {
        case wkbPoint:
        {
            points.push_back( meshVertexInfo( poFeature ) );
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
            fromShapefile( poFDefn, poCT, poFeature, faces );
            OGRFeature::DestroyFeature( poFeature );
        }

        OCTDestroyCoordinateTransformation ( poCT );
    }
    
    GDALClose( poDS );        
}

void tgMeshTriangulation::toShapefile( const std::string& datasource, const char* layer, std::vector<const meshVertexInfo *>& points ) const
{
    GDALDataset*  poDS = NULL;
    OGRLayer*     poLayer = NULL;

    poDS = mesh->openDatasource( datasource );
    if ( poDS ) {
        poLayer = mesh->openLayer( poDS, wkbPoint25D, tgMesh::LAYER_FIELDS_TDS_VERTEX, layer );
        if ( poLayer ) {
            for ( unsigned int i=0; i<points.size(); i++ ) {
                toShapefile( poLayer, points[i] );
            }
        }

        GDALClose( poDS );    
    }
}

void tgMeshTriangulation::toShapefile( OGRLayer* poLayer, const meshVertexInfo* vip) const
{
    OGRPoint      point;

    point.setX( vip->getX() );
    point.setY( vip->getY() );
    point.setZ( vip->getZ() );

    OGRFeature* poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
    poFeature->SetGeometry( &point );    
    poFeature->SetField( "tg_vert", vip->getId() );

    poFeature->SetField( "tds_x", vip->getX() );
    poFeature->SetField( "tds_y", vip->getY() );
    poFeature->SetField( "tds_z", vip->getZ() );

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
    // bool          isFinite = true;

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
                // isFinite = false;
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

void tgMeshTriangulation::toShapefile( OGRLayer* poLayer, const meshTriSegment& seg, const char* desc ) const
{
    OGRPoint      point;
    OGRLineString line;

    point.setZ( 0.0 );

    point.setX( seg.source().x() );
    point.setY( seg.source().y() );
    line.addPoint(&point);

    point.setX( seg.target().x() );
    point.setY( seg.target().y() );
    line.addPoint(&point);

    OGRFeature* poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
    poFeature->SetGeometry(&line);    
    poFeature->SetField("tg_desc", desc );

    if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(poFeature);        
}

void tgMeshTriangulation::toShapefile( const std::string& datasource, const char* layer, const std::vector<movedNode>& map )
{
    GDALDataset*  poDS = NULL;
    OGRLayer*     poLayer = NULL;

    poDS = mesh->openDatasource( datasource );
    if ( poDS ) {
        poLayer = mesh->openLayer( poDS, wkbPoint25D, tgMesh::LAYER_FIELDS_NONE, layer );
        if ( poLayer ) {
            unsigned int ptNum = 1;
            for ( unsigned int i=0; i<map.size(); i++ ) {
                char desc[16];

                meshTriPoint toPt   = map[i].newPosition;

//              sprintf( desc, "%04d_from", ptNum );
//              toShapefile( poLayer, fromPt, desc );

                toShapefile( poLayer, toPt, desc );
                sprintf( desc, "%04d_to", ptNum );

                ptNum++;
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

void tgMeshTriangulation::saveTds( const std::string& bucketPath ) const
{
    GDALDataset*  poDS = NULL;
    OGRLayer*     poPointLayer = NULL;
    OGRLayer*     poFaceLayer = NULL;

    poDS = mesh->openDatasource( bucketPath );
    if ( poDS ) {
        poPointLayer = mesh->openLayer( poDS, wkbPoint25D, tgMesh::LAYER_FIELDS_TDS_VERTEX, "tds_points" );
        poFaceLayer  = mesh->openLayer( poDS, wkbLineString25D, tgMesh::LAYER_FIELDS_TDS_FACE, "tds_faces" );

        if ( poPointLayer ) {
            // don't save first point : it's infinite vertex
            for (unsigned int i=1; i<vertexInfo.size(); i++) {
                toShapefile( poPointLayer, &vertexInfo[i] );
            }
        }

        if ( poFaceLayer ) {
            for (unsigned int i=0; i<faceInfo.size(); i++) {
                toShapefile( poFaceLayer, faceInfo[i] );
            }
        } else {
            SG_LOG( SG_GENERAL, SG_INFO, "tgMesh::constrainedTriangulate - saveTDS - Error openeing face layer" );            
        }

        // close datasource
        GDALClose( poDS );
    }
}

bool tgMeshTriangulation::loadTds( const std::string& bucketPath )
{
    // load the tile points
    std::vector<meshVertexInfo>   points;
    std::vector<meshFaceInfo>     faces;
    std::string                   filePath;
    bool                          hasLand = false;

    meshTriTDS& tds = meshTriangulation.tds();
    tds.clear();

    // load vertices, and save their handles in V
    filePath = bucketPath + "/tds_points.shp"; 
    fromShapefile( filePath, points );

    filePath = bucketPath + "/tds_faces.shp"; 
    fromShapefile( filePath, faces );

    if (!points.empty() && !faces.empty()) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "loadTDS from " << points.size() << " points and " << faces.size() << " faces" );
        SG_LOG(SG_GENERAL, SG_DEBUG, "loadTDS - begin valid: " << tds.is_valid() << " dimension: " << tds.dimension() << " verts: " << tds.number_of_vertices() );

        unsigned int n = points.size();
        unsigned int m = faces.size();
        unsigned int i;

        // tds dimension starts at -2 ( 0 verts )
        //                         -1 ( infinite vert )
        //                          0 ( line from infinit vert to finite vert )
        //                          1 ( three verts )
        //                          2 ( faces defined )
        // insert verts
        if ( n != 0 ) {
            int index = 0;
            hasLand = true;

            tds.set_dimension(2);

            //std::vector<meshTriTDS::Vertex_handle> V(n+1);
            //std::vector<meshTriTDS::Face_handle>   F(m);
            vertexIndexToHandleMap.clear();
            faceIndexToHandleMap.clear();

            // create the first ( infinite vertex
            vertexIndexToHandleMap[0] = tds.create_vertex();

            // read the rest from the array
            for( i = 0; i < n; i++ ) {
                index = points[i].getId();

                // index 0 is infinite vertex, so don't set it's position
                vertexIndexToHandleMap[index] = tds.create_vertex();
                if (index) {
                    vertexIndexToHandleMap[index]->set_point( points[i].getPoint() );
                }
            }

            for( i = 0; i < m; i++ ) {
                int fid = faces[i].getFid();

                faceIndexToHandleMap[fid] = tds.create_face();
                for(int j = 0; j < 3; j++){
                    int vid = faces[i].getVid(j);
                    faceIndexToHandleMap[fid]->set_vertex(j, vertexIndexToHandleMap[vid]);
                    // The face pointer of vertices is set too often,
                    // but otherwise we had to use a further map
                    vertexIndexToHandleMap[vid]->set_face( faceIndexToHandleMap[fid] );
                }
            }

            // Setting the neighbor pointers 
            for( i = 0; i < m; i++ ) {
                int fid = faces[i].getFid();

                for( int j = 0; j < tds.dimension()+1; j++ ){
                    int nid = faces[i].getNid(j);
                    faceIndexToHandleMap[fid]->set_neighbor(j, faceIndexToHandleMap[nid]);
                }
            }

            meshTriangulation.set_infinite_vertex(vertexIndexToHandleMap[0]);

            meshTriCDT::All_faces_iterator fit, fit_end;
            int i;
            for ( fit = meshTriangulation.all_faces_begin(), fit_end = meshTriangulation.all_faces_end(), i = 0; fit != fit_end; fit++, i++ ) {
                for ( int j=0; j<3; j++ ) {
                    bool con = faces[i].getConstrained(j) ? true:false;
                    fit->set_constraint(j, con?true:false);
                }
            }

            SG_LOG(SG_GENERAL, SG_DEBUG, "LoadTDS - COMPLETE TDS valid: " << tds.is_valid() << " dimension: " << tds.dimension() << " verts: " << tds.number_of_vertices() );
            meshTriangulation.is_valid(true);

        }
    }

    return hasLand;
}

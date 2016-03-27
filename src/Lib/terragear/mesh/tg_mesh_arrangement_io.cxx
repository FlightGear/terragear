#include <simgear/debug/logstream.hxx>

#include <CGAL/Bbox_2.h>

#include "tg_mesh.hxx"

// this needs to save polygons...
void tgMeshArrangement::toShapefile( const std::string& datasource, const char* layer ) const
{
    GDALDataset*  poDS = NULL;
    OGRLayer*     poLineLayer = NULL;
    OGRLayer*     poPointLayer = NULL;
    OGRLayer*     poFaceLayer = NULL;
    char          layerName[256];
    
    poDS = mesh->openDatasource( datasource );
    if ( poDS ) {
        sprintf( layerName, "%s_segs", layer );
        poLineLayer = mesh->openLayer( poDS, wkbLineString25D, tgMesh::LAYER_FIELDS_NONE, layerName );
        
        if ( poLineLayer ) {
            meshArrangement::Edge_const_iterator eit;
            
            for ( eit = meshArr.edges_begin(); eit != meshArr.edges_end(); ++eit ) {        
                toShapefile( poLineLayer, eit->curve(), "edge" );
            }
        }
        
        sprintf( layerName, "%s_pts", layer );
        poPointLayer = mesh->openLayer( poDS, wkbPoint25D, tgMesh::LAYER_FIELDS_NONE, layerName );

        if ( poPointLayer ) {
            meshArrangement::Vertex_const_iterator vit;
            
            for ( vit = meshArr.vertices_begin(); vit != meshArr.vertices_end(); ++vit ) {        
                toShapefile( poPointLayer, vit->point(), "pt" );
            }
        }

        sprintf( layerName, "%s_faces", layer );
        poFaceLayer = mesh->openLayer( poDS, wkbPolygon25D, tgMesh::LAYER_FIELDS_ARR, layerName );
        
        if ( poFaceLayer ) {
            // traverse the faces via metaLookup - so we can save good info per face
            for ( unsigned int i=0; i<metaLookup.size(); i++ ) {
                toShapefile( poFaceLayer, metaLookup[i].face, metaLookup[i].point, "face" );
            }
        }
        
    }
    
    // close datasource
    GDALClose( poDS );    
}

void tgMeshArrangement::toShapefile( OGRLayer* poLayer, const meshArrFaceConstHandle f, const cgalPoly_Point& qp, const char* desc ) const
{
    OGRPoint      point;
    OGRLinearRing ring;
    OGRPolygon    poly;
    
    // write the face
    if ( f->has_outer_ccb() ) {
        meshArrHalfedgeConstCirculator ccb = f->outer_ccb();
        meshArrHalfedgeConstCirculator cur = ccb;
        meshArrHalfedgeConstHandle     curHe;

        point.setZ( 0.0 );
        
        do {
            curHe = cur;

            point.setX( CGAL::to_double( curHe->source()->point().x() ) );
            point.setY( CGAL::to_double( curHe->source()->point().y() ) );
            ring.addPoint(&point);
            
            cur++;
        } while ( cur != ccb );
        
        ring.closeRings();
        poly.addRing(&ring);
        
        OGRFeature* poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
        poFeature->SetGeometry(&poly);
        poFeature->SetField("qp_lon", CGAL::to_double( qp.x() ) );
        poFeature->SetField("qp_lat", CGAL::to_double( qp.y() ) );
        
        if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
        {
            SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
        }
        OGRFeature::DestroyFeature(poFeature);                
    } else {
        SG_LOG(SG_GENERAL, SG_ALERT, "face has no outer ccb");        
    }
}

void tgMeshArrangement::toShapefile( OGRLayer* poLayer, const meshArrSegment& seg, const char* desc ) const
{    
    OGRPoint      point;
    OGRLineString line;
    
    point.setZ( 0.0 );

    point.setX( CGAL::to_double( seg.source().x() ) );
    point.setY( CGAL::to_double( seg.source().y() ) );
    line.addPoint(&point);

    point.setX( CGAL::to_double( seg.target().x() ) );
    point.setY( CGAL::to_double( seg.target().y() ) );
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

void tgMeshArrangement::toShapefile( OGRLayer* poLayer, const meshArrPoint& pt, const char* desc ) const
{    
    OGRPoint      point;
    
    point.setZ( 0.0 );    
    point.setX( CGAL::to_double( pt.x() ) );
    point.setY( CGAL::to_double( pt.y() ) );
    
    OGRFeature* poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
    poFeature->SetGeometry(&point);    
    poFeature->SetField("tg_desc", desc );
    
    if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(poFeature);        
}


void tgMeshArrangement::fromShapefile( const OGRFeatureDefn* poFDefn, OGRCoordinateTransformation* poCT, OGRFeature* poFeature, std::vector<meshArrSegment>& segments ) const
{
    OGRGeometry *poGeometry = poFeature->GetGeometryRef();
    if (poGeometry == NULL) {
        SG_LOG( SG_GENERAL, SG_INFO, "Found feature without geometry!" );
        return;
    }
    
    OGRwkbGeometryType geoType = wkbFlatten(poGeometry->getGeometryType());
    if (geoType != wkbPolygon && geoType != wkbMultiPolygon) {
        SG_LOG( SG_GENERAL, SG_INFO, "Unknown feature" << geoType );
        return;
    }
    
    // now the geometry(s)
    poGeometry->transform( poCT );
    
    switch( geoType ) {
        case wkbPolygon:
        {
            tgPolygonSet polySet( poFeature, (OGRPolygon *)poGeometry );
            std::vector<cgalPoly_Segment> segs; 
            polySet.toSegments( segs, false );
            for ( unsigned int i=0; i<segs.size(); i++ ) {
                segments.push_back( meshArrSegment(segs[i].source(), segs[i].target() ) );
            }
            break;
        }
        
        case wkbMultiPolygon:
        {
            SG_LOG( SG_GENERAL, SG_ALERT, " multi line not supported " );
            break;
        }
        
        default:
            break;
    }
    
    return;
}

void tgMeshArrangement::fromShapefile( const std::string& filename, std::vector<meshArrSegment>& segments ) const
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

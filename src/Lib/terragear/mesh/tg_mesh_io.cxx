#include <simgear/debug/logstream.hxx>

#include "tg_mesh.hxx"

GDALDataset* tgMesh::openDatasource( const char* datasource_name )
{
    GDALDataset*    poDS = NULL;
    GDALDriver*     poDriver = NULL;
    const char*     format_name = "ESRI Shapefile";
    
    SG_LOG( SG_GENERAL, SG_DEBUG, "Open Datasource: " << datasource_name );
    
    GDALAllRegister();
    
    poDriver = GetGDALDriverManager()->GetDriverByName( format_name );
    if ( poDriver ) {    
        poDS = poDriver->Create( datasource_name, 0, 0, 0, GDT_Unknown, NULL );
    }
    
    return poDS;
}

OGRLayer* tgMesh::openLayer( GDALDataset* poDS, OGRwkbGeometryType lt, const char* layer_name )
{
    OGRLayer*           poLayer = NULL;
 
    if ( !strlen( layer_name )) {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgMesh::openLayer: layer name is NULL" );
        exit(0);
    }
    
    poLayer = poDS->GetLayerByName( layer_name );
    if ( !poLayer ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "tgMesh::openLayer: layer " << layer_name << " doesn't exist - create" );

        OGRSpatialReference srs;
        srs.SetWellKnownGeogCS("WGS84");
        
        poLayer = poDS->CreateLayer( layer_name, &srs, lt, NULL );
        
        OGRFieldDefn descriptionField( "tg_desc", OFTString );
        descriptionField.SetWidth( 128 );
        if( poLayer->CreateField( &descriptionField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_desc' failed" );
        }
        
    } else {
        SG_LOG(SG_GENERAL, SG_DEBUG, "tgMesh::toShapefile: layer " << layer_name << " already exists - open" );        
    }
   
    return poLayer;
}

void tgMesh::toShapefile( const char* datasource, const char* layer, const meshArrangement& arr )
{
    GDALDataset*  poDS = NULL;
    OGRLayer*     poLayer = NULL;
    
    poDS = openDatasource( datasource );
    if ( poDS ) {
        poLayer = openLayer( poDS, wkbLineString25D, layer );
        if ( poLayer ) {
            meshArrangement::Edge_const_iterator eit;
            
            for ( eit = arr.edges_begin(); eit != arr.edges_end(); ++eit ) {        
                toShapefile( poLayer, eit->curve(), "edge" );
            }            
        }
    }
    
    // close datasource
    GDALClose( poDS );    
}

void tgMesh::toShapefile( OGRLayer* poLayer, const meshArrSegment& seg, const char* desc )
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

void tgMesh::toShapefile( const char* datasource, const char* layer, const meshTriCDTPlus& triangulation, bool marked )
{
    GDALDataset*  poDS = NULL;
    OGRLayer*     poLayer = NULL;
    
    poDS = openDatasource( datasource );
    if ( poDS ) {
        poLayer = openLayer( poDS, wkbLineString25D, layer );
        if ( poLayer ) {
            for (meshTriCDTPlus::Finite_faces_iterator fit=triangulation.finite_faces_begin(); fit!=triangulation.finite_faces_end(); ++fit) {
                if ( marked ) {
                    if ( fit->info().hasFace() ) {
                        meshTriangle tri = meshTriangulation.triangle(fit);
                        toShapefile( poLayer, tri );
                    }
                } else {
                    meshTriangle tri = meshTriangulation.triangle(fit);
                    toShapefile( poLayer, tri );                    
                }
            }
        }
    }
    
    // close datasource
    GDALClose( poDS );    
}

void tgMesh::toShapefile( OGRLayer* poLayer, const meshTriangle& tri )
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

void tgMesh::toShapefiles( const char* dataset )
{
    
}

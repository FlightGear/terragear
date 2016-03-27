#include <simgear/debug/logstream.hxx>

#include <CGAL/Bbox_2.h>

#include "tg_mesh.hxx"

GDALDataset* tgMesh::openDatasource( const std::string& datasource_name ) const
{
    GDALDataset*    poDS = NULL;
    GDALDriver*     poDriver = NULL;
    const char*     format_name = "ESRI Shapefile";
    
    SG_LOG( SG_GENERAL, SG_DEBUG, "Open Datasource: " << datasource_name );
    
    GDALAllRegister();
    
    poDriver = GetGDALDriverManager()->GetDriverByName( format_name );
    if ( poDriver ) {    
        poDS = poDriver->Create( datasource_name.c_str(), 0, 0, 0, GDT_Unknown, NULL );
    }
    
    return poDS;
}

OGRLayer* tgMesh::openLayer( GDALDataset* poDS, OGRwkbGeometryType lt, MeshLayerFields lf, const char* layer_name ) const
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
        
        if ( lf == LAYER_FIELDS_ARR ) {
            OGRFieldDefn qp_lon( "qp_lon", OFTReal );
            qp_lon.SetWidth( 24 );
            qp_lon.SetPrecision( 3 );        
            if( poLayer->CreateField( &qp_lon ) != OGRERR_NONE ) {
                SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'qp_lon' failed" );
            }

            OGRFieldDefn qp_lat( "qp_lat", OFTReal );
            qp_lat.SetWidth( 24 );
            qp_lat.SetPrecision( 3 );        
            if( poLayer->CreateField( &qp_lat ) != OGRERR_NONE ) {
                SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'qp_lat' failed" );
            }
        } else if ( lf == LAYER_FIELDS_TDS_VERTEX ) {
            OGRFieldDefn vertex( "tg_vert", OFTInteger );
            if( poLayer->CreateField( &vertex ) != OGRERR_NONE ) {
                SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_vert' failed" );
            }
        } else if ( lf == LAYER_FIELDS_TDS_FACE ) {
            // when saving / loading TDS, we need constraint, vertex, and neighbor indices
            OGRFieldDefn fid( "tds_fid", OFTInteger );
            if( poLayer->CreateField( &fid ) != OGRERR_NONE ) {
                SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tds_fid' failed" );
            }
            
            OGRFieldDefn constrained( "tds_cons", OFTString );
            descriptionField.SetWidth( 8 );
            if( poLayer->CreateField( &constrained ) != OGRERR_NONE ) {
                SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tds_cons' failed" );
            }

            OGRFieldDefn vindex( "tds_vidx", OFTString );
            descriptionField.SetWidth( 32 );
            if( poLayer->CreateField( &vindex ) != OGRERR_NONE ) {
                SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tds_vidx' failed" );
            }
            
            OGRFieldDefn nindex( "tds_nidx", OFTString );
            descriptionField.SetWidth( 32 );
            if( poLayer->CreateField( &nindex ) != OGRERR_NONE ) {
                SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tds_nidx' failed" );
            }
        }
    } else {
        SG_LOG(SG_GENERAL, SG_DEBUG, "tgMesh::toShapefile: layer " << layer_name << " already exists - open" );        
    }
   
    return poLayer;
}
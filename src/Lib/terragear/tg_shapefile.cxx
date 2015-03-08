#include <ogrsf_frmts.h> 

#include <simgear/debug/logstream.hxx>
#include <simgear/misc/sg_path.hxx>

#include "tg_shapefile.hxx"
// #include "tg_misc.hxx"

bool tgShapefile::initialized = false;

void* tgShapefile::OpenDatasource( const char* datasource_name )
{
    OGRDataSource*  datasource;
    OGRSFDriver*    ogrdriver;
    const char*     format_name = "ESRI Shapefile";

    SG_LOG( SG_GENERAL, SG_DEBUG, "Open Datasource: " << datasource_name );
 
    if (!tgShapefile::initialized) {
        OGRRegisterAll();
        tgShapefile::initialized = true;
    }

    SGPath sgp( datasource_name );
    sgp.create_dir( 0755 );
    
    ogrdriver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName( format_name );
    if ( !ogrdriver ) {
        SG_LOG( SG_GENERAL, SG_ALERT, "Unknown datasource format driver: " << format_name );
        exit(1);
    }

    datasource = ogrdriver->Open( datasource_name, TRUE );
    if ( !datasource ) {
        datasource = ogrdriver->CreateDataSource( datasource_name, NULL );
    }

    if ( !datasource ) {
        SG_LOG( SG_GENERAL, SG_ALERT, "Unable to open or create datasource: " << datasource_name );
    }

    return (void*)datasource;
}

void* tgShapefile::OpenLayer( void* ds_id, const char* layer_name, shapefile_layer_t type) {
    OGRDataSource* datasource = ( OGRDataSource * )ds_id;
    OGRLayer* layer;

    OGRwkbGeometryType  ogr_type;
    OGRSpatialReference srs;
    srs.SetWellKnownGeogCS("WGS84");

    switch( type ) {
        case LT_POINT:
            ogr_type = wkbPoint25D;
            break;
            
        case LT_LINE:
            ogr_type = wkbLineString25D;
            break;
            
        case LT_POLY:
        default:
            ogr_type = wkbPolygon25D;
            break;
    }
    
    layer = datasource->GetLayerByName( layer_name );

    if ( !layer ) {
        layer = datasource->CreateLayer( layer_name, &srs, ogr_type, NULL );

        OGRFieldDefn descriptionField( "ID", OFTString );
        descriptionField.SetWidth( 128 );

        if( layer->CreateField( &descriptionField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'Description' failed" );
        }
    }

    if ( !layer ) {
        SG_LOG( SG_GENERAL, SG_ALERT, "Creation of layer '" << layer_name << "' failed" );
        return NULL;
    }

    return (void*)layer;
}

void* tgShapefile::CloseDatasource( void* ds_id )
{
    OGRDataSource* datasource = ( OGRDataSource * )ds_id;
    OGRDataSource::DestroyDataSource( datasource );

    return (void *)-1;
}


void tgShapefile::FromClipper( const ClipperLib::Paths& subject, bool asPolygon, const std::string& datasource, const std::string& layer, const std::string& description )
{
    void* ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
    if ( !ds_id ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromClipper open datasource failed. datasource: " << datasource << " layer: " << layer << " description: " << description );
    }
    
    OGRLayer* l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_POLY );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenLayer returned " << (unsigned long)l_id);

    OGRPolygon    polygon;
    OGRLinearRing ring;
    OGRPoint      point;
    
    for ( unsigned int i = 0; i < subject.size(); i++ ) {
        ClipperLib::Path const& contour = subject[i];

        if (contour.size() < 3) {
            SG_LOG(SG_GENERAL, SG_DEBUG, "Polygon with less than 3 points");
        } else {
            // FIXME: Current we ignore the hole-flag and instead assume
            //        that the first ring is not a hole and the rest
            //        are holes
            for (unsigned int pt = 0; pt < contour.size(); pt++) {
                SGGeod geod = SGGeod_FromClipper( contour[pt] );

                point.setX( geod.getLongitudeDeg() );
                point.setY( geod.getLatitudeDeg() );
                point.setZ( 0.0 );

                ring.addPoint( &point );
            }

            ring.closeRings();
            polygon.addRingDirectly( &ring );
        }

        OGRFeature* feature = new OGRFeature( l_id->GetLayerDefn() );
        feature->SetField("ID", description.c_str());
        feature->SetGeometry(&polygon);
        if( l_id->CreateFeature( feature ) != OGRERR_NONE )
        {
            SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
        }

        OGRFeature::DestroyFeature(feature);
    }

    // close after each write
    if ( ds_id >= 0 ) {
        ds_id = tgShapefile::CloseDatasource( ds_id );
    }
}

void tgShapefile::FromContour( const tgContour& subject, bool asPolygon, const std::string& datasource, const std::string& layer, const std::string& description )
{
    void* ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
    if ( !ds_id ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromContour open datasource failed. datasource: " << datasource << " layer: " << layer << " description: " << description );
    }

    OGRLayer* l_id;
    if ( asPolygon ) {
        l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_POLY );
    } else {
        l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_LINE );
    }
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenLayer returned " << (unsigned long)l_id);

    if (subject.GetSize() < 3) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Polygon with less than 3 points");
    } else {
        if ( asPolygon ) {
            OGRPolygon    polygon;
            OGRLinearRing ring;
            OGRPoint      point;
            
            for (unsigned int pt = 0; pt < subject.GetSize(); pt++) {
                point.setX( subject.GetNode(pt).getLongitudeDeg() );
                point.setY( subject.GetNode(pt).getLatitudeDeg() );
                point.setZ( 0.0 );
                ring.addPoint(&point);
                
                // get heading to next point
                double heading;
                if ( pt == subject.GetSize()-1 ) {
                    heading = SGGeodesy::courseDeg( subject.GetNode(pt), subject.GetNode(0) );
                } else {
                    heading = SGGeodesy::courseDeg( subject.GetNode(pt), subject.GetNode(pt+1) );
                }
                
                SGGeod left  = SGGeodesy::direct( subject.GetNode(pt), SGMiscd::normalizePeriodic(0, 360, heading-170), 0.2 );            
                point.setX( left.getLongitudeDeg() );
                point.setY( left.getLatitudeDeg() );
                point.setZ( 0.0 );
                ring.addPoint(&point);
                
                SGGeod right = SGGeodesy::direct( subject.GetNode(pt), SGMiscd::normalizePeriodic(0, 360, heading+170), 0.2 );
                point.setX( right.getLongitudeDeg() );
                point.setY( right.getLatitudeDeg() );
                point.setZ( 0.0 );
                ring.addPoint(&point);
                
                point.setX( subject.GetNode(pt).getLongitudeDeg() );
                point.setY( subject.GetNode(pt).getLatitudeDeg() );
                point.setZ( 0.0 );
                ring.addPoint(&point);
            }
            ring.closeRings();
            polygon.addRingDirectly(&ring);

            OGRFeature* feature = NULL;
            feature = new OGRFeature( l_id->GetLayerDefn() );
            feature->SetField("ID", description.c_str());
            feature->SetGeometry(&polygon);
            if( l_id->CreateFeature( feature ) != OGRERR_NONE )
            {
                SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
            }
            OGRFeature::DestroyFeature(feature);
        } else {
            OGRLineString ogr_contour;
            OGRPoint      point;
            
            for (unsigned int pt = 0; pt < subject.GetSize(); pt++) {                
                point.setX( subject.GetNode(pt).getLongitudeDeg() );
                point.setY( subject.GetNode(pt).getLatitudeDeg() );
                point.setZ( 0.0 );
                ogr_contour.addPoint(&point);
                
                // get heading to next point
                double heading;
                if ( pt == subject.GetSize()-1 ) {
                    heading = SGGeodesy::courseDeg( subject.GetNode(pt), subject.GetNode(0) );
                } else {
                    heading = SGGeodesy::courseDeg( subject.GetNode(pt), subject.GetNode(pt+1) );
                }
                
                SGGeod left  = SGGeodesy::direct( subject.GetNode(pt), SGMiscd::normalizePeriodic(0, 360, heading-170), 1.0 );            
                point.setX( left.getLongitudeDeg() );
                point.setY( left.getLatitudeDeg() );
                point.setZ( 0.0 );
                ogr_contour.addPoint(&point);
                
                SGGeod right = SGGeodesy::direct( subject.GetNode(pt), SGMiscd::normalizePeriodic(0, 360, heading+170), 1.0 );
                point.setX( right.getLongitudeDeg() );
                point.setY( right.getLatitudeDeg() );
                point.setZ( 0.0 );
                ogr_contour.addPoint(&point);
                
                point.setX( subject.GetNode(pt).getLongitudeDeg() );
                point.setY( subject.GetNode(pt).getLatitudeDeg() );
                point.setZ( 0.0 );
                ogr_contour.addPoint(&point);
            }

            point.setX( subject.GetNode(0).getLongitudeDeg() );
            point.setY( subject.GetNode(0).getLatitudeDeg() );
            point.setZ( 0.0 );
            ogr_contour.addPoint(&point);
                        
            OGRFeature* feature = NULL;
            feature = new OGRFeature( l_id->GetLayerDefn() );
            feature->SetField("ID", description.c_str());
            feature->SetGeometry(&ogr_contour);
            if( l_id->CreateFeature( feature ) != OGRERR_NONE )
            {
                SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
            }
            OGRFeature::DestroyFeature(feature);            
        }
    }

    // close after each write
    ds_id = tgShapefile::CloseDatasource( ds_id );
}

void tgShapefile::FromPolygon( void *lid, const tgPolygon& subject, bool asPolygon, bool withTriangles, const std::string& description )
{    
    OGRLayer* l_id = (OGRLayer *)lid;
    
    if ( asPolygon ) {
        OGRPolygon* polygon = new OGRPolygon;
        
        for ( unsigned int i = 0; i < subject.Contours(); i++ ) {
            bool skip_ring=false;
            tgContour contour = subject.GetContour( i );
            
            if (contour.GetSize() < 3) {
                SG_LOG(SG_GENERAL, SG_DEBUG, "Polygon with less than 3 points " << contour.GetSize() );
                skip_ring=true;
            }
            
            // FIXME: Current we ignore the hole-flag and instead assume
            //        that the first ring is not a hole and the rest
            //        are holes
            OGRLinearRing ring;
            OGRPoint      point;
            for (unsigned int pt = 0; pt < contour.GetSize(); pt++) {
                point.setX( contour.GetNode(pt).getLongitudeDeg() );
                point.setY( contour.GetNode(pt).getLatitudeDeg() );
                point.setZ( 0.0 );
                ring.addPoint(&point);
            }
            ring.closeRings();
            
            if (!skip_ring) {
                polygon->addRingDirectly(&ring);
            }
            
            OGRFeature* feature = NULL;
            feature = new OGRFeature( l_id->GetLayerDefn() );
            feature->SetField("ID", description.c_str());
            feature->SetGeometry(polygon);
            if( l_id->CreateFeature( feature ) != OGRERR_NONE )
            {
                SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
            }
            OGRFeature::DestroyFeature(feature);
        }
    } else if ( withTriangles ) {
        for ( unsigned int i = 0; i < subject.Triangles(); i++ ) {
            tgTriangle triangle = subject.GetTriangle( i );
            OGRLineString ogr_triangle;
            OGRPoint point;
            
            // FIXME: Current we ignore the hole-flag and instead assume
            //        that the first ring is not a hole and the rest
            //        are holes
            for (unsigned int pt = 0; pt < 3; pt++) {                
                point.setX( triangle.GetNode(pt).getLongitudeDeg() );
                point.setY( triangle.GetNode(pt).getLatitudeDeg() );
                point.setZ( 0.0 );
                ogr_triangle.addPoint(&point);
            }
            
            // add the first point again
            point.setX( triangle.GetNode(0).getLongitudeDeg() );
            point.setY( triangle.GetNode(0).getLatitudeDeg() );
            point.setZ( 0.0 );
            ogr_triangle.addPoint(&point);
            
            OGRFeature* feature = NULL;
            feature = new OGRFeature( l_id->GetLayerDefn() );
            feature->SetField("ID", description.c_str());
            feature->SetGeometry(&ogr_triangle);
            if( l_id->CreateFeature( feature ) != OGRERR_NONE )
            {
                SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
            }
            OGRFeature::DestroyFeature(feature);
        }
    } else {
        for ( unsigned int i = 0; i < subject.Contours(); i++ ) {
            tgContour contour = subject.GetContour( i );
            OGRLineString ogr_contour;
            OGRPoint point;
            
            if (contour.GetSize() < 3) {
                SG_LOG(SG_GENERAL, SG_DEBUG, "Polygon with less than 3 points");
            }
            
            // FIXME: Current we ignore the hole-flag and instead assume
            //        that the first ring is not a hole and the rest
            //        are holes
            for (unsigned int pt = 0; pt < contour.GetSize(); pt++) {                
                point.setX( contour.GetNode(pt).getLongitudeDeg() );
                point.setY( contour.GetNode(pt).getLatitudeDeg() );
                point.setZ( 0.0 );
                ogr_contour.addPoint(&point);                        
            }
            
            // add the first point again
            point.setX( contour.GetNode(0).getLongitudeDeg() );
            point.setY( contour.GetNode(0).getLatitudeDeg() );
            point.setZ( 0.0 );
            ogr_contour.addPoint(&point);
            
            OGRFeature* feature = NULL;
            feature = new OGRFeature( l_id->GetLayerDefn() );
            feature->SetField("ID", description.c_str());
            feature->SetGeometry(&ogr_contour);
            if( l_id->CreateFeature( feature ) != OGRERR_NONE )
            {
                SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
            }
            OGRFeature::DestroyFeature(feature);
        }
    }    
}

void tgShapefile::FromPolygon( const tgPolygon& subject, bool asPolygon, bool withTriangles, const std::string& datasource, const std::string& layer, const std::string& description )
{
    if ( asPolygon && withTriangles ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromPolygon Can't create shapefile asPolygon with triangles" );
        return;
    }
    
    void* ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
    if ( !ds_id ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromPolygon open datasource failed. datasource: " << datasource << " layer: " << layer << " description: " << description );
    }
    
    OGRLayer* l_id;    
    if ( asPolygon ) {
        l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_POLY );
    } else {
        l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_LINE );
    }
    
    FromPolygon( l_id, subject, asPolygon, withTriangles, description );
    
    // close after each write
    ds_id = tgShapefile::CloseDatasource( ds_id );
}

void tgShapefile::FromPolygonList( const std::vector<tgPolygon>& list, bool asPolygon, bool withTriangles, const std::string& datasource, const std::string& layer, const std::string& description )
{
    if ( asPolygon && withTriangles ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromPolygon Can't create shapefile asPolygon with triangles" );
        return;
    }
    
    if ( !list.empty() ) {    
        void*          ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
        if ( !ds_id ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromPolygonList open datasource failed. datasource: " << datasource << " layer: " << layer << " description: " << description );
        }

        OGRLayer* l_id;
        if ( asPolygon ) {
            l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_POLY );
        } else {
            l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_LINE );
        }
        
        char poly_desc[64];
        for ( unsigned int i=0; i<list.size(); i++ ) {
            sprintf(poly_desc, "%s_%d", description.c_str(), i+1);
            tgShapefile::FromPolygon( (void *)l_id, list[i], asPolygon, withTriangles, poly_desc );
        }
        
        // close after each write
        ds_id = tgShapefile::CloseDatasource( ds_id );        
    }
}

void tgShapefile::FromGeod( const SGGeod& geode, const std::string& datasource, const std::string& layer, const std::string& description )
{
    void* ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
    if ( !ds_id ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromGeod open datasource failed. datasource: " << datasource << " layer: " << layer << " description: " << description );
    }
    
    OGRLayer* l_id = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_POINT );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenLayer returned " << (unsigned long)l_id);
    
    OGRPoint point;
    
    point.setX( geode.getLongitudeDeg() );
    point.setY( geode.getLatitudeDeg() );
    point.setZ( 0.0 );
        
    OGRFeature* feature = NULL;
    feature = new OGRFeature( l_id->GetLayerDefn() );
    feature->SetField("ID", description.c_str());
    feature->SetGeometry(&point);
      
    if( l_id->CreateFeature( feature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(feature);
    
    // close after each write
    ds_id = tgShapefile::CloseDatasource( ds_id );    
}

void tgShapefile::FromGeodList( const std::vector<SGGeod>& list, bool show_dir, const std::string& datasource, const std::string& layer, const std::string& description )
{
    if ( !list.empty() ) {    
        char geod_desc[64];

        for ( unsigned int i=0; i<list.size(); i++ ) {
            sprintf(geod_desc, "%s_%d", description.c_str(), i+1);
            if ( show_dir ) {
                if ( i < list.size()-1 ) {
                    tgShapefile::FromSegment( tgSegment( list[i], list[i+1] ), true, datasource, layer, geod_desc );
                }
            } else {
                tgShapefile::FromGeod( list[i], datasource, layer, geod_desc );
            }
        }
    }
}

void tgShapefile::FromSegment( void* lid, const tgSegment& subject, bool show_dir, const std::string& description )
{
    //OGRLineString* line = new OGRLineString();
    OGRLineString line;
    OGRLayer* l_id = (OGRLayer *)lid;
    //OGRPoint* start = new OGRPoint;
    OGRPoint start;
    
    SGGeod geodStart = subject.GetGeodStart();
    SGGeod geodEnd   = subject.GetGeodEnd();
    
    start.setX( geodStart.getLongitudeDeg() );
    start.setY( geodStart.getLatitudeDeg() );
    start.setZ( 0.0 );
    
    line.addPoint(&start);

    if ( show_dir ) {
        SGGeod lArrow = SGGeodesy::direct( geodEnd, SGMiscd::normalizePeriodic(0, 360, subject.GetHeading()+190), 0.2 );
        SGGeod rArrow = SGGeodesy::direct( geodEnd, SGMiscd::normalizePeriodic(0, 360, subject.GetHeading()+170), 0.2 );
        
        // OGRPoint* end = new OGRPoint;
        OGRPoint end;
        end.setX( geodEnd.getLongitudeDeg() );
        end.setY( geodEnd.getLatitudeDeg() );
        end.setZ( 0.0 );
        line.addPoint(&end);
        
        // OGRPoint* lend = new OGRPoint;
        OGRPoint lend;
        lend.setX( lArrow.getLongitudeDeg() );
        lend.setY( lArrow.getLatitudeDeg() );
        lend.setZ( 0.0 );
        line.addPoint(&lend);
        
        //OGRPoint* rend = new OGRPoint;
        OGRPoint rend;
        rend.setX( rArrow.getLongitudeDeg() );
        rend.setY( rArrow.getLatitudeDeg() );
        rend.setZ( 0.0 );
        line.addPoint(&rend);
        
        // finish the arrow
        line.addPoint(&end);
    } else {        
        // OGRPoint* end = new OGRPoint;
        OGRPoint end;
        end.setX( geodEnd.getLongitudeDeg() );
        end.setY( geodEnd.getLatitudeDeg() );
        end.setZ( 0.0 );
        line.addPoint(&end);
    }
    
    OGRFeature* feature = NULL;
    feature = OGRFeature::CreateFeature( l_id->GetLayerDefn() );    
    feature->SetField("ID", description.c_str());
    feature->SetGeometry( &line ); 

    if( l_id->CreateFeature( feature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(feature);
}

void tgShapefile::FromSegment( const tgSegment& subject, bool show_dir, const std::string& datasource, const std::string& layer, const std::string& description )
{
    void* ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
    if ( !ds_id ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromSegment open datasource failed. datasource: " << datasource << " layer: " << layer << " description: " << description );
    }
    
    OGRLayer* l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_LINE );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenLayer returned " << (unsigned long)l_id);

    FromSegment( (void *)l_id, subject, show_dir, description );
    
    // close after each write
    ds_id = tgShapefile::CloseDatasource( ds_id );
}

void tgShapefile::FromSegmentList( const std::vector<tgSegment>& list, bool show_dir, const std::string& datasource, const std::string& layer, const std::string& description )
{
    if ( !list.empty() ) {    
        void* ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
        if ( !ds_id ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromSegment open datasource failed. datasource: " << datasource << " layer: " << layer << " description: " << description );
        }
        
        OGRLayer* l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_LINE );
        SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenLayer returned " << (unsigned long)l_id);
        
        char seg_desc[64];
        for ( unsigned int i=0; i<list.size(); i++ ) {
            sprintf(seg_desc, "%s_%d", description.c_str(), i+1);
            tgShapefile::FromSegment( (void *)l_id, list[i], show_dir, seg_desc );
        }

        // close after each write
        ds_id = tgShapefile::CloseDatasource( ds_id );        
    }
}

void tgShapefile::FromRay( const tgRay& subject, const std::string& datasource, const std::string& layer, const std::string& description )
{
    void*  ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
    if ( !ds_id ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromRay open datasource failed. datasource: " << datasource << " layer: " << layer << " description: " << description );
    }
        
    OGRLayer* l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_LINE );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenLayer returned " << (unsigned long)l_id);
    
    OGRLineString line;
    OGRPoint      start;

    SGGeod geodStart = subject.GetGeodStart();
    SGGeod geodEnd   = subject.GetGeodEnd();
    
    start.setX( geodStart.getLongitudeDeg() );
    start.setY( geodStart.getLatitudeDeg() );
    start.setZ( 0.0 );
    
    line.addPoint(&start);

    SGGeod lArrow = SGGeodesy::direct( geodEnd, SGMiscd::normalizePeriodic(0, 360, subject.GetHeading()+190), 0.2 );
    SGGeod rArrow = SGGeodesy::direct( geodEnd, SGMiscd::normalizePeriodic(0, 360, subject.GetHeading()+170), 0.2 );

    OGRPoint end;
    end.setX( geodEnd.getLongitudeDeg() );
    end.setY( geodEnd.getLatitudeDeg() );
    end.setZ( 0.0 );
    line.addPoint(&end);
    
    OGRPoint lend;
    lend.setX( lArrow.getLongitudeDeg() );
    lend.setY( lArrow.getLatitudeDeg() );
    lend.setZ( 0.0 );
    line.addPoint(&lend);
    
    OGRPoint rend;
    rend.setX( rArrow.getLongitudeDeg() );
    rend.setY( rArrow.getLatitudeDeg() );
    rend.setZ( 0.0 );
    line.addPoint(&rend);
    
    // finish the arrow
    line.addPoint(&end);

    OGRFeature* feature = NULL;
    feature = OGRFeature::CreateFeature( l_id->GetLayerDefn() );    
    feature->SetField("ID", description.c_str());
    feature->SetGeometry( &line ); 

    if( l_id->CreateFeature( feature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(feature);
    
    // close after each write
    ds_id = tgShapefile::CloseDatasource( ds_id );
}

void tgShapefile::FromRayList( const std::vector<tgRay>& list, const std::string& datasource, const std::string& layer, const std::string& description )
{
    if ( !list.empty() ) {    
        char ray_desc[64];

        for ( unsigned int i=0; i<list.size(); i++ ) {
            sprintf(ray_desc, "%s_%d", description.c_str(), i+1);
            tgShapefile::FromRay( list[i], datasource, layer, ray_desc );
        }        
    }
}

void tgShapefile::FromLine( const tgLine& subject, const std::string& datasource, const std::string& layer, const std::string& description )
{
    void* ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
    if ( !ds_id ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromLine open datasource failed. datasource: " << datasource << " layer: " << layer << " description: " << description );
    }
        
    OGRLayer* l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_LINE );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenLayer returned " << (unsigned long)l_id);
        
    OGRLineString line;
    OGRPoint      start;

    SGGeod geodStart = subject.GetGeodStart();
    SGGeod geodEnd   = subject.GetGeodEnd();
    
    start.setX( geodStart.getLongitudeDeg() );
    start.setY( geodStart.getLatitudeDeg() );
    start.setZ( 0.0 );
    
    line.addPoint(&start);
    
    SGGeod slArrow = SGGeodesy::direct( geodStart, SGMiscd::normalizePeriodic(0, 360, subject.GetHeading()-10), 0.2 );
    SGGeod srArrow = SGGeodesy::direct( geodStart, SGMiscd::normalizePeriodic(0, 360, subject.GetHeading()+10), 0.2 );
    
    OGRPoint lstart;
    lstart.setX( slArrow.getLongitudeDeg() );
    lstart.setY( slArrow.getLatitudeDeg() );
    lstart.setZ( 0.0 );
    line.addPoint(&lstart);
    
    OGRPoint rstart;
    rstart.setX( srArrow.getLongitudeDeg() );
    rstart.setY( srArrow.getLatitudeDeg() );
    rstart.setZ( 0.0 );
    line.addPoint(&rstart);
    
    // start back at the beginning
    line.addPoint(&start);

    SGGeod elArrow = SGGeodesy::direct( geodEnd, SGMiscd::normalizePeriodic(0, 360, subject.GetHeading()+170), 0.2 );
    SGGeod erArrow = SGGeodesy::direct( geodEnd, SGMiscd::normalizePeriodic(0, 360, subject.GetHeading()+190), 0.2 );
    
    OGRPoint end;
    end.setX( geodEnd.getLongitudeDeg() );
    end.setY( geodEnd.getLatitudeDeg() );
    end.setZ( 0.0 );
    line.addPoint(&end);
    
    OGRPoint lend;
    lend.setX( elArrow.getLongitudeDeg() );
    lend.setY( elArrow.getLatitudeDeg() );
    lend.setZ( 0.0 );
    line.addPoint(&lend);

    OGRPoint rend;
    rend.setX( erArrow.getLongitudeDeg() );
    rend.setY( erArrow.getLatitudeDeg() );
    rend.setZ( 0.0 );
    line.addPoint(&rend);

    // finish the arrow
    line.addPoint(&end);
    
    OGRFeature* feature = NULL;
    feature = OGRFeature::CreateFeature( l_id->GetLayerDefn() );    
    feature->SetField("ID", description.c_str());
    feature->SetGeometry( &line ); 

    if( l_id->CreateFeature( feature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(feature);
    
    // close after each write
    ds_id = tgShapefile::CloseDatasource( ds_id );
}

void tgShapefile::FromLineList( const std::vector<tgLine>& list, const std::string& datasource, const std::string& layer, const std::string& description )
{
    if ( !list.empty() ) {
        char line_desc[64];
    
        for ( unsigned int i=0; i<list.size(); i++ ) {
            sprintf(line_desc, "%s_%d", description.c_str(), i+1);
            tgShapefile::FromLine( list[i], datasource, layer, line_desc );
        }
    }
}

void tgShapefile::FromRectangle( const tgRectangle& subject, const std::string& datasource, const std::string& layer, const std::string& description )
{
    void* ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
    if ( !ds_id ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromRectangle open datasource failed. datasource: " << datasource << " layer: " << layer << " description: " << description );
    }
    
    OGRLayer* l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_LINE );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenLayer returned " << (unsigned long)l_id);
        
    OGRLineString ogr_bb;

    SGGeod  min = subject.getMin();
    SGGeod  max = subject.getMax();
    
    OGRPoint point;
    point.setX( min.getLongitudeDeg() );
    point.setY( min.getLatitudeDeg() );
    point.setZ( 0.0 );
    ogr_bb.addPoint(&point);

    point.setX( max.getLongitudeDeg() );
    point.setY( min.getLatitudeDeg() );
    ogr_bb.addPoint(&point);

    point.setX( max.getLongitudeDeg() );
    point.setY( max.getLatitudeDeg() );
    ogr_bb.addPoint(&point);

    point.setX( min.getLongitudeDeg() );
    point.setY( max.getLatitudeDeg() );
    ogr_bb.addPoint(&point);
    
    point.setX( min.getLongitudeDeg() );
    point.setY( min.getLatitudeDeg() );
    ogr_bb.addPoint(&point);
    
    OGRFeature* feature = NULL;
    feature = new OGRFeature( l_id->GetLayerDefn() );
    feature->SetField("ID", description.c_str());
    feature->SetGeometry(&ogr_bb);
    if( l_id->CreateFeature( feature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(feature);
    
    // close after each write
    ds_id = tgShapefile::CloseDatasource( ds_id );
}

tgPolygon tgShapefile::ToPolygon( const void* subject )
{
    const OGRPolygon* ogr_poly = (const OGRPolygon*)subject;
    OGRLinearRing const *ring = ogr_poly->getExteriorRing();

    tgContour contour;
    tgPolygon result;

    for (int i = 0; i < ring->getNumPoints(); i++) {
        contour.AddNode( SGGeod::fromDegM( ring->getX(i), ring->getY(i), ring->getZ(i)) );
    }
    contour.SetHole( false );
    result.AddContour( contour );

    // then add the inner rings
    for ( int j = 0 ; j < ogr_poly->getNumInteriorRings(); j++ ) {
        ring = ogr_poly->getInteriorRing( j );
        contour.Erase();

        for (int i = 0; i < ring->getNumPoints(); i++) {
            contour.AddNode( SGGeod::fromDegM( ring->getX(i), ring->getY(i), ring->getZ(i)) );
        }
        contour.SetHole( true );
        result.AddContour( contour );
    }
    result.SetTexMethod( TG_TEX_BY_GEODE );

    return result;
}
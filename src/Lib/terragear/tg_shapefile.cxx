#include <ogrsf_frmts.h> 

#include <simgear/debug/logstream.hxx>
#include <simgear/misc/sg_path.hxx>

#include "tg_shapefile.hxx"

bool tgShapefile::initialized = false;

void* tgShapefile::OpenDatasource( const char* datasource_name )
{
    GDALDataset*    poDS;
    GDALDriver*     poDriver;
    const char*     format_name = "ESRI Shapefile";
    
    SG_LOG( SG_GENERAL, SG_DEBUG, "Open Datasource: " << datasource_name );
    
    if (!tgShapefile::initialized) {
        GDALAllRegister();
        tgShapefile::initialized = true;
    }
    
    SGPath sgp( datasource_name );
    sgp.create_dir( 0755 );
    
    poDriver = GetGDALDriverManager()->GetDriverByName( format_name );
    if ( !poDriver ) {
        SG_LOG( SG_GENERAL, SG_ALERT, "Unknown datasource format driver: " << format_name );
        exit(1);
    }
    
    poDS = poDriver->Create( datasource_name, 0, 0, 0, GDT_Unknown, NULL );
    if ( !poDS ) {
        SG_LOG( SG_GENERAL, SG_ALERT, "Unable to open or create datasource: " << datasource_name );
    }
    
    return (void*)poDS;
}

void* tgShapefile::OpenLayer( void* ds_id, const char* layer_name, shapefile_layer_t type) {
    GDALDataset* poDS = ( GDALDataset * )ds_id;
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
    
    layer = poDS->GetLayerByName( layer_name );
    
    if ( !layer ) {
        layer = poDS->CreateLayer( layer_name, &srs, ogr_type, NULL );
        
        OGRFieldDefn descriptionField( "tg_desc", OFTString );
        descriptionField.SetWidth( 128 );
        if( layer->CreateField( &descriptionField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_desc' failed" );
        }
        
        OGRFieldDefn idField( "tg_id", OFTString );
        idField.SetWidth( 128 );
        if( layer->CreateField( &idField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_id' failed" );
        }
        
        OGRFieldDefn preserve3DField( "tg_3d", OFTInteger );
        if( layer->CreateField( &preserve3DField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_3d' failed" );
        }
        
        OGRFieldDefn materialField( "tg_mat", OFTString );
        materialField.SetWidth( 32 );
        if( layer->CreateField( &materialField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_material' failed" );
        }
        
        OGRFieldDefn texMethodField( "tg_texmeth", OFTInteger );
        if( layer->CreateField( &texMethodField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_tex_method' failed" );
        }
        
        OGRFieldDefn texRefLonField( "tg_reflon", OFTReal );
        if( layer->CreateField( &texRefLonField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_tp_ref_lon' failed" );
        }
        
        OGRFieldDefn texRefLatField( "tg_reflat", OFTReal );
        if( layer->CreateField( &texRefLatField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_tp_ref_lat' failed" );
        }
        
        OGRFieldDefn texHeadingField( "tg_heading", OFTReal );
        if( layer->CreateField( &texHeadingField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_tp_heading' failed" );
        }
        
        OGRFieldDefn texWidthField( "tg_width", OFTReal );
        if( layer->CreateField( &texWidthField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_tp_width' failed" );
        }
        
        OGRFieldDefn texLengthField( "tg_length", OFTReal );
        if( layer->CreateField( &texLengthField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_tp_length' failed" );
        }
        
        OGRFieldDefn texMinUField( "tg_minu", OFTReal );
        if( layer->CreateField( &texMinUField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_tp_minu' failed" );
        }
        
        OGRFieldDefn texMinVField( "tg_minv", OFTReal );
        if( layer->CreateField( &texMinVField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_tp_minv' failed" );
        }
        
        OGRFieldDefn texMaxUField( "tg_maxu", OFTReal );
        if( layer->CreateField( &texMaxUField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_tp_maxu' failed" );
        }
        
        OGRFieldDefn texMaxVField( "tg_maxv", OFTReal );
        if( layer->CreateField( &texMaxVField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_tp_maxv' failed" );
        }
        
        OGRFieldDefn texMinClipUField( "tg_mincu", OFTReal );
        if( layer->CreateField( &texMinClipUField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_tp_min_clipu' failed" );
        }
        
        OGRFieldDefn texMinClipVField( "tg_mincv", OFTReal );
        if( layer->CreateField( &texMinClipVField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_tp_min_clipv' failed" );
        }
        
        OGRFieldDefn texMaxClipUField( "tg_maxcu", OFTReal );
        if( layer->CreateField( &texMaxClipUField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_tp_max_clipu' failed" );
        }
        
        OGRFieldDefn texMaxClipVField( "tg_maxcv", OFTReal );
        if( layer->CreateField( &texMaxClipVField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_tp_max_clipv' failed" );
        }
        
        OGRFieldDefn texCenterLatField( "tg_clat", OFTReal );
        if( layer->CreateField( &texCenterLatField ) != OGRERR_NONE ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Creation of field 'tg_tp_center_lat' failed" );
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
    GDALDataset* poDS = ( GDALDataset * )ds_id;
    GDALClose( poDS );
    
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
        feature->SetField("tg_desc", description.c_str());
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

void tgShapefile::FromContour( void *lid, const tgContour& subject, bool asPolygon, bool withNumber, const std::string& description )
{
    OGRLayer* l_id = (OGRLayer *)lid;
    
    if (subject.GetSize() < 3) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Polygon with less than 3 points");
    } else {
        if ( withNumber ) {
            OGRPoint point;
            char desc[128];
            
            for (unsigned int pt = 0; pt < subject.GetSize(); pt++) {
                sprintf( desc, "%s_%d", description.c_str(), pt );
                
                point.setX( CGAL::to_double( subject.GetPoint(pt).x() ) );
                point.setY( CGAL::to_double( subject.GetPoint(pt).y() ) );
                point.setZ( 0.0 );
            
                OGRFeature* feature = NULL;
                feature = new OGRFeature( l_id->GetLayerDefn() );
                feature->SetField("tg_desc", desc);
                feature->SetGeometry(&point);
            
                if( l_id->CreateFeature( feature ) != OGRERR_NONE )
                {
                    SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
                }
                OGRFeature::DestroyFeature(feature);
            }
        } else if ( asPolygon ) {
            OGRPolygon    polygon;
            OGRLinearRing ring;
            OGRPoint      point;
            
            for (unsigned int pt = 0; pt < subject.GetSize(); pt++) {
                point.setX( CGAL::to_double( subject.GetPoint(pt).x() ) );
                point.setY( CGAL::to_double( subject.GetPoint(pt).y() ) );
                point.setZ( 0.0 );
                ring.addPoint(&point);                
            }
            ring.closeRings();
            polygon.addRing(&ring);

            OGRFeature* feature = NULL;
            feature = new OGRFeature( l_id->GetLayerDefn() );
            feature->SetField("tg_desc", description.c_str());
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
                point.setX( CGAL::to_double( subject.GetPoint(pt).x() ) );
                point.setY( CGAL::to_double( subject.GetPoint(pt).y() ) );
                point.setZ( 0.0 );
                ogr_contour.addPoint(&point);                
            }

            point.setX( CGAL::to_double( subject.GetPoint(0).x() ) );
            point.setY( CGAL::to_double( subject.GetPoint(0).y() ) );
            point.setZ( 0.0 );
            ogr_contour.addPoint(&point);
                        
            OGRFeature* feature = NULL;
            feature = new OGRFeature( l_id->GetLayerDefn() );
            feature->SetField("tg_desc", description.c_str());
            feature->SetGeometry(&ogr_contour);
            if( l_id->CreateFeature( feature ) != OGRERR_NONE )
            {
                SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
            }
            OGRFeature::DestroyFeature(feature);            
        }
    }
}

void tgShapefile::FromContour( const tgContour& subject, bool asPolygon, bool withNumber, const std::string& datasource, const std::string& layer, const std::string& description )
{
    void* ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
    if ( !ds_id ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromSegment open datasource failed. datasource: " << datasource << " layer: " << layer << " description: " << description );
    }

    OGRLayer* l_id;    
    if ( withNumber ) {
        l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_POINT );        
    } else if ( asPolygon ) {
        l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_POLY );
    } else {
        l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_LINE );
    }

    FromContour( (void *)l_id, subject, asPolygon, withNumber, description );
    
    // close after each write
    ds_id = tgShapefile::CloseDatasource( ds_id );
}

void tgShapefile::FromContourList( const std::vector<tgContour>& list, bool asPolygon, bool withNumber, const std::string& datasource, const std::string& layer, const std::string& description )
{
    if ( !list.empty() ) {    
        void* ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
        if ( !ds_id ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromContourList open datasource failed. datasource: " << datasource << " layer: " << layer << " description: " << description );
        }

        OGRLayer* l_id;
        if ( withNumber ) {
            l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_POINT );        
        } else if ( asPolygon ) {
            l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_POLY );
        } else {
            l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_LINE );
        }
        
        char cont_desc[64];
        for ( unsigned int i=0; i<list.size(); i++ ) {
            sprintf(cont_desc, "%s_%d", description.c_str(), i+1);
            tgShapefile::FromContour( (void *)l_id, list[i], asPolygon, withNumber, cont_desc );
        }
        
        // close after each write
        ds_id = tgShapefile::CloseDatasource( ds_id );
    }
}

static void SetTPFields( OGRFeature* feature, const tgPolygon& subject )
{
    feature->SetField("tg_reflon", subject.GetTexParams().ref.getLongitudeDeg() );
    feature->SetField("tg_reflat", subject.GetTexParams().ref.getLatitudeDeg() );
    feature->SetField("tg_heading", subject.GetTexParams().heading );
    feature->SetField("tg_width", subject.GetTexParams().width );
    feature->SetField("tg_length", subject.GetTexParams().length );
    feature->SetField("tg_minu", subject.GetTexParams().minu );
    feature->SetField("tg_minv", subject.GetTexParams().minv );
    feature->SetField("tg_maxu", subject.GetTexParams().maxu );
    feature->SetField("tg_maxv", subject.GetTexParams().maxv );
    feature->SetField("tg_mincu", subject.GetTexParams().min_clipu );
    feature->SetField("tg_mincv", subject.GetTexParams().min_clipv );
    feature->SetField("tg_maxcu", subject.GetTexParams().max_clipu );
    feature->SetField("tg_maxcv", subject.GetTexParams().max_clipv );
}

static tgTexParams GetTPFields( tgTexMethod tm, OGRFeature* feature )
{
    tgTexParams tp;
    
    if ( tm == TG_TEX_BY_GEODE ) {
        tp.center_lat = feature->GetFieldAsDouble("tg_clat");
    } else {
        tp.ref.setLongitudeDeg( feature->GetFieldAsDouble("tg_reflon") );
        tp.ref.setLatitudeDeg( feature->GetFieldAsDouble("tg_reflat") );
        tp.heading   = feature->GetFieldAsDouble("tg_heading");
        tp.width     = feature->GetFieldAsDouble("tg_width");
        tp.length    = feature->GetFieldAsDouble("tg_length");
        tp.minu      = feature->GetFieldAsDouble("tg_minu");
        tp.minv      = feature->GetFieldAsDouble("tg_minv");
        tp.maxu      = feature->GetFieldAsDouble("tg_maxu");
        tp.maxv      = feature->GetFieldAsDouble("tg_maxv");
        tp.min_clipu = feature->GetFieldAsDouble("tg_mincu");
        tp.min_clipv = feature->GetFieldAsDouble("tg_mincv");
        tp.max_clipu = feature->GetFieldAsDouble("tg_maxcu");
        tp.max_clipv = feature->GetFieldAsDouble("tg_maxcv");
    }
    
    return tp;
}


void tgShapefile::FromPolygon( void *lid, void *pid, const tgPolygon& subject, bool asPolygon, bool withTriangles, const std::string& description )
{    
    OGRLayer* poLayer   = (OGRLayer *)lid;
    OGRLayer* poPtLayer = (OGRLayer *)pid;
    
    if ( asPolygon ) {
        OGRPolygon polygon;
        
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
            
            // make sure boundaries are clockwise, holes are counterclockwise
            if ( (!contour.GetHole()) && (!contour.IsClockwise()) ) {
                SG_LOG(SG_GENERAL, SG_ALERT, " reverse boundary " );
                contour.Reverse();
            } else if ( contour.GetHole() && contour.IsClockwise() ) {
                SG_LOG(SG_GENERAL, SG_ALERT, " reverse hole " );
                contour.Reverse();
            }
            
            for (unsigned int pt = 0; pt < contour.GetSize(); pt++) {
//              point.setX( contour.GetNode(pt).getLongitudeDeg() );
//              point.setY( contour.GetNode(pt).getLatitudeDeg() );
//              point.setX( contour.GetNode(pt).getLongitudeDeg() );
//              point.setY( contour.GetNode(pt).getLatitudeDeg() );
//              if ( subject.GetPreserve3D() ) {
//                  point.setZ( contour.GetNode(pt).getElevationM() );                    
//              } else {
//                  point.setZ( 0.0 );
//              }

                point.setX( CGAL::to_double( contour.GetPoint(pt).x() ) );
                point.setY( CGAL::to_double( contour.GetPoint(pt).y() ) );
                point.setZ( 0.0 );
                
                ring.addPoint(&point);
            }
            ring.closeRings();
            
            if (!skip_ring) {
                polygon.addRing(&ring);
            }
        }
        
        OGRFeature* poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
        
        char id[16];
        sprintf( id, "%06d", subject.GetId() );
        poFeature->SetField("tg_id", id );
        poFeature->SetField("tg_mat", subject.GetMaterial().c_str() );
        poFeature->SetField("tg_texmeth", subject.GetTexMethod() );
        poFeature->SetField("tg_3d", subject.GetPreserve3D() );
        
        switch( subject.GetTexMethod() ) {
            case TG_TEX_BY_GEODE:
                poFeature->SetField("tg_clat", subject.GetTexParams().center_lat);
                break;
                
            case TG_TEX_BY_TPS_NOCLIP:
                SetTPFields( poFeature, subject );
                break;
                
            case TG_TEX_BY_TPS_CLIPU:
                SetTPFields( poFeature, subject );
                break;

            case TG_TEX_BY_TPS_CLIPV:
                SetTPFields( poFeature, subject );
                break;

            case TG_TEX_BY_TPS_CLIPUV:
                SetTPFields( poFeature, subject );
                break;

            case TG_TEX_1X1_ATLAS:                    
                SetTPFields( poFeature, subject );
                break;
                
            default:
                break;
        }
        
        poFeature->SetGeometry(&polygon);
        if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
        {
            SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
        }
        OGRFeature::DestroyFeature(poFeature);
        
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
            feature = new OGRFeature( poLayer->GetLayerDefn() );
            feature->SetField("tg_desc", description.c_str());
            feature->SetGeometry(&ogr_triangle);
            if( poLayer->CreateFeature( feature ) != OGRERR_NONE )
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
            
            OGRFeature* poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
            poFeature->SetField("tg_desc", description.c_str());
            poFeature->SetGeometry(&ogr_contour);
            if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
            {
                SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
            }
            OGRFeature::DestroyFeature(poFeature);
        }
    }
    
    // then add the interior points
    std::vector<tgPoint> interiorPoints = subject.GetInteriorPoints();
    if ( poPtLayer && !interiorPoints.empty() ) {
        OGRPoint point;

        for ( unsigned int i=0; i<interiorPoints.size(); i++ ) {
            point.setX( CGAL::to_double( interiorPoints[i].x() ) );
            point.setY( CGAL::to_double( interiorPoints[i].y() ) );
            point.setZ( 0.0 );
    
            OGRFeature* feature = NULL;
            feature = new OGRFeature( poPtLayer->GetLayerDefn() );
            feature->SetGeometry(&point);
    
            if( poPtLayer->CreateFeature( feature ) != OGRERR_NONE )
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
    
    OGRLayer* pl_id;    
    if ( asPolygon ) {
        pl_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_POLY );
    } else {
        pl_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_LINE );
    }
    char point_layer[256];
    sprintf( point_layer, "%s_pts", layer.c_str() );
    OGRLayer* ipl_id = (OGRLayer *)tgShapefile::OpenLayer( ds_id, point_layer, LT_POINT );
    
    FromPolygon( pl_id, ipl_id, subject, asPolygon, withTriangles, description );
    
    // close after each write
    ds_id = tgShapefile::CloseDatasource( ds_id );
}

void tgShapefile::FromPolygonList( const std::vector<tgPolygon>& list, bool asPolygon, bool withTriangles, const std::string& datasource, const std::string& layer, const std::string& description )
{
    if ( asPolygon && withTriangles ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromPolygonList Can't create shapefile asPolygon with triangles" );
        return;
    }
    
    if ( !list.empty() ) {    
        void*          ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
        if ( !ds_id ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromPolygonList open datasource failed. datasource: " << datasource << " layer: " << layer << " description: " << description );
        }

        OGRLayer* pl_id;
        if ( asPolygon ) {
            pl_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_POLY );
        } else {
            pl_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_LINE );
        }
        char point_layer[256];
        sprintf( point_layer, "%s_pts", layer.c_str() );
        OGRLayer* ipl_id = (OGRLayer *)tgShapefile::OpenLayer( ds_id, point_layer, LT_POINT );
        
        char poly_desc[64];
        for ( unsigned int i=0; i<list.size(); i++ ) {
            sprintf(poly_desc, "%s_%d", description.c_str(), i+1);
            tgShapefile::FromPolygon( (void *)pl_id, ipl_id, list[i], asPolygon, withTriangles, poly_desc );
        }
        
        // close after each write
        ds_id = tgShapefile::CloseDatasource( ds_id );        
    }
}

void tgShapefile::FromPolygonList( const std::vector<tgPolygon>& list, const std::string& datasource )
{
    if ( !list.empty() ) {    
        void*          ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
        if ( !ds_id ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromPolygonList open datasource failed. datasource: " << datasource );
        } else {
            for ( unsigned int i=0; i<list.size(); i++ ) {
                char poly_desc[128];
                sprintf(poly_desc, "%s_%d", list[i].GetMaterial().c_str(), list[i].GetId() );
                OGRLayer* pl_id;
                
                if ( list[i].GetMaterial().length() == 0 ) {
                    SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromPolygonList poly has no material - skipping: " );
                    continue;
                } else {
                    pl_id = (OGRLayer *)tgShapefile::OpenLayer( ds_id, list[i].GetMaterial().c_str(), LT_POLY );
                }
                char point_layer[256];
                sprintf( point_layer, "%s_pts", list[i].GetMaterial().c_str() );
                OGRLayer* ipl_id = (OGRLayer *)tgShapefile::OpenLayer( ds_id, point_layer, LT_POINT );
                
                tgShapefile::FromPolygon( pl_id, ipl_id, list[i], true, false, poly_desc );
            }               
        }

        // close after each write
        ds_id = tgShapefile::CloseDatasource( ds_id );
    }
}

void tgShapefile::FromGeod( void* lid, const SGGeod& geode, const std::string& description )
{
    OGRLayer* l_id = (OGRLayer *)lid;
    OGRPoint point;
    
    point.setX( geode.getLongitudeDeg() );
    point.setY( geode.getLatitudeDeg() );
    point.setZ( 0.0 );
    
    OGRFeature* feature = NULL;
    feature = new OGRFeature( l_id->GetLayerDefn() );
    feature->SetField("tg_desc", description.c_str());
    feature->SetGeometry(&point);
    
    if( l_id->CreateFeature( feature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(feature);    
}

void tgShapefile::FromGeod( const SGGeod& geode, const std::string& datasource, const std::string& layer, const std::string& description )
{
    void* ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
    if ( !ds_id ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromGeod open datasource failed. datasource: " << datasource << " layer: " << layer << " description: " << description );
    }
    
    OGRLayer* l_id = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_POINT );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenLayer returned " << (unsigned long)l_id);
    
    FromGeod( l_id, geode, description );
    
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

void tgShapefile::FromEdgeArrPoint( void* lid, const edgeArrPoint& pt, const std::string& description )
{
    OGRLayer* l_id = (OGRLayer *)lid;
    OGRPoint point;
    
    point.setX( CGAL::to_double( pt.x() ) );
    point.setY( CGAL::to_double( pt.y() ) );
    point.setZ( 0.0 );
    
    OGRFeature* feature = NULL;
    feature = new OGRFeature( l_id->GetLayerDefn() );
    feature->SetField("tg_desc", description.c_str());
    feature->SetGeometry(&point);
    
    if( l_id->CreateFeature( feature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(feature);    
}

void tgShapefile::FromEdgeArrPoint( const edgeArrPoint& pt, const std::string& datasource, const std::string& layer, const std::string& description )
{
    void* ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
    if ( !ds_id ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromGeod open datasource failed. datasource: " << datasource << " layer: " << layer << " description: " << description );
    }
    
    OGRLayer* l_id = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_POINT );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenLayer returned " << (unsigned long)l_id);
    
    FromEdgeArrPoint( l_id, pt, description );
    
    // close after each write
    ds_id = tgShapefile::CloseDatasource( ds_id );    
}

void tgShapefile::FromEdgeArrPointList( const std::vector<edgeArrPoint>& list, bool show_dir, const std::string& datasource, const std::string& layer, const std::string& description )
{
    if ( !list.empty() ) {    
        char pt_desc[64];
        
        for ( unsigned int i=0; i<list.size(); i++ ) {
            sprintf(pt_desc, "%s_%d", description.c_str(), i+1);
            if ( show_dir ) {
                if ( i < list.size()-1 ) {
                    tgShapefile::FromEdgeArrRay( edgeArrRay( list[i], list[i+1] ), datasource, layer, pt_desc );
                }
            } else {
                tgShapefile::FromEdgeArrPoint( list[i], datasource, layer, pt_desc );
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
    feature->SetField("tg_desc", description.c_str());
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
    feature->SetField("tg_desc", description.c_str());
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
    feature->SetField("tg_desc", description.c_str());
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

#if 0
void tgShapefile::FromConstraint( void* lid, const tgConstraint& subject )
{
    OGRLayer* l_id = (OGRLayer *)lid;
    
    OGRLineString line;
    OGRPoint      start;
    
    edgeArrPoint pStart  = subject.getStart();
    SGGeod       gStart  = SGGeod::fromDeg( CGAL::to_double( pStart.x() ), CGAL::to_double( pStart.y() ) );
    edgeArrPoint pEnd    = subject.getEnd();
    SGGeod       gEnd    = SGGeod::fromDeg( CGAL::to_double( pEnd.x() ),   CGAL::to_double( pEnd.y() ) );
    
    double       heading = subject.getHeading();

    // handle rays that are generated with heading - they are too short...
    //if ( subject.getType() == tgConstraint::consRay ) {
    //    geodEnd = SGGeodesy::direct( geodStart, heading, 5.0 );
    //}
    
    start.setX( CGAL::to_double( pStart.x() ) );
    start.setY( CGAL::to_double( pStart.y() ) );
    start.setZ( 0.0 );    
    line.addPoint(&start);

    if ( subject.getType() == tgConstraint::consLine ) {
        SGGeod slArrow = SGGeodesy::direct( gStart, SGMiscd::normalizePeriodic(0, 360, heading-10), 0.2 );
        SGGeod srArrow = SGGeodesy::direct( gStart, SGMiscd::normalizePeriodic(0, 360, heading+10), 0.2 );
        
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
    }

    OGRPoint end;
    end.setX( gEnd.getLongitudeDeg() );
    end.setY( gEnd.getLatitudeDeg() );
    end.setZ( 0.0 );
    line.addPoint(&end);

    if ( subject.getType() == tgConstraint::consLine || subject.getType() == tgConstraint::consRay ) {
        SGGeod elArrow = SGGeodesy::direct( gEnd, SGMiscd::normalizePeriodic(0, 360, heading+170), 0.2 );
        SGGeod erArrow = SGGeodesy::direct( gEnd, SGMiscd::normalizePeriodic(0, 360, heading+190), 0.2 );
        
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
    }

    OGRFeature* feature = NULL;
    feature = OGRFeature::CreateFeature( l_id->GetLayerDefn() );    
    feature->SetField( "tg_desc", subject.getDescription().c_str() );
    feature->SetGeometry( &line ); 
    
    if( l_id->CreateFeature( feature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(feature);
}

void tgShapefile::FromConstraint( const tgConstraint& subject, const std::string& datasource, const std::string& layer )
{
    void* ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
    if ( !ds_id ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromConstraint open datasource failed. datasource: " << datasource << " layer: " << layer );
    }
    
    OGRLayer* l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_LINE );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenLayer returned " << (unsigned long)l_id);
    
    FromConstraint( (void *)l_id, subject );
    
    // close after each write
    ds_id = tgShapefile::CloseDatasource( ds_id );
}

void tgShapefile::FromConstraintList( const std::vector<tgConstraint>& list, const std::string& datasource, const std::string& layer  )
{
    if ( !list.empty() ) {    
        void* ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
        if ( !ds_id ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromConstraint open datasource failed. datasource: " << datasource << " layer: " << layer );
        }
        
        OGRLayer* l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_LINE );
        SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenLayer returned " << (unsigned long)l_id);
        
        for ( unsigned int i=0; i<list.size(); i++ ) {
            tgShapefile::FromConstraint( (void *)l_id, list[i] );
        }
        
        // close after each write
        ds_id = tgShapefile::CloseDatasource( ds_id );
    }
}
#endif

void tgShapefile::FromEdgeArrRay( void* lid, const edgeArrRay& subject, const std::string& description )
{
    OGRLayer* l_id = (OGRLayer *)lid;
    
    OGRLineString line;
    OGRPoint      start;

    edgeArrPoint pStart = subject.source();
    SGGeod       gStart = SGGeod::fromDeg( CGAL::to_double( pStart.x() ), CGAL::to_double( pStart.y() ) );
    
    edgeArrKernel::Direction_2 dir = subject.direction();
    double angle   = SGMiscd::rad2deg( atan2( CGAL::to_double(dir.dy()), CGAL::to_double(dir.dx()) ) );
    double heading = SGMiscd::normalizePeriodic( 0, 360, -(angle-90) );
    
    SGGeod gEnd = SGGeodesy::direct( gStart, heading, 5.0 );
    
    start.setX( CGAL::to_double( pStart.x() ) );
    start.setY( CGAL::to_double( pStart.y() ) );
    start.setZ( 0.0 );    
    line.addPoint(&start);

    OGRPoint end;
    end.setX( gEnd.getLongitudeDeg() );
    end.setY( gEnd.getLatitudeDeg() );
    end.setZ( 0.0 );
    line.addPoint(&end);

    SGGeod elArrow = SGGeodesy::direct( gEnd, SGMiscd::normalizePeriodic(0, 360, heading+170), 0.2 );
    SGGeod erArrow = SGGeodesy::direct( gEnd, SGMiscd::normalizePeriodic(0, 360, heading+190), 0.2 );
        
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
    feature->SetField( "tg_desc", description.c_str() );
    feature->SetGeometry( &line ); 
    
    if( l_id->CreateFeature( feature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(feature);
}

void tgShapefile::FromEdgeArrRay( const edgeArrRay& subject, const std::string& datasource, const std::string& layer, const std::string& description )
{
    void* ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
    if ( !ds_id ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromConstraint open datasource failed. datasource: " << datasource << " layer: " << layer );
    }
    
    OGRLayer* l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_LINE );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenLayer returned " << (unsigned long)l_id);
    
    FromEdgeArrRay( (void *)l_id, subject, description );
    
    // close after each write
    ds_id = tgShapefile::CloseDatasource( ds_id );
}

void tgShapefile::FromEdgeArrRayList( const std::vector<edgeArrRay>& list, const std::string& datasource, const std::string& layer, const std::string& description )
{
    if ( !list.empty() ) {    
        char ray_desc[64];
        
        void* ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
        if ( !ds_id ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromConstraint open datasource failed. datasource: " << datasource << " layer: " << layer );
        }
        
        OGRLayer* l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), LT_LINE );
        SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenLayer returned " << (unsigned long)l_id);
            
        for ( unsigned int i=0; i<list.size(); i++ ) {
            sprintf(ray_desc, "%s_%d", description.c_str(), i+1);
            tgShapefile::FromEdgeArrRay( (void *)l_id, list[i], ray_desc );
        }
        
        // close after each write
        ds_id = tgShapefile::CloseDatasource( ds_id );
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
    feature->SetField("tg_desc", description.c_str());
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
        contour.AddPoint( tgPoint(ring->getX(i), ring->getY(i)) );
        //contour.AddNode( SGGeod::fromDegM( ring->getX(i), ring->getY(i), ring->getZ(i)) );
    }
    contour.SetHole( false );
    result.AddContour( contour );

    // then add the inner rings
    for ( int j = 0 ; j < ogr_poly->getNumInteriorRings(); j++ ) {
        ring = ogr_poly->getInteriorRing( j );
        contour.Erase();

        for (int i = 0; i < ring->getNumPoints(); i++) {
            contour.AddPoint( tgPoint(ring->getX(i), ring->getY(i)) );
            //contour.AddNode( SGGeod::fromDegM( ring->getX(i), ring->getY(i), ring->getZ(i)) );
        }
        contour.SetHole( true );
        result.AddContour( contour );
    }
    result.SetTexMethod( TG_TEX_BY_GEODE );

    return result;
}

static void ToPolygons( const OGRFeatureDefn* poFDefn, OGRCoordinateTransformation* poCT, OGRFeature* poFeature, tgpolygon_list& polys )
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

    // grab the material, and texparams for this feature
    std::string material   = poFeature->GetFieldAsString("tg_mat");    
    tgTexMethod tmeth      = (tgTexMethod)poFeature->GetFieldAsInteger("tg_texmeth");
    int         preserve3D = poFeature->GetFieldAsInteger("tg_3d");
    tgTexParams tparams    = GetTPFields( tmeth, poFeature );    
    
    // now the geometry(s)
    poGeometry->transform( poCT );
            
    OGRMultiPolygon* multipoly = NULL;
    tgPolygon poly;
    
    switch( geoType ) {
        case wkbPolygon:
            poly = tgShapefile::ToPolygon(poGeometry);
            SG_LOG( SG_GENERAL, SG_INFO, "loaded single poly with " << poly.Contours() << " contours" );
            poly.SetPreserve3D(preserve3D);
            poly.SetTexMethod(tmeth);
            poly.SetMaterial(material);
            poly.SetTexParams(tparams);
            polys.push_back(poly);
            break;
            
        case wkbMultiPolygon:
            multipoly=(OGRMultiPolygon*)poGeometry;
            SG_LOG( SG_GENERAL, SG_INFO, "loaded multi poly with " << multipoly->getNumGeometries() << " polys" );
            for (int i=0;i<multipoly->getNumGeometries();i++) {
                poly = tgShapefile::ToPolygon(multipoly->getGeometryRef(i));
                SG_LOG( SG_GENERAL, SG_INFO, "   loaded poly with " << poly.Contours() << " contours" );
                poly.SetPreserve3D(preserve3D);
                poly.SetTexMethod(tmeth);
                poly.SetMaterial(material);
                poly.SetTexParams(tparams);                
                polys.push_back(poly);                
            }
            break;
            
        default:
            break;
    }
    
    return;
}

void processLayer(OGRLayer* poLayer, tgpolygon_list& polys )
{
    OGRFeatureDefn*                 poFDefn = NULL;
    OGRCoordinateTransformation*    poCT = NULL;
    char*                           srsWkt;
    OGRSpatialReference*            oSourceSRS;
    OGRSpatialReference             oTargetSRS;
    OGRFeature*                     poFeature = NULL;
    std::string                     layername;
        
    /* determine the indices of the required columns */
    poFDefn = poLayer->GetLayerDefn();
    layername = poFDefn->GetName();
    
    /* setup a transformation to WGS84 */
    oSourceSRS = poLayer->GetSpatialRef();
    if (oSourceSRS == NULL) {
        SG_LOG( SG_GENERAL, SG_ALERT, "Layer " << layername << " has no defined spatial reference system" );
        exit( 1 );
    }
    
    oSourceSRS->exportToWkt(&srsWkt);
    SG_LOG( SG_GENERAL, SG_INFO, "Source spatial reference system: " << srsWkt );
    OGRFree(srsWkt);
    
    oTargetSRS.SetWellKnownGeogCS( "WGS84" );    
    poCT = OGRCreateCoordinateTransformation(oSourceSRS, &oTargetSRS);

    SG_LOG( SG_GENERAL, SG_ALERT, "Layer " << layername << " has " << poLayer->GetFeatureCount() << " features " );
    
    // Generate the work queue for this layer
    while ( ( poFeature = poLayer->GetNextFeature()) != NULL )
    {
        ToPolygons( poFDefn, poCT, poFeature, polys );
        OGRFeature::DestroyFeature( poFeature );
    }
    
    OCTDestroyCoordinateTransformation ( poCT );
}

void tgShapefile::ToPolygons( const SGPath& p, tgpolygon_list& polys )
{
    GDALDataset* poDS = NULL;
    OGRLayer*    poLayer = NULL;
    
    if (!tgShapefile::initialized) {
        GDALAllRegister();
        tgShapefile::initialized = true;
    }
        
    poDS = (GDALDataset*)GDALOpenEx( p.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );
    if( poDS == NULL )
    {
        SG_LOG( SG_GENERAL, SG_ALERT, "Failed opening datasource " << p.c_str() );
        return;
    }
    
    SG_LOG( SG_GENERAL, SG_ALERT, "Processing datasource " << p.c_str() << " with " << poDS->GetLayerCount() << " layers " );
    polys.clear();
    
    for (int i=0; i<poDS->GetLayerCount(); i++) {
        poLayer = poDS->GetLayer(i);
            
        assert(poLayer != NULL);
        processLayer(poLayer, polys );
    }
    
    GDALClose( poDS );    
    
    for ( unsigned int i=0; i<polys.size(); i++ ) {
        SG_LOG( SG_GENERAL, SG_ALERT, "return poly " << i << " with material " << polys[i].GetMaterial() );
    }        
}

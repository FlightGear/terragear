#include <ogrsf_frmts.h>

#include <simgear/debug/logstream.hxx>

#include "tg_misc.hxx"
#include "tg_shapefile.hxx"

bool tgShapefile::initialized = false;

void* tgShapefile::OpenDatasource( const char* datasource_name )
{
    GDALDataset*  datasource;
    GDALDriver*     ogrdriver;
    GDALDriverManager* drivermanager;
    const std::string   format_name = "ESRI Shapefile";

    if (!tgShapefile::initialized) {
        GDALAllRegister();
        tgShapefile::initialized = true;
    }

    datasource = (GDALDataset *) GDALOpenEx(datasource_name,GDAL_OF_VECTOR|GDAL_OF_UPDATE, NULL ,NULL,NULL);
    if ( !datasource ) {
          drivermanager = GetGDALDriverManager();
          ogrdriver = drivermanager->GetDriverByName( format_name.c_str() );
          if ( !ogrdriver ) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Unknown datasource format driver: " << format_name );
            exit(1);
          }
          datasource = ogrdriver->Create(datasource_name,0,0,0,GDT_Unknown,NULL);
    }

    if ( !datasource ) {
        SG_LOG( SG_GENERAL, SG_ALERT, "Unable to open or create datasource: " << datasource_name );
        exit(1);
    }

    return (void*)datasource;
}

void* tgShapefile::OpenLayer( void* ds_id, const char* layer_name ) {
    GDALDataset* datasource = ( GDALDataset * )ds_id;
    OGRLayer* layer;

    OGRSpatialReference srs;
    srs.SetWellKnownGeogCS("WGS84");

    layer = datasource->GetLayerByName( layer_name );

    if ( !layer ) {
        layer = datasource->CreateLayer( layer_name, &srs, wkbPolygon25D, NULL );

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

void* tgShapefile::OpenLineLayer( void* ds_id, const char* layer_name ) {
    GDALDataset* datasource = ( GDALDataset * )ds_id;
    OGRLayer* layer;

    OGRSpatialReference srs;
    srs.SetWellKnownGeogCS("WGS84");

    layer = datasource->GetLayerByName( layer_name );

    if ( !layer ) {
        layer = datasource->CreateLayer( layer_name, &srs, wkbLineString25D, NULL );

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
    GDALDataset* datasource = ( GDALDataset * )ds_id;
    GDALClose((GDALDatasetH) datasource );

    GDALDestroyDriverManager();
    tgShapefile::initialized = false;

    return (void *)-1;
}

void tgShapefile::FromClipper( const ClipperLib::Paths& subject, const std::string& datasource, const std::string& layer, const std::string& description )
{
    void*          ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenDatasource returned " << (unsigned long)ds_id);

    OGRLayer*      l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str() );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenLayer returned " << (unsigned long)l_id);

    OGRPolygon*    polygon = new OGRPolygon();
    SG_LOG(SG_GENERAL, SG_DEBUG, "subject has " << subject.size() << " contours ");

    for ( unsigned int i = 0; i < subject.size(); ++i ) {
        ClipperLib::Path const& contour = subject[i];

        if (contour.size() < 3) {
            SG_LOG(SG_GENERAL, SG_DEBUG, "Polygon with less than 3 points");
        } else {
            OGRLinearRing *ring = new OGRLinearRing();

            // FIXME: Current we ignore the hole-flag and instead assume
            //        that the first ring is not a hole and the rest
            //        are holes
            for (unsigned int pt = 0; pt < contour.size(); pt++) {
                OGRPoint *point = new OGRPoint();
                SGGeod geod = SGGeod_FromClipper( contour[pt] );

                point->setX( geod.getLongitudeDeg() );
                point->setY( geod.getLatitudeDeg() );
                point->setZ( 0.0 );

                ring->addPoint( point );
            }

            ring->closeRings();
            polygon->addRingDirectly( ring );
        }

        OGRFeature* feature = new OGRFeature( l_id->GetLayerDefn() );

        feature->SetField("ID", description.c_str());
        feature->SetGeometry(polygon);

        if ( l_id->CreateFeature( feature ) != OGRERR_NONE )
        {
            SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
        }

        OGRFeature::DestroyFeature(feature);
    }

    // close after each write
    tgShapefile::CloseDatasource( ds_id );
}

void tgShapefile::FromContour( const tgContour& subject, const std::string& datasource, const std::string& layer, const std::string& description )
{
    void*          ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenDatasource returned " << (unsigned long)ds_id);

    OGRLayer*      l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str() );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenLayer returned " << (unsigned long)l_id);

    OGRPolygon*    polygon = new OGRPolygon();

    if (subject.GetSize() < 3) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Polygon with less than 3 points");
    } else {
        // FIXME: Current we ignore the hole-flag and instead assume
        //        that the first ring is not a hole and the rest
        //        are holes
        OGRLinearRing *ring=new OGRLinearRing();
        for (unsigned int pt = 0; pt < subject.GetSize(); pt++) {
            OGRPoint *point=new OGRPoint();

            point->setX( subject.GetNode(pt).getLongitudeDeg() );
            point->setY( subject.GetNode(pt).getLatitudeDeg() );
            point->setZ( 0.0 );
            ring->addPoint(point);
        }
        ring->closeRings();

        polygon->addRingDirectly(ring);

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

    // close after each write
    tgShapefile::CloseDatasource( ds_id );
}

void tgShapefile::FromTriangles( const tgPolygon& subject, const std::string& datasource, const std::string& layer, const std::string& description )
{
    void*          ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenDatasource returned " << (unsigned long)ds_id);

    OGRLayer*      l_id  = (OGRLayer *)tgShapefile::OpenLineLayer( ds_id, layer.c_str() );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenLayer returned " << (unsigned long)l_id);

    SG_LOG(SG_GENERAL, SG_DEBUG, "subject has " << subject.Contours() << " contours ");

    for ( unsigned int i = 0; i < subject.Triangles(); i++ ) {
        // FIXME: Current we ignore the hole-flag and instead assume
        //        that the first ring is not a hole and the rest
        //        are holes
        OGRLinearRing  ring;
        for (unsigned int pt = 0; pt < 3; pt++) {
            OGRPoint point;

            point.setX( subject.GetTriNode(i, pt).getLongitudeDeg() );
            point.setY( subject.GetTriNode(i, pt).getLatitudeDeg() );
            point.setZ( 0.0 );
            ring.addPoint(&point);
        }
        ring.closeRings();

        OGRFeature* feature = NULL;
        feature = new OGRFeature( l_id->GetLayerDefn() );
        feature->SetField("ID", description.c_str());
        feature->SetGeometry(&ring);
        if( l_id->CreateFeature( feature ) != OGRERR_NONE )
        {
            SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
        }
        OGRFeature::DestroyFeature(feature);
    }

    // close after each write
    tgShapefile::CloseDatasource( ds_id );    
}

void tgShapefile::FromPolygon( const tgPolygon& subject, const std::string& datasource, const std::string& layer, const std::string& description )
{
    void*          ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenDatasource returned " << (unsigned long)ds_id);

    OGRLayer*      l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str() );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgShapefile::OpenLayer returned " << (unsigned long)l_id);

    OGRPolygon*    polygon = new OGRPolygon();

    SG_LOG(SG_GENERAL, SG_DEBUG, "subject has " << subject.Contours() << " contours ");

    for ( unsigned int i = 0; i < subject.Contours(); i++ ) {
        bool skip_ring=false;
        tgContour contour = subject.GetContour( i );

        if (contour.GetSize() < 3) {
            SG_LOG(SG_GENERAL, SG_DEBUG, "Polygon with less than 3 points");
            skip_ring=true;
        }

        // FIXME: Current we ignore the hole-flag and instead assume
        //        that the first ring is not a hole and the rest
        //        are holes
        OGRLinearRing *ring=new OGRLinearRing();
        for (unsigned int pt = 0; pt < contour.GetSize(); pt++) {
            OGRPoint *point=new OGRPoint();

            point->setX( contour.GetNode(pt).getLongitudeDeg() );
            point->setY( contour.GetNode(pt).getLatitudeDeg() );
            point->setZ( 0.0 );
            ring->addPoint(point);
        }
        ring->closeRings();

        if (!skip_ring) {
            polygon->addRingDirectly(ring);
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

    // close after each write
    tgShapefile::CloseDatasource( ds_id );
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

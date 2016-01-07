#include "tg_polygon_set.hxx"

void tgPolygonSet::toDebugShapefile( OGRLayer* poLayer, const cgalPoly_Point& pt, const char* desc )
{
    OGRPoint point;
    
    point.setX( CGAL::to_double(pt.x() ));
    point.setY( CGAL::to_double(pt.y() ));
    point.setZ( 0.0 );
    
    OGRFeature* feature = new OGRFeature( poLayer->GetLayerDefn() );
    feature->SetGeometry(&point);
    
    if( poLayer->CreateFeature( feature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(feature);
}

void tgPolygonSet::toDebugShapefile( OGRLayer* poLayer, const cgalPoly_Polygon& poly, const char* desc )
{    
    OGRPolygon    polygon;
    OGRPoint      point;
    OGRLinearRing ring;
        
    if ( poLayer ) {
        // in CGAL, the outer boundary is counter clockwise - in GDAL, it's expected to be clockwise
        cgalPoly_Polygon::Vertex_iterator it;
        
        for ( it = poly.vertices_begin(); it != poly.vertices_end(); it++ ) {
            point.setX( CGAL::to_double( (*it).x() ) );
            point.setY( CGAL::to_double( (*it).y() ) );
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
}

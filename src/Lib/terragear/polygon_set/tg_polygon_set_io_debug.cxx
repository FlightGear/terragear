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

void tgPolygonSet::toDebugShapefile( OGRLayer* poLayer, const cgalPoly_PolygonSet& polySet, const char* desc )
{
    std::list<cgalPoly_PolygonWithHoles>                 pwh_list;
    std::list<cgalPoly_PolygonWithHoles>::const_iterator it;

    polySet.polygons_with_holes( std::back_inserter(pwh_list) );
    SG_LOG(SG_GENERAL, SG_DEBUG, "tgPolygonSet::toShapefile: got " << pwh_list.size() << " polys with holes ");
    
    // save each poly with holes to the layer
    for (it = pwh_list.begin(); it != pwh_list.end(); ++it) {
        cgalPoly_PolygonWithHoles pwh = (*it);

        toDebugShapefile( poLayer, pwh, desc );
    }
}
    
void tgPolygonSet::toDebugShapefile( OGRLayer* poLayer, const cgalPoly_PolygonWithHoles& pwh, const char* desc )
{
    OGRPolygon    polygon;
    OGRPoint      point;
    OGRLinearRing ring;
    
    // in CGAL, the outer boundary is counter clockwise - in GDAL, it's expected to be clockwise
    cgalPoly_Polygon poly;
    cgalPoly_Polygon::Vertex_iterator it;

    poly = pwh.outer_boundary();
    for ( it = poly.vertices_begin(); it != poly.vertices_end(); it++ ) {
        point.setX( CGAL::to_double( (*it).x() ) );
        point.setY( CGAL::to_double( (*it).y() ) );
        point.setZ( 0.0 );
                
        ring.addPoint(&point);
    }
    ring.closeRings();
    polygon.addRing(&ring);

    // then write each hole
    cgalPoly_PolygonWithHoles::Hole_const_iterator hit;
    for (hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
        OGRLinearRing hole;
        poly = (*hit);

        for ( it = poly.vertices_begin(); it != poly.vertices_end(); it++ ) {
            point.setX( CGAL::to_double( (*it).x() ) );
            point.setY( CGAL::to_double( (*it).y() ) );
            point.setZ( 0.0 );
                    
            hole.addPoint(&point);            
        }
        hole.closeRings();
        polygon.addRing(&hole);
    }

    OGRFeature* poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
    poFeature->SetGeometry(&polygon);
    
    if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(poFeature);
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

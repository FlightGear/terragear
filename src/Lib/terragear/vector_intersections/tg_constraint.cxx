#include <ogrsf_frmts.h> 
#include <terragear/tg_shapefile.hxx>

#include "tg_constraint.hxx"

tgConstraint::tgConstraint( consType_e t, const edgeArrPoint& s, const edgeArrPoint& e, unsigned long p, const std::string& desc ) 
{
    edgeArrLinearCurve c;
    
    type  = t; 
    start = s;
    end   = e;
    
    switch( type ) {
        case consLine:
            c = edgeArrLine(s, e);
            break;
            
        case consRay:
            c = edgeArrRay(s, e);
            break;
            
        case consSegment:
            c = edgeArrSegment(s, e);
            break;
    }
    
    // Get heading from Geodesy
    SGGeod gSource = SGGeod::fromDeg( CGAL::to_double( s.x() ), CGAL::to_double( s.y() ) );
    SGGeod gTarget = SGGeod::fromDeg( CGAL::to_double( e.x() ), CGAL::to_double( e.y() ) );
    heading = SGGeodesy::courseDeg( gSource, gTarget );
    
    ConstraintData d = ConstraintData( p, desc );
    curve = edgeArrCurve( c, d );
}

bool tgConstraint::hasOn( edgeArrPoint& pt ) {
    bool has_on = false;
    
    switch( type ) {
        case consLine: 
            has_on = curve.line().has_on( pt );
            break;
            
        case consRay:
            has_on = curve.ray().has_on( pt );
            break;
            
        case consSegment:
            has_on = curve.ray().has_on( pt );
            break;                
    }
    
    return has_on;
}

void tgConstraint::breakAt( edgeArrPoint& pt ) 
{
    edgeArrLinearCurve c;
    ConstraintData     d( getPeer(), getDescription() );  
    
    // lines become rays
    // rays become segments
    // segments become shorter segments...
    switch( type ) {
        case consLine:
            // which direction should the ray go?  TBD
            c = edgeArrRay(pt, end);
            break;
            
        case consRay:
            c = edgeArrSegment(start, pt);
            break;
            
        case consSegment:
            c = edgeArrSegment(start, pt);
            break;                
    }
    
    
    // create new curve
    curve = edgeArrCurve( c, d );
}

bool tgConstraint::breakWith( const tgConstraint& other ) 
{
    bool replace = false;
    
    // check if the other constraint intersects with a portion of this constraint
    // there doesn't seem to be a base class other than Object, which isn't what we want
    CGAL::Object result;
    
    switch( type ) {
        case consLine:
            switch( other.getType() ) {
                case consLine:
                    result = CGAL::intersection( curve.line(), other.line() );
                    break;
                case consRay:
                    result = CGAL::intersection( curve.line(), other.ray() );
                    break;
                case consSegment:
                    result = CGAL::intersection( curve.line(), other.segment() );
                    break;
            }
            break;
            
        case consRay:
            switch( other.getType() ) {
                case consLine:
                    result = CGAL::intersection( curve.ray(), other.line() );
                    break;
                case consRay:
                    result = CGAL::intersection( curve.ray(), other.ray() );
                    break;
                case consSegment:
                    result = CGAL::intersection( curve.ray(), other.segment() );
                    break;
            }
            break;                        
                    
        case consSegment:
            switch( other.getType() ) {
                case consLine:
                    result = CGAL::intersection( curve.segment(), other.line() );
                    break;
                case consRay:
                    result = CGAL::intersection( curve.segment(), other.ray() );
                    break;
                case consSegment:
                    result = CGAL::intersection( curve.segment(), other.segment() );
                    break;
            }
            break;
    }
    
    // lot's of cases - let's handle them one at a time and see what shales out...
    //
    // case 1: we add a segment constraint, and it overlaps with a new segment constraint
    // replace with segment
    if (const edgeArrSegment *iseg = CGAL::object_cast<edgeArrSegment>(&result)) {
        if ( ( type == consRay ) && ( iseg->source() == getStart() ) ) {
            // replace this constraint with the segment
            replace = true;
        }
    }
    
    return replace;
}

bool tgConstraint::Intersect( const tgConstraint& other, edgeArrPoint& intersectionLocation ) {
    bool intersects = false;
    
    // there doesn't seem to be a base class other than Object, which isn't what we want
    CGAL::Object result;
    
    switch( type ) {
        case consLine:
            switch( other.getType() ) {
                case consLine:
                    result = CGAL::intersection( curve.line(), other.line() );
                    break;
                case consRay:
                    result = CGAL::intersection( curve.line(), other.ray() );
                    break;
                case consSegment:
                    result = CGAL::intersection( curve.line(), other.segment() );
                    break;
            }
            break;
            
        case consRay:
            switch( other.getType() ) {
                case consLine:
                    result = CGAL::intersection( curve.ray(), other.line() );
                    break;
                case consRay:
                    result = CGAL::intersection( curve.ray(), other.ray() );
                    break;
                case consSegment:
                    result = CGAL::intersection( curve.ray(), other.segment() );
                    break;
            }
            break;                        
            
        case consSegment:
            switch( other.getType() ) {
                case consLine:
                    result = CGAL::intersection( curve.segment(), other.line() );
                    break;
                case consRay:
                    result = CGAL::intersection( curve.segment(), other.ray() );
                    break;
                case consSegment:
                    result = CGAL::intersection( curve.segment(), other.segment() );
                    break;
            }
            break;
    }
    
    if (const edgeArrPoint *ipoint = CGAL::object_cast<edgeArrPoint>(&result)) {
        // handle the point intersection case with *ipoint.
        intersectionLocation = *ipoint;
        intersects = true;
    }
    
    return intersects;
}

void tgConstraint::toShapefile(void* lid) const
{
    OGRLayer* l_id = (OGRLayer *)lid;
    
    OGRLineString oLine;
    OGRPoint      oStart, oEnd;
    
    SGGeod        gStart  = SGGeod::fromDeg( CGAL::to_double( start.x() ), CGAL::to_double( start.y() ) );
    SGGeod        gEnd    = SGGeod::fromDeg( CGAL::to_double( end.x() ),   CGAL::to_double( end.y() ) );    
    
    oStart.setX( CGAL::to_double( start.x() ) );
    oStart.setY( CGAL::to_double( start.y() ) );
    oStart.setZ( 0.0 );    
    oLine.addPoint(&oStart);

    if ( type == tgConstraint::consLine ) {
        SGGeod slArrow = SGGeodesy::direct( gStart, SGMiscd::normalizePeriodic(0, 360, heading-10), 0.2 );
        SGGeod srArrow = SGGeodesy::direct( gStart, SGMiscd::normalizePeriodic(0, 360, heading+10), 0.2 );
        
        OGRPoint oLStart;
        oLStart.setX( slArrow.getLongitudeDeg() );
        oLStart.setY( slArrow.getLatitudeDeg() );
        oLStart.setZ( 0.0 );
        oLine.addPoint(&oLStart);
        
        OGRPoint oRStart;
        oRStart.setX( srArrow.getLongitudeDeg() );
        oRStart.setY( srArrow.getLatitudeDeg() );
        oRStart.setZ( 0.0 );
        oLine.addPoint(&oRStart);
        
        // start back at the beginning
        oLine.addPoint(&oStart);
    }

    oEnd.setX( CGAL::to_double( end.x() ) );
    oEnd.setY( CGAL::to_double( end.y() ) );
    oEnd.setZ( 0.0 );
    oLine.addPoint(&oEnd);

    if ( type == tgConstraint::consLine || type == tgConstraint::consRay ) {
        SGGeod elArrow = SGGeodesy::direct( gEnd, SGMiscd::normalizePeriodic(0, 360, heading+170), 0.2 );
        SGGeod erArrow = SGGeodesy::direct( gEnd, SGMiscd::normalizePeriodic(0, 360, heading+190), 0.2 );
        
        OGRPoint oLEnd;
        oLEnd.setX( elArrow.getLongitudeDeg() );
        oLEnd.setY( elArrow.getLatitudeDeg() );
        oLEnd.setZ( 0.0 );
        oLine.addPoint(&oLEnd);
        
        OGRPoint oREnd;
        oREnd.setX( erArrow.getLongitudeDeg() );
        oREnd.setY( erArrow.getLatitudeDeg() );
        oREnd.setZ( 0.0 );
        oLine.addPoint(&oREnd);
        
        // finish the arrow
        oLine.addPoint(&oEnd);
    }

    OGRFeature* feature = NULL;
    feature = OGRFeature::CreateFeature( l_id->GetLayerDefn() );    
    feature->SetField( "tg_desc", getDescription().c_str() );
    feature->SetGeometry( &oLine ); 
    
    if( l_id->CreateFeature( feature ) != OGRERR_NONE )
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Failed to create feature in shapefile");
    }
    OGRFeature::DestroyFeature(feature);    
}

void tgConstraint::toShapefile(const std::string& datasource, const std::string& layer) const
{
    void* ds_id = tgShapefile::OpenDatasource( datasource.c_str() );
    if ( ds_id ) {
        OGRLayer* l_id  = (OGRLayer *)tgShapefile::OpenLayer( ds_id, layer.c_str(), tgShapefile::LT_LINE );
        toShapefile( (void *)l_id );
    
        // close after each write
        ds_id = tgShapefile::CloseDatasource( ds_id );
    } else {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgShapefile::FromConstraint open datasource failed. datasource: " << datasource << " layer: " << layer );
    }    
}
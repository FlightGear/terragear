#include <simgear/debug/logstream.hxx>
#include <simgear/sg_inlines.h>

#include "tg_cgal.hxx"

tgRectangle tgSegment::GetBoundingBox( void ) const
{
    SGGeod min, max;

    double minx =  SG_MIN2( CGAL::to_double( start.x() ), CGAL::to_double( end.x() ));
    double miny =  SG_MIN2( CGAL::to_double( start.y() ), CGAL::to_double( end.y() ));
    double maxx =  SG_MAX2( CGAL::to_double( start.x() ), CGAL::to_double( end.x() ));
    double maxy =  SG_MAX2( CGAL::to_double( start.y() ), CGAL::to_double( end.y() ));

    min = SGGeod::fromDeg( minx, miny );
    max = SGGeod::fromDeg( maxx, maxy );

    return tgRectangle( min, max );
}

bool tgSegment::Intersect( const tgSegment& seg, SGGeod& intersection) const
{
    EPECSegment_2 s1 = toCgal();
    EPECSegment_2 s2 = seg.toCgal();
    bool          retval = false;
        
    CGAL::Object result = CGAL::intersection(s1, s2);
    if (const EPECPoint_2 *ipoint = CGAL::object_cast<EPECPoint_2>(&result)) {
        // handle the point intersection case with *ipoint.
        intersection = SGGeod::fromDeg( CGAL::to_double(ipoint->x()), CGAL::to_double(ipoint->y()) );
        retval = true;
    }
    
    return retval;
}
    
bool tgSegment::Intersect( const tgRay& ray, SGGeod& intersection) const
{
    EPECSegment_2 s1 = toCgal();
    EPECRay_2     r2 = ray.toCgal();
    bool          retval = false;
    
    CGAL::Object result = CGAL::intersection(s1, r2);
    if (const EPECPoint_2 *ipoint = CGAL::object_cast<EPECPoint_2>(&result)) {
        // handle the point intersection case with *ipoint.
        intersection = SGGeod::fromDeg( CGAL::to_double(ipoint->x()), CGAL::to_double(ipoint->y()) );
        retval = true;
    }
    
    return retval;
}
    
bool tgSegment::Intersect( const tgLine& line, SGGeod& intersection) const
{
    EPECSegment_2 s1 = toCgal();
    EPECLine_2    l2 = line.toCgal();
    bool          retval = false;
    
    CGAL::Object result = CGAL::intersection(s1, l2);
    if (const EPECPoint_2 *ipoint = CGAL::object_cast<EPECPoint_2>(&result)) {
        // handle the point intersection case with *ipoint.
        intersection = SGGeod::fromDeg( CGAL::to_double(ipoint->x()), CGAL::to_double(ipoint->y()) );
        retval = true;
    }
    
    return retval;
}

bool tgSegment::Intersect( const tgSegment& seg, SGGeod* iPos, tgSegment* iSeg ) const
{
    EPECSegment_2 s1 = toCgal();
    EPECSegment_2 s2 = seg.toCgal();
    bool          retval = false;
    
    CGAL::Object result = CGAL::intersection(s1, s2);
    if (const EPECPoint_2 *ipoint = CGAL::object_cast<EPECPoint_2>(&result)) {
        // handle the point intersection case with *iPos.
        SG_LOG(SG_GENERAL, SG_INFO, "tgSegment::Intersect: result is point" );
        iPos = new SGGeod();
        iPos->setLongitudeDeg( CGAL::to_double(ipoint->x()) );
        iPos->setLatitudeDeg( CGAL::to_double(ipoint->y()) );
        retval = true;
    } else if (const EPECSegment_2 *iseg = CGAL::object_cast<EPECSegment_2>(&result)) {
        // handle the point intersection case with *iSeg.
        SG_LOG(SG_GENERAL, SG_INFO, "tgSegment::Intersect: result is segment" );
        iSeg = new tgSegment( iseg->source(), iseg->target() );
        retval = true;
    }
    
    return retval;
    
}

bool tgSegment::IsOnLine( const SGGeod& pos ) const
{
    bool on_line = false;
    double m, m1, b, b1, y_err, x_err, y_err_min, x_err_min;

    const double bbEpsilon  = SG_EPSILON*10;
    const double errEpsilon = SG_EPSILON*4;

    SGGeod p0 = EPECPointToGeod(start);
    SGGeod p1 = EPECPointToGeod(end);

    double xdist = fabs(p0.getLongitudeDeg() - p1.getLongitudeDeg());
    double ydist = fabs(p0.getLatitudeDeg()  - p1.getLatitudeDeg());

    x_err_min = xdist + 1.0;
    y_err_min = ydist + 1.0;

    if ( xdist > ydist ) {
        // sort these in a sensible order
        SGGeod p_min, p_max;
        if ( p0.getLongitudeDeg() < p1.getLongitudeDeg() ) {
            p_min = p0;
            p_max = p1;
        } else {
            p_min = p1;
            p_max = p0;
        }

        m = (p_min.getLatitudeDeg() - p_max.getLatitudeDeg()) / (p_min.getLongitudeDeg() - p_max.getLongitudeDeg());
        b = p_max.getLatitudeDeg() - m * p_max.getLongitudeDeg();

        if ( (pos.getLongitudeDeg() > (p_min.getLongitudeDeg() + (bbEpsilon))) && (pos.getLongitudeDeg() < (p_max.getLongitudeDeg() - (bbEpsilon))) ) {
            y_err = fabs(pos.getLatitudeDeg() - (m * pos.getLongitudeDeg() + b));

            if ( y_err < errEpsilon ) {
                on_line = true;
                if ( y_err < y_err_min ) {
                    y_err_min = y_err;
                }
            }
        }
    } else {
        // sort these in a sensible order
        SGGeod p_min, p_max;
        if ( p0.getLatitudeDeg() < p1.getLatitudeDeg() ) {
            p_min = p0;
            p_max = p1;
        } else {
            p_min = p1;
            p_max = p0;
        }

        m1 = (p_min.getLongitudeDeg() - p_max.getLongitudeDeg()) / (p_min.getLatitudeDeg() - p_max.getLatitudeDeg());
        b1 = p_max.getLongitudeDeg() - m1 * p_max.getLatitudeDeg();

        if ( (pos.getLatitudeDeg() > (p_min.getLatitudeDeg() + (bbEpsilon))) && (pos.getLatitudeDeg() < (p_max.getLatitudeDeg() - (bbEpsilon))) ) {

            x_err = fabs(pos.getLongitudeDeg() - (m1 * pos.getLatitudeDeg() + b1));

            if ( x_err < errEpsilon ) {
                on_line = true;
                if ( x_err < x_err_min ) {
                    x_err_min = x_err;
                }
            }
        }
    }

    return on_line;
}


bool tgRay::Intersect( const tgSegment& seg, SGGeod& intersection) const
{
    EPECRay_2     r1 = toCgal();
    EPECSegment_2 s2 = seg.toCgal();
    bool          retval = false;
    
    CGAL::Object result = CGAL::intersection(r1, s2);
    if (const EPECPoint_2 *ipoint = CGAL::object_cast<EPECPoint_2>(&result)) {
        // handle the point intersection case with *ipoint.
        intersection = SGGeod::fromDeg( CGAL::to_double(ipoint->x()), CGAL::to_double(ipoint->y()) );
        retval = true;
    }
    
    return retval;
}
    
bool tgRay::Intersect( const tgRay& ray, SGGeod& intersection) const
{
    EPECRay_2     r1 = toCgal();
    EPECRay_2     r2 = ray.toCgal();
    bool          retval = false;
    
    CGAL::Object result = CGAL::intersection(r1, r2);
    if (const EPECPoint_2 *ipoint = CGAL::object_cast<EPECPoint_2>(&result)) {
        // handle the point intersection case with *ipoint.
        intersection = SGGeod::fromDeg( CGAL::to_double(ipoint->x()), CGAL::to_double(ipoint->y()) );
        retval = true;
    }
    
    return retval;
}
    
bool tgRay::Intersect( const tgLine& line, SGGeod& intersection) const
{
    EPECRay_2     r1 = toCgal();
    EPECLine_2    l2 = line.toCgal();
    bool          retval = false;
    
    CGAL::Object result = CGAL::intersection(r1, l2);
    if (const EPECPoint_2 *ipoint = CGAL::object_cast<EPECPoint_2>(&result)) {
        // handle the point intersection case with *ipoint.
        intersection = SGGeod::fromDeg( CGAL::to_double(ipoint->x()), CGAL::to_double(ipoint->y()) );
        retval = true;
    }
    
    return retval;
}


bool tgLine::Intersect( const tgSegment& seg, SGGeod& intersection) const
{
    EPECLine_2    l1 = toCgal();
    EPECSegment_2 s2 = seg.toCgal();
    bool          retval = false;
    
    CGAL::Object result = CGAL::intersection(l1, s2);
    if (const EPECPoint_2 *ipoint = CGAL::object_cast<EPECPoint_2>(&result)) {
        // handle the point intersection case with *ipoint.
        intersection = SGGeod::fromDeg( CGAL::to_double(ipoint->x()), CGAL::to_double(ipoint->y()) );
        retval = true;
    } else if (const EPECSegment_2 *iseg = CGAL::object_cast<EPECSegment_2>(&result)) {
        SG_LOG(SG_GENERAL, SG_INFO, "tgLine::Intersect: result is segment " << iseg );
        retval = true;
    } else if (const EPECRay_2 *iray = CGAL::object_cast<EPECRay_2>(&result)) {
        SG_LOG(SG_GENERAL, SG_INFO, "tgLine::Intersect: result is ray " << iray );
        retval = true;
    } 
    
    return retval;
}
    
bool tgLine::Intersect( const tgRay& ray, SGGeod& intersection) const
{
    EPECLine_2    l1 = toCgal();
    EPECRay_2     r2 = ray.toCgal();
    bool          retval = false;
    
    CGAL::Object result = CGAL::intersection(l1, r2);
    if (const EPECPoint_2 *ipoint = CGAL::object_cast<EPECPoint_2>(&result)) {
        // handle the point intersection case with *ipoint.
        intersection = SGGeod::fromDeg( CGAL::to_double(ipoint->x()), CGAL::to_double(ipoint->y()) );
        retval = true;
    }
    
    return retval;
}
    
bool tgLine::Intersect( const tgLine& line, SGGeod& intersection) const
{
    EPECLine_2    l1 = toCgal();
    EPECLine_2    l2 = line.toCgal();
    bool          retval = false;
    
    CGAL::Object result = CGAL::intersection(l1, l2);
    if (const EPECPoint_2 *ipoint = CGAL::object_cast<EPECPoint_2>(&result)) {
        // handle the point intersection case with *ipoint.
        intersection = SGGeod::fromDeg( CGAL::to_double(ipoint->x()), CGAL::to_double(ipoint->y()) );
        retval = true;
    }
    
    return retval;
}

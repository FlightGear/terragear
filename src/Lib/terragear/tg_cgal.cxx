#include <simgear/debug/logstream.hxx>

#include "tg_cgal.hxx"

bool tgSegment::Intersect( const tgSegment& seg, SGGeod& intersection) 
{
    CBSegment_2 s1 = toCgal();
    CBSegment_2 s2 = seg.toCgal();
    bool        retval = false;
        
    CGAL::Object result = CGAL::intersection(s1, s2);
    if (const CBPoint_2 *ipoint = CGAL::object_cast<CBPoint_2>(&result)) {
        // handle the point intersection case with *ipoint.
        intersection = SGGeod::fromDeg( CGAL::to_double(ipoint->x()), CGAL::to_double(ipoint->y()) );
        retval = true;
    }
    
    return retval;
}
    
bool tgSegment::Intersect( const tgRay& ray, SGGeod& intersection) 
{
    CBSegment_2 s1 = toCgal();
    CBRay_2     r2 = ray.toCgal();
    bool        retval = false;
    
    CGAL::Object result = CGAL::intersection(s1, r2);
    if (const CBPoint_2 *ipoint = CGAL::object_cast<CBPoint_2>(&result)) {
        // handle the point intersection case with *ipoint.
        intersection = SGGeod::fromDeg( CGAL::to_double(ipoint->x()), CGAL::to_double(ipoint->y()) );
        retval = true;
    }
    
    return retval;
}
    
bool tgSegment::Intersect( const tgLine& line, SGGeod& intersection) 
{
    CBSegment_2 s1 = toCgal();
    CBLine_2    l2 = line.toCgal();
    bool        retval = false;
    
    CGAL::Object result = CGAL::intersection(s1, l2);
    if (const CBPoint_2 *ipoint = CGAL::object_cast<CBPoint_2>(&result)) {
        // handle the point intersection case with *ipoint.
        intersection = SGGeod::fromDeg( CGAL::to_double(ipoint->x()), CGAL::to_double(ipoint->y()) );
        retval = true;
    }
    
    return retval;
}



bool tgRay::Intersect( const tgSegment& seg, SGGeod& intersection)
{
    CBRay_2     r1 = toCgal();
    CBSegment_2 s2 = seg.toCgal();
    bool        retval = false;
    
    CGAL::Object result = CGAL::intersection(r1, s2);
    if (const CBPoint_2 *ipoint = CGAL::object_cast<CBPoint_2>(&result)) {
        // handle the point intersection case with *ipoint.
        intersection = SGGeod::fromDeg( CGAL::to_double(ipoint->x()), CGAL::to_double(ipoint->y()) );
        retval = true;
    }
    
    return retval;
}
    
bool tgRay::Intersect( const tgRay& ray, SGGeod& intersection)
{
    CBRay_2     r1 = toCgal();
    CBRay_2     r2 = ray.toCgal();
    bool        retval = false;
    
    CGAL::Object result = CGAL::intersection(r1, r2);
    if (const CBPoint_2 *ipoint = CGAL::object_cast<CBPoint_2>(&result)) {
        // handle the point intersection case with *ipoint.
        intersection = SGGeod::fromDeg( CGAL::to_double(ipoint->x()), CGAL::to_double(ipoint->y()) );
        retval = true;
    }
    
    return retval;
}
    
bool tgRay::Intersect( const tgLine& line, SGGeod& intersection)
{
    CBRay_2     r1 = toCgal();
    CBLine_2    l2 = line.toCgal();
    bool        retval = false;
    
    CGAL::Object result = CGAL::intersection(r1, l2);
    if (const CBPoint_2 *ipoint = CGAL::object_cast<CBPoint_2>(&result)) {
        // handle the point intersection case with *ipoint.
        intersection = SGGeod::fromDeg( CGAL::to_double(ipoint->x()), CGAL::to_double(ipoint->y()) );
        retval = true;
    }
    
    return retval;
}


bool tgLine::Intersect( const tgSegment& seg, SGGeod& intersection)
{
    CBLine_2    l1 = toCgal();
    CBSegment_2 s2 = seg.toCgal();
    bool        retval = false;
    
    CGAL::Object result = CGAL::intersection(l1, s2);
    if (const CBPoint_2 *ipoint = CGAL::object_cast<CBPoint_2>(&result)) {
        // handle the point intersection case with *ipoint.
        intersection = SGGeod::fromDeg( CGAL::to_double(ipoint->x()), CGAL::to_double(ipoint->y()) );
        retval = true;
    } else if (const CBSegment_2 *iseg = CGAL::object_cast<CBSegment_2>(&result)) {
        SG_LOG(SG_GENERAL, SG_INFO, "tgLine::Intersect: result is segment" );
        retval = true;
    } else if (const CBRay_2 *iray = CGAL::object_cast<CBRay_2>(&result)) {
        SG_LOG(SG_GENERAL, SG_INFO, "tgLine::Intersect: result is ray" );
        retval = true;
    } 
    
    return retval;
}
    
bool tgLine::Intersect( const tgRay& ray, SGGeod& intersection)
{
    CBLine_2    l1 = toCgal();
    CBRay_2     r2 = ray.toCgal();
    bool        retval = false;
    
    CGAL::Object result = CGAL::intersection(l1, r2);
    if (const CBPoint_2 *ipoint = CGAL::object_cast<CBPoint_2>(&result)) {
        // handle the point intersection case with *ipoint.
        intersection = SGGeod::fromDeg( CGAL::to_double(ipoint->x()), CGAL::to_double(ipoint->y()) );
        retval = true;
    }
    
    return retval;
}
    
bool tgLine::Intersect( const tgLine& line, SGGeod& intersection)
{
    CBLine_2    l1 = toCgal();
    CBLine_2    l2 = line.toCgal();
    bool        retval = false;
    
    CGAL::Object result = CGAL::intersection(l1, l2);
    if (const CBPoint_2 *ipoint = CGAL::object_cast<CBPoint_2>(&result)) {
        // handle the point intersection case with *ipoint.
        intersection = SGGeod::fromDeg( CGAL::to_double(ipoint->x()), CGAL::to_double(ipoint->y()) );
        retval = true;
    }
    
    return retval;
}

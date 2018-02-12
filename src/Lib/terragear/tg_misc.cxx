#include <limits.h>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/intersections.h>

#include <simgear/debug/logstream.hxx>

#include "tg_polygon.hxx"
#include "tg_misc.hxx"

const double isEqual2D_Epsilon = 0.000000001;

#define CLIPPER_FIXEDPT           (1000000000000)
#define CLIPPER_METERS_PER_DEGREE (111000)

SGGeod SGGeod_snap( const SGGeod& in, double grid )
{
    return SGGeod::fromDegM( grid * SGMisc<double>::round( in.getLongitudeDeg()/grid ),
                             grid * SGMisc<double>::round( in.getLatitudeDeg() /grid ),
                             grid * SGMisc<double>::round( in.getElevationM()  /grid ) );
}

bool SGGeod_isEqual2D( const SGGeod& g0, const SGGeod& g1 )
{
    return ( (fabs( g0.getLongitudeDeg() - g1.getLongitudeDeg() ) < isEqual2D_Epsilon) &&
             (fabs( g0.getLatitudeDeg()  - g1.getLatitudeDeg() )  < isEqual2D_Epsilon ) );
}

SGVec2d SGGeod_ToSGVec2d( const SGGeod& p )
{
    return SGVec2d( p.getLongitudeDeg(), p.getLatitudeDeg() );
}

// Calculate theta of angle (a, b, c)
double SGGeod_CalculateTheta( const SGGeod& p0, const SGGeod& p1, const SGGeod& p2 )
{
    SGVec2d v0 = SGGeod_ToSGVec2d( p0 );
    SGVec2d v1 = SGGeod_ToSGVec2d( p1 );
    SGVec2d v2 = SGGeod_ToSGVec2d( p2 );

    return SGVec2d_CalculateTheta( v0, v1, v2 );
}

// Calculate theta of angle of v0, v1, and v2 - vectors represent cartestian positions
double SGVec2d_CalculateTheta( const SGVec2d& v0, const SGVec2d& v1, const SGVec2d& v2 )
{
    SGVec2d u, v;
    double udist, vdist, uv_dot;

    // u . v = ||u|| * ||v|| * cos(theta)

    u = v1 - v0;
    udist = dist(v1, v2);

    v = v1 - v2;
    vdist = dist(v1, v2);

    uv_dot = dot(u, v);

    return acos(uv_dot / (udist * vdist) );
}

// calculate theta from two directions
double CalculateTheta( const SGVec3d& dirCur, const SGVec3d& dirNext )
{
    double dp = dot( dirCur, dirNext );
    
    return acos( dp );
}

ClipperLib::IntPoint SGGeod_ToClipper( const SGGeod& p )
{
    ClipperLib::cInt x, y;

    if ( p.getLongitudeDeg() > 0 ) {
        x = (ClipperLib::cInt)( (p.getLongitudeDeg() * CLIPPER_FIXEDPT) + 0.5  );
    } else {
        x = (ClipperLib::cInt)( (p.getLongitudeDeg() * CLIPPER_FIXEDPT) - 0.5  );
    }
    
    if ( p.getLatitudeDeg() > 0 ) {
        y = (ClipperLib::cInt)( (p.getLatitudeDeg()  * CLIPPER_FIXEDPT) + 0.5  );
    } else {
        y = (ClipperLib::cInt)( (p.getLatitudeDeg()  * CLIPPER_FIXEDPT) - 0.5  );
    }
    
    return ClipperLib::IntPoint( x, y );
}

SGGeod SGGeod_FromClipper( const ClipperLib::IntPoint& p )
{
    double lon, lat;

    lon = (double)( ((double)p.X) / (double)CLIPPER_FIXEDPT );
    lat = (double)( ((double)p.Y) / (double)CLIPPER_FIXEDPT );

    return SGGeod::fromDeg( lon, lat );
}

double Dist_ToClipper( double dist )
{
    return ( (dist / CLIPPER_METERS_PER_DEGREE) * CLIPPER_FIXEDPT );
}

#ifdef _MSC_VER
#   define LONG_LONG_MAX LLONG_MAX
#   define LONG_LONG_MIN LLONG_MIN
#endif

tgRectangle BoundingBox_FromClipper( const ClipperLib::Paths& subject )
{
    ClipperLib::IntPoint min_pt, max_pt;
    SGGeod min, max;

    min_pt.X = min_pt.Y = LONG_LONG_MAX;
    max_pt.X = max_pt.Y = LONG_LONG_MIN;

    // for each polygon, we need to check the orientation, to set the hole flag...
    for (unsigned int i=0; i<subject.size(); i++)
    {
        for (unsigned int j = 0; j < subject[i].size(); j++)
        {
            if ( subject[i][j].X < min_pt.X ) {
                min_pt.X = subject[i][j].X;
            }
            if ( subject[i][j].Y < min_pt.Y ) {
                min_pt.Y = subject[i][j].Y;
            }

            if ( subject[i][j].X > max_pt.X ) {
                max_pt.X = subject[i][j].X;
            }
            if ( subject[i][j].Y > max_pt.Y ) {
                max_pt.Y = subject[i][j].Y;
            }
        }
    }

    min = SGGeod_FromClipper( min_pt );
    max = SGGeod_FromClipper( max_pt );

    return tgRectangle( min, max );
}

SGGeod OffsetPointMiddle( const SGGeod& gPrev, const SGGeod& gCur, const SGGeod& gNext, double offset_by, int& turn_dir )
{
    double  courseCur, courseNext, courseAvg, theta;
    SGVec3d dirCur, dirNext, dirAvg, cp;
    double  courseOffset, distOffset;
    SGGeod  pt;

    SG_LOG(SG_GENERAL, SG_DEBUG, "Find average angle for contour: prev (" << gPrev << "), "
                                                                  "cur (" << gCur  << "), "
                                                                 "next (" << gNext << ")" );

    // first, find if the line turns left or right ar src
    // for this, take the cross product of the vectors from prev to src, and src to next.
    // if the cross product is negetive, we've turned to the left
    // if the cross product is positive, we've turned to the right
    courseCur = SGGeodesy::courseDeg( gCur, gPrev );
    dirCur = SGVec3d( sin( courseCur*SGD_DEGREES_TO_RADIANS ), cos( courseCur*SGD_DEGREES_TO_RADIANS ), 0.0f );

    courseNext = SGGeodesy::courseDeg( gCur, gNext );
    dirNext = SGVec3d( sin( courseNext*SGD_DEGREES_TO_RADIANS ), cos( courseNext*SGD_DEGREES_TO_RADIANS ), 0.0f );

    // Now find the average
    dirAvg = normalize( dirCur + dirNext );
    courseAvg = SGMiscd::rad2deg( atan( dirAvg.x()/dirAvg.y() ) );
    if (courseAvg < 0) {
        courseAvg += 180.0f;
    }

    // check the turn direction
    cp    = cross( dirCur, dirNext );
    theta = SGMiscd::rad2deg(CalculateTheta( dirCur, dirNext ) );

    if ( (abs(theta - 180.0) < 0.1) || (abs(theta) < 0.1) || (isnan(theta)) ) {
        // straight line blows up math - offset 90 degree and dist is as given
        courseOffset = SGMiscd::normalizePeriodic(0, 360, courseNext-90.0);
        distOffset   = offset_by;
    }  else  {
        // calculate correct distance for the offset point
        if (cp.z() < 0.0f) {
            courseOffset = SGMiscd::normalizePeriodic(0, 360, courseAvg+180);
            turn_dir = 0;
        } else {
            courseOffset = SGMiscd::normalizePeriodic(0, 360, courseAvg);
            turn_dir = 1;
        }
        distOffset = (offset_by)/sin(SGMiscd::deg2rad(courseNext-courseOffset));
    }

    // calculate the point from cur
    pt = SGGeodesy::direct(gCur, courseOffset, distOffset);
    SG_LOG(SG_GENERAL, SG_DEBUG, "\theading is " << courseOffset << " distance is " << distOffset << " point is (" << pt.getLatitudeDeg() << "," << pt.getLongitudeDeg() << ")" );

    return pt;
}

SGGeod OffsetPointMiddle( const SGGeod& gPrev, const SGGeod& gCur, const SGGeod& gNext, double offset_by )
{
    int unused;
    return OffsetPointMiddle( gPrev, gCur, gNext, offset_by, unused );
}


SGGeod OffsetPointFirst( const SGGeod& cur, const SGGeod& next, double offset_by )
{
    double courseOffset;
    SGGeod pt;

    SG_LOG(SG_GENERAL, SG_DEBUG, "Find OffsetPoint at Start : cur (" << cur  << "), "
                                                            "next (" << next << ")" );

    // find the offset angle
    courseOffset = SGGeodesy::courseDeg( cur, next ) - 90;
    courseOffset = SGMiscd::normalizePeriodic(0, 360, courseOffset);

    // calculate the point from cur
    pt = SGGeodesy::direct( cur, courseOffset, offset_by );
    SG_LOG(SG_GENERAL, SG_DEBUG, "\theading is " << courseOffset << " distance is " << offset_by << " point is (" << pt.getLatitudeDeg() << "," << pt.getLongitudeDeg() << ")" );

    return pt;
}

SGGeod OffsetPointLast( const SGGeod& prev, const SGGeod& cur, double offset_by )
{
    double courseOffset;
    SGGeod pt;

    SG_LOG(SG_GENERAL, SG_DEBUG, "Find OffsetPoint at End   : prev (" << prev  << "), "
                                                              "cur (" << cur << ")" );

    // find the offset angle
    courseOffset = SGGeodesy::courseDeg( prev, cur ) - 90;
    courseOffset = SGMiscd::normalizePeriodic(0, 360, courseOffset);

    // calculate the point from cur
    pt = SGGeodesy::direct( cur, courseOffset, offset_by );
    SG_LOG(SG_GENERAL, SG_DEBUG, "\theading is " << courseOffset << " distance is " << offset_by << " point is (" << pt.getLatitudeDeg() << "," << pt.getLongitudeDeg() << ")" );

    return pt;
}


void OffsetPointsMiddle( const SGGeod& gPrev, const SGGeod& gCur, const SGGeod& gNext, double offset_by, double width, int& turn_dir, SGGeod& inner, SGGeod& outer )
{
    double  courseCur, courseNext, courseAvg, theta;
    SGVec3d dirCur, dirNext, dirAvg, cp;
    double  courseOffset, distOffsetInner, distOffsetOuter;
    SGGeod  pt;

    SG_LOG(SG_GENERAL, SG_DEBUG, "Find average angle for contour: prev (" << gPrev << "), "
                                                                  "cur (" << gCur  << "), "
                                                                 "next (" << gNext << ")" );

    // first, find if the line turns left or right ar src
    // for this, take the cross product of the vectors from prev to src, and src to next.
    // if the cross product is negetive, we've turned to the left
    // if the cross product is positive, we've turned to the right
    courseCur = SGGeodesy::courseDeg( gCur, gPrev );
    dirCur = SGVec3d( sin( courseCur*SGD_DEGREES_TO_RADIANS ), cos( courseCur*SGD_DEGREES_TO_RADIANS ), 0.0f );

    courseNext = SGGeodesy::courseDeg( gCur, gNext );
    dirNext = SGVec3d( sin( courseNext*SGD_DEGREES_TO_RADIANS ), cos( courseNext*SGD_DEGREES_TO_RADIANS ), 0.0f );

    // Now find the average
    dirAvg = normalize( dirCur + dirNext );
    courseAvg = SGMiscd::rad2deg( atan( dirAvg.x()/dirAvg.y() ) );
    if (courseAvg < 0) {
        courseAvg += 180.0f;
    }

    // check the turn direction
    cp    = cross( dirCur, dirNext );
    theta = SGMiscd::rad2deg(CalculateTheta( dirCur, dirNext ) );

    if ( (abs(theta - 180.0) < 0.1) || (abs(theta) < 0.1) || (isnan(theta)) ) {
        // straight line blows up math - offset 90 degree and dist is as given
        courseOffset = SGMiscd::normalizePeriodic(0, 360, courseNext-90.0);
        distOffsetInner  = offset_by+width/2.0f;
        distOffsetOuter  = offset_by-width/2.0f;
    }  else  {
        // calculate correct distance for the offset point
        if (cp.z() < 0.0f) {
            courseOffset = SGMiscd::normalizePeriodic(0, 360, courseAvg+180);
            turn_dir = 0;
        } else {
            courseOffset = SGMiscd::normalizePeriodic(0, 360, courseAvg);
            turn_dir = 1;
        }
        distOffsetInner = (offset_by+width/2.0f)/sin(SGMiscd::deg2rad(courseNext-courseOffset));
        distOffsetOuter = (offset_by-width/2.0f)/sin(SGMiscd::deg2rad(courseNext-courseOffset));
    }

    // calculate the points from cur
    inner = SGGeodesy::direct(gCur, courseOffset, distOffsetInner);
    outer = SGGeodesy::direct(gCur, courseOffset, distOffsetOuter);
}

void OffsetPointsMiddle( const SGGeod& gPrev, const SGGeod& gCur, const SGGeod& gNext, double offset_by, double width, SGGeod& inner, SGGeod& outer )
{
    int unused;
    return OffsetPointsMiddle( gPrev, gCur, gNext, offset_by, width, unused, inner, outer );
}


void OffsetPointsFirst( const SGGeod& cur, const SGGeod& next, double offset_by, double width, SGGeod& inner, SGGeod& outer )
{
    double courseOffset;

    SG_LOG(SG_GENERAL, SG_DEBUG, "Find OffsetPoint at Start : cur (" << cur  << "), "
                                                            "next (" << next << ")" );

    // find the offset angle
    courseOffset = SGGeodesy::courseDeg( cur, next ) - 90;
    courseOffset = SGMiscd::normalizePeriodic(0, 360, courseOffset);

    // calculate the point from cur
    inner = SGGeodesy::direct( cur, courseOffset, offset_by+width/2.0f );
    outer = SGGeodesy::direct( cur, courseOffset, offset_by-width/2.0f );
}

void OffsetPointsLast( const SGGeod& prev, const SGGeod& cur, double offset_by, double width, SGGeod& inner, SGGeod& outer )
{
    double courseOffset;

    SG_LOG(SG_GENERAL, SG_DEBUG, "Find OffsetPoint at End   : prev (" << prev  << "), "
                                                              "cur (" << cur << ")" );

    // find the offset angle
    courseOffset = SGGeodesy::courseDeg( prev, cur ) - 90;
    courseOffset = SGMiscd::normalizePeriodic(0, 360, courseOffset);

    // calculate the point from cur
    inner = SGGeodesy::direct( cur, courseOffset, offset_by+width/2.0f );
    outer = SGGeodesy::direct( cur, courseOffset, offset_by-width/2.0f );
}






SGGeod midpoint( const SGGeod& p0, const SGGeod& p1 )
{
    return SGGeod::fromDegM( (p0.getLongitudeDeg() + p1.getLongitudeDeg()) / 2,
                             (p0.getLatitudeDeg()  + p1.getLatitudeDeg()) / 2,
                             (p0.getElevationM()   + p1.getElevationM()) / 2 );
}

bool intersection(const SGGeod &p0, const SGGeod &p1, const SGGeod& p2, const SGGeod& p3, SGGeod& intersection)
{
    typedef CGAL::Exact_predicates_exact_constructions_kernel         Kernel;
    typedef Kernel::Point_2                                           Point_2;
    typedef CGAL::Segment_2<Kernel>                                   Segment_2;

    Point_2 a1( p0.getLongitudeDeg(), p0.getLatitudeDeg() );
    Point_2 b1( p1.getLongitudeDeg(), p1.getLatitudeDeg() );
    Point_2 a2( p2.getLongitudeDeg(), p2.getLatitudeDeg() );
    Point_2 b2( p3.getLongitudeDeg(), p3.getLatitudeDeg() );

    Segment_2 seg1( a1, b1 );
    Segment_2 seg2( a2, b2 );

    CGAL::Object result = CGAL::intersection(seg1, seg2);
    const CGAL::Point_2<Kernel> *ipoint = CGAL::object_cast<CGAL::Point_2<Kernel> >(&result);
    const CGAL::Segment_2<Kernel> *iseg = CGAL::object_cast<CGAL::Segment_2<Kernel> >(&result);
    
    if (ipoint) {
        // handle the point intersection case with *ipoint.
        return true;
    } else if (iseg ) {
        // handle the segment intersection case with *iseg.
        return true;
    } else {
        // handle the no intersection case.
        return false;
    }
}

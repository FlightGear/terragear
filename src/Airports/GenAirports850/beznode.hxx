#ifndef _BEZNODE_H_
#define _BEZNODE_H_

#include <vector>
#include <string.h>
#include <float.h>
#include <memory>

#include <simgear/debug/logstream.hxx>
#include <simgear/math/SGMath.hxx>

#include "debug.hxx"

inline double LinearDistance( const SGGeod& p0, const SGGeod& p1 )
{
    return SGGeodesy::distanceM( p0, p1 );
}

inline SGGeod CalculateLinearLocation( const SGGeod& p0, const SGGeod& p1, double t )
{
    // these are 2d approximations using lon, and lat as x,y cartesion coords
    // how expensive would this be to do in Geodetic math?
    SGVec2d v0 = SGVec2d( p0.getLongitudeDeg(), p0.getLatitudeDeg() );
    SGVec2d v1 = SGVec2d( p1.getLongitudeDeg(), p1.getLatitudeDeg() );
    SGVec2d result;

    double term1 = (1.0f - t);
    double term2 = t;

    result = (v0 * term1) + (v1 * term2);

    return SGGeod::fromDeg( result.x(), result.y() );
}

inline double QuadraticDistance( const SGGeod& p0, const SGGeod& cp, const SGGeod& p1 )
{
    return LinearDistance( p0, cp ) + LinearDistance( cp, p1 );
}

inline SGGeod CalculateQuadraticLocation( const SGGeod& p0, const SGGeod& cp, const SGGeod& p1, double t )
{
    SGVec2d v0  = SGVec2d( p0.getLongitudeDeg(), p0.getLatitudeDeg() );
    SGVec2d v1  = SGVec2d( p1.getLongitudeDeg(), p1.getLatitudeDeg() );
    SGVec2d vcp = SGVec2d( cp.getLongitudeDeg(), cp.getLatitudeDeg() );
    SGVec2d result;

    double term1 = (1.0f - t) * (1.0f - t);
    double term2 = 2 * (1.0f - t) * t;
    double term3 = t * t; 

    result = (v0 * term1) + (vcp * term2) + (v1 * term3);

    return SGGeod::fromDeg( result.x(), result.y() );
}

inline double CubicDistance( const SGGeod& p0, const SGGeod& cp0, const SGGeod& cp1, const SGGeod& p1 )
{
    return LinearDistance( p0, cp0 ) + LinearDistance( cp0, cp1 ) + LinearDistance( cp1, p1 );
}

inline SGGeod CalculateCubicLocation( const SGGeod& p0, const SGGeod& cp0, const SGGeod& cp1, const SGGeod& p1, double t )
{
    SGVec2d v0   = SGVec2d( p0.getLongitudeDeg(),  p0.getLatitudeDeg() );
    SGVec2d v1   = SGVec2d( p1.getLongitudeDeg(),  p1.getLatitudeDeg() );
    SGVec2d vcp0 = SGVec2d( cp0.getLongitudeDeg(), cp0.getLatitudeDeg() );
    SGVec2d vcp1 = SGVec2d( cp1.getLongitudeDeg(), cp1.getLatitudeDeg() );
    SGVec2d result;

    double term1 = (1.0f - t) * (1.0f - t) * (1.0f - t);
    double term2 = 3 * (1.0f - t) * (1.0f - t) * t;
    double term3 = 3 * (1.0f - t) * t * t;
    double term4 = t * t * t;

    result = (v0 * term1) + (vcp0 * term2) + (vcp1 * term3) + (v1 * term4);

    return SGGeod::fromDeg( result.x(), result.y() );
}

inline double CalculateTheta( const SGGeod& p0, const SGGeod& p1, const SGGeod& p2 )
{
    SGVec2d v0, v1, v2;
    SGVec2d u, v;
    double  udist, vdist, uv_dot;

    v0 = SGVec2d( p0.getLongitudeDeg(), p0.getLatitudeDeg() );
    v1 = SGVec2d( p1.getLongitudeDeg(), p1.getLatitudeDeg() );
    v2 = SGVec2d( p2.getLongitudeDeg(), p2.getLatitudeDeg() );

    u  = v1 - v0;
    udist = norm(u);

    v = v1 - v2;
    vdist = norm(v);

    uv_dot = dot(u, v);

    return acos( uv_dot / (udist * vdist) );
}

/* cal theta when we have unit vectors */
/* TODO : Use cp to determine right or left turn */
inline double CalculateTheta( const SGVec3d& dirCur, const SGVec3d& dirNext, const SGVec3d& cp )
{
    double dp = dot( dirCur, dirNext );

    return acos( dp );
}


#define BEZIER_DETAIL   (8)
#define LINE_WIDTH      (0.75)
#define WIREFRAME       (1)

#define CURVE_NONE                      (0)
#define CURVE_LINEAR                    (1)
#define CURVE_QUADRATIC                 (2)
#define CURVE_CUBIC                     (3)


class BezNode 
{
public:
    explicit BezNode( SGGeod l ) :
        loc(l)
    {
        has_prev_cp = false;
        has_next_cp = false;

        mark  = 0;
        light = 0;
        term  = false;
        close = false;
    }

    BezNode( double lat, double lon ) :
        BezNode(SGGeod::fromDeg(lon, lat))
    {
    }

    BezNode( SGGeod l, SGGeod cp ) :
        loc(l),
        prev_cp(Mirror(cp)),
        next_cp(cp)
    {
        has_prev_cp = true;
        has_next_cp = true;

        mark    = 0;
        light   = 0;
        term    = false;
        close   = false;
    }

    BezNode( double lat, double lon, double cp_lat, double cp_lon ) :
        BezNode(SGGeod::fromDeg(lon, lat), SGGeod::fromDeg(cp_lon, cp_lat))
    {
    }

    SGGeod Mirror( const SGGeod& pt )
    {
        double heading, az2, dist;

        // mirror given point about our location
        SGGeodesy::inverse( loc, pt, heading, az2, dist );
        heading = SGMiscd::normalizePeriodic(0, 360, heading + 180);

        return SGGeodesy::direct(loc, heading, dist);
    }

    void SetMarking( unsigned int m )
    {
        mark = m;
    }

    unsigned int GetMarking( )
    {
        return mark;
    }

    void SetLighting( unsigned int l )
    {
        light = l;
    }

    unsigned int GetLighting( )
    {
        return light;
    }

    void SetClose( bool c )
    {
        close = c;
    }

    void SetTerm( bool t )
    {
        term = t;
    }

    bool IsAt( double lat, double lon )
    {
        return ( (loc.getLatitudeDeg() == lat) && (loc.getLongitudeDeg() == lon) );
    }

    bool HasPrevCp()
    {
        return has_prev_cp;
    }

    bool HasNextCp()
    {
        return has_next_cp;
    }

    SGGeod GetLoc()
    {
        return loc;
    }

    SGGeod GetPrevCp()
    {
        return prev_cp;
    }

    SGGeod GetNextCp()
    {
        return next_cp;
    }

    void ClearNextCp(void)
    {
        has_next_cp = false;
    }

    void SetNextCp( double lat, double lon )
    {
        next_cp = SGGeod::fromDeg(lon, lat);
        has_next_cp = true;
    }

    void Print()
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, 
            "\tLoc:     " << loc     << "\n" <<
            "\tprev_cp: " << prev_cp << "\n" <<
            "\tnext_cp: " << next_cp << "\n" );
    }

private:
    SGGeod          loc;
    SGGeod          prev_cp;
    bool            has_prev_cp;
    SGGeod          next_cp;
    bool            has_next_cp;

    unsigned int    mark;
    unsigned int    light;
    bool            term;
    bool            close;
};


// array of BezNodes make a contour
typedef std::vector<std::shared_ptr<BezNode>> BezContour;
typedef std::vector<BezContour> BezContourArray;

#endif


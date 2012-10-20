#ifndef _BEZNODE_H_
#define _BEZNODE_H_

#include <string.h>
#include <float.h>
#include <iostream>

#include <Geometry/point3d.hxx>
#include <simgear/debug/logstream.hxx>

// TEMP...
inline double LinearDistance( Point3D p0, Point3D p1 )
{
    return sqrt( pow( fabs( p1.x()-p0.x() ), 2 ) + 
                 pow( fabs( p1.y()-p0.y() ), 2 ) );
}

inline Point3D CalculateLinearLocation( Point3D p0, Point3D p1, double t )
{
    Point3D result;

    double term1 = (1.0f - t);
    double term2 = t;

    result = (p0 * term1) + (p1 * term2);

    return result;
}

inline double QuadraticDistance( Point3D p0, Point3D cp, Point3D p1 )
{
    return LinearDistance( p0, cp ) + LinearDistance( cp, p1 );
}

inline Point3D CalculateQuadraticLocation( Point3D p0, Point3D cp, Point3D p1, double t )
{
    Point3D result;

    double term1 = (1.0f - t) * (1.0f - t);
    double term2 = 2 * (1.0f - t) * t;
    double term3 = t * t; 

    result = (p0 * term1) + (cp * term2) + (p1 * term3);

    return result;
}

inline double CubicDistance( Point3D p0, Point3D cp0, Point3D cp1, Point3D p1 )
{
    return LinearDistance( p0, cp0 ) + LinearDistance( cp0, cp1 ) + LinearDistance( cp1, p1 );
}

inline Point3D CalculateCubicLocation( Point3D p0, Point3D cp0, Point3D cp1, Point3D p1, double t )
{
    Point3D result;

    double term1 = (1.0f - t) * (1.0f - t) * (1.0f - t);
    double term2 = 3 * (1.0f - t) * (1.0f - t) * t;
    double term3 = 3 * (1.0f - t) * t * t;
    double term4 = t * t * t;

    result = (p0 * term1) + (cp0 * term2) + (cp1 * term3) + (p1 * term4);

    return result;
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
    BezNode( Point3D l ) : prev_cp(0.0f, 0.0f, 0.0f), next_cp(0.0f, 0.0f, 0.0f)
    {
        loc   = l;
        mark  = 0;
        light = 0;
        term  = false;
        close = false;
    }

    BezNode( double lat, double lon ) : prev_cp(0.0f, 0.0f, 0.0f), next_cp(0.0f, 0.0f, 0.0f)
    {
        loc   = Point3D( lon, lat, 0.0f );
        mark  = 0;
        light = 0;
        term  = false;
        close = false;
    }

    BezNode( Point3D l, Point3D cp )
    {
        loc     = l;
        next_cp = cp;
        prev_cp = Mirror(cp);
        mark    = 0;
        light   = 0;
        term    = false;
        close   = false;
    }

    BezNode( double lat, double lon, double cp_lat, double cp_lon )
    {
        loc     = Point3D( lon, lat, 0.0f );
        next_cp = Point3D( cp_lon, cp_lat, 0.0f );
        prev_cp = Mirror( next_cp );
        mark    = 0;
        light   = 0;
        term    = false;
        close   = false;
    }

    Point3D Mirror( Point3D pt )
    {
        // mirror given point about our location
        return (loc - (pt - loc));
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
        return ( (loc.lat() == lat) && (loc.lon() == lon) );
    }

    bool HasPrevCp()
    {
        return ( (prev_cp.x() != 0.0f) && (prev_cp.y() != 0.0f) );
    }

    bool HasNextCp()
    {
        return ( (next_cp.x() != 0.0f) && (next_cp.y() != 0.0f) );
    }

    Point3D GetLoc()
    {
        return loc;
    }

    Point3D GetPrevCp()
    {
        return prev_cp;
    }

    Point3D GetNextCp()
    {
        return next_cp;
    }

    void ClearNextCp(void)
    {
        next_cp = Point3D(0.0f,0.0f, 0.0f);
    }

    void SetNextCp( double lat, double lon )
    {
        next_cp = Point3D(lon, lat, 0.0f);
    }

    // TODO: log levels and macros (does terragear have them?)
    void Print()
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, 
            "\tLoc(" << loc.x() << "," << loc.y() << ")" <<
            "\tprev_cp(" << prev_cp.x() << "," << prev_cp.y() << ")" <<
            "\tnext_cp(" << next_cp.x() << "," << next_cp.y() << ")" );
    }

private:
    Point3D         loc;
    Point3D         prev_cp;
    Point3D         next_cp;
    unsigned int    mark;
    unsigned int    light;
    bool            term;
    bool            close;
};


// array of BezNodes make a contour
typedef std::vector <BezNode *> BezContour;
typedef std::vector <BezContour *> BezContourArray;

#endif


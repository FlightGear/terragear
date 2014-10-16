#ifndef __TG_EUCLIDEAN_HXX__
#define __TG_EUCLIDEAN_HXX__

#include <simgear/compiler.h>
#include <simgear/math/sg_types.hxx>
        
#define USE_EUCLIDEAN   (1)

class TGEuclidean 
{
public:
    static SGGeod direct( const SGGeod& pos, double course, double distm ) {
#if USE_EUCLIDEAN
        double angle  = SGMiscd::normalizePeriodic( 0, 360, 90-course );
        double distd  = (distm)/111000;    
        return SGGeod::fromDeg( pos.getLongitudeDeg() + distd*cos(SGMiscd::deg2rad(angle)), 
                                pos.getLatitudeDeg() + distd*sin(SGMiscd::deg2rad(angle)) );
#else
        return SGGeodesy::direct( pos, course, distm );
#endif        
    }
    
#if 1    
    static double courseRad( const SGGeod& start, const SGGeod& end ) {
#if USE_EUCLIDEAN        
        return SGMiscd::normalizeAngle2( 
                        -atan2( end.getLatitudeDeg()  - start.getLatitudeDeg(),
                                end.getLongitudeDeg() - start.getLongitudeDeg() ) + SGD_PI/2 );
#else
        return SGGeodesy::courseRad( start, end );
#endif        
    }
#endif

    static double courseDeg( const SGGeod& start, const SGGeod& end ) {
#if USE_EUCLIDEAN        
        return SGMiscd::rad2deg( courseRad( start, end ) );
#else
        return SGGeodesy::courseDeg( start, end );
#endif        
    }

    static double distanceM( const SGGeod& start, const SGGeod& end ) {
#if USE_EUCLIDEAN
        double x1 = start.getLongitudeDeg();
        double y1 = start.getLatitudeDeg();
        double x2 = end.getLongitudeDeg();
        double y2 = end.getLatitudeDeg();

        return sqrt( pow(x1-x2, 2) + pow(y1-y2, 2) );
#else
        return SGGeodesy::distanceM( start, end );
#endif        
    }
};

#endif /* __TG_EUCLIDEAN_HXX__ */
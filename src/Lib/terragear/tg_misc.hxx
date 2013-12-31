#include <simgear/math/SGMath.hxx>

#include "clipper.hpp"
#include "tg_rectangle.hxx"

// SGBucket corners
#define SG_BUCKET_SW    (0)
#define SG_BUCKET_SE    (1)
#define SG_BUCKET_NE    (2)
#define SG_BUCKET_NW    (3)


// SGGeod Cleanup
SGGeod  SGGeod_snap( const SGGeod& in, double grid );

// SGGeod Conversion
SGVec2d              SGGeod_ToSGVec2d( const SGGeod& p );
ClipperLib::IntPoint SGGeod_ToClipper( const SGGeod& p );
SGGeod               SGGeod_FromClipper( const ClipperLib::IntPoint& p );

// SGGeod Equivelence test
bool    SGGeod_isEqual2D( const SGGeod& g0, const SGGeod& g1 );

// SGGeod Angle calculation
double  SGGeod_CalculateTheta( const SGGeod& p0, const SGGeod& p1, const SGGeod& p2 );

// SGVec2d Angle calculation
double SGVec2d_CalculateTheta( const SGVec2d& v0, const SGVec2d& v1, const SGVec2d& v2);

// Angle calculation from 2 directions
double CalculateTheta( const SGVec3d& dirCur, const SGVec3d& dirNext );

// Clipper Conversion
double Dist_ToClipper( double dist );

// should be in rectangle
tgRectangle BoundingBox_FromClipper( const ClipperLib::Paths& subject );

bool intersection(const SGGeod &p0, const SGGeod &p1, const SGGeod& p2, const SGGeod& p3, SGGeod& intersection);


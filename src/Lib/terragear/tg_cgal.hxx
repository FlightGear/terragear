#ifndef __TG_CGAL_CONV_HXX__
#define __TG_CGAL_CONV_HXX__

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <simgear/math/SGMath.hxx>

#include "tg_euclidean.hxx"

// For the CGAL rays, segments and lines, we'll use exact constructions with square root
// square root is needed for CGAL::bisector()

typedef CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt CBKernel;
typedef CBKernel::Point_2                                           CBPoint_2;
typedef CBKernel::Direction_2                                       CBDirection_2;
typedef CGAL::Line_2<CBKernel>                                      CBLine_2;
typedef CGAL::Ray_2<CBKernel>                                       CBRay_2;
typedef CGAL::Vector_2<CBKernel>                                    CBVector_2;
typedef CGAL::Segment_2<CBKernel>                                   CBSegment_2;

// forward declarations
class tgLine;
class tgRay;
class tgSegment;

// abstract class for segments, rays, and lines
class tgCgalBase
{
public:
    static CBDirection_2 HeadingToDirection( double heading )
    {
        double angle = SGMiscd::normalizePeriodic(0, 360, 90-heading);
    
        double dy    = sin( SGMiscd::deg2rad(angle) );
        double dx    = cos( SGMiscd::deg2rad(angle) );
    
        return CBDirection_2(dx,dy);
    }
        
    static double DirectionToHeading( CBDirection_2 dir )
    {
        double angle = SGMiscd::rad2deg( atan2( CGAL::to_double(dir.dy()), CGAL::to_double(dir.dx()) ) );
    
        return SGMiscd::normalizePeriodic( 0, 360, -(angle-90) );
    }
    
protected:
    CBDirection_2           cg_dir;
};

typedef std::vector <tgCgalBase>  tgcgalbase_list;
typedef tgcgalbase_list::iterator tgcgalbase_list_iterator;
typedef tgcgalbase_list::const_iterator const_tgcgalbase_list_iterator;

class tgSegment : public tgCgalBase
{
public:
    tgSegment() {}
    tgSegment( const SGGeod& s, const SGGeod& e ) : start(s), end(e) {}

    CBSegment_2 toCgal( void ) const
    {    
        CBPoint_2 a1( start.getLongitudeDeg(), start.getLatitudeDeg() );
        CBPoint_2 a2( end.getLongitudeDeg(), end.getLatitudeDeg() );

        return CBSegment_2( a1, a2 );
    }    

    static tgSegment Reverse( const tgSegment& subject ) {
        return ( tgSegment( subject.end, subject.start ) );
    }

    bool Intersect( const tgSegment& seg, SGGeod& intersection); 
    bool Intersect( const tgRay& ray, SGGeod& intersection);    
    bool Intersect( const tgLine& line, SGGeod& intersection);
    
    SGGeod       start;
    SGGeod       end;
};

typedef std::vector <tgSegment>  tgsegment_list;
typedef tgsegment_list::iterator tgsegment_list_iterator;
typedef tgsegment_list::const_iterator const_tgsegment_list_iterator;

class tgRay : public tgCgalBase
{
public:
    tgRay() {}
    tgRay( const SGGeod& s, double h ) : start(s), heading(h) {}
    tgRay( const SGGeod& s, const SGGeod& e ) : start(s) {
        heading = TGEuclidean::courseDeg( s, e );
    }

    static tgRay Reverse( const tgRay& subject ) {
        return ( tgRay( subject.start, SGMiscd::normalizePeriodic(0, 360, subject.heading + 180) ) );
    }

    CBRay_2 toCgal( void ) const
    {    
        CBPoint_2 a2( start.getLongitudeDeg(), start.getLatitudeDeg() );
        CBRay_2   ray2( a2, HeadingToDirection( heading ) );

        return ray2;
    }    
    
    bool Intersect( const tgSegment& seg, SGGeod& intersection);
    bool Intersect( const tgRay& ray, SGGeod& intersection);
    bool Intersect( const tgLine& line, SGGeod& intersection);
    
    SGGeod       start;
    double       heading;
};

typedef std::vector <tgRay>  tgray_list;
typedef tgray_list::iterator tgray_list_iterator;
typedef tgray_list::const_iterator const_tgray_list_iterator;



class tgLine : public tgCgalBase
{
public:
    tgLine() {}
    tgLine( const SGGeod& s, const SGGeod& e ) : start(s), end(e) {}

    static tgLine Reverse( const tgLine& subject ) {
        return ( tgLine( subject.end, subject.end ) );
    }

    CBLine_2 toCgal( void ) const
    {    
        CBPoint_2 a1( start.getLongitudeDeg(), start.getLatitudeDeg() );
        CBPoint_2 a2( end.getLongitudeDeg(), end.getLatitudeDeg() );

        return CBLine_2( a1, a2 );
    }    
    
    bool Intersect( const tgSegment& seg, SGGeod& intersection);
    bool Intersect( const tgRay& ray, SGGeod& intersection);    
    bool Intersect( const tgLine& line, SGGeod& intersection);
    
    SGGeod       start;
    SGGeod       end;
};

typedef std::vector <tgLine>  tgline_list;
typedef tgline_list::iterator tgline_list_iterator;
typedef tgline_list::const_iterator const_tgline_list_iterator;

double Bisect( const SGGeod& p1, double h1, double h2, bool right );

#endif /* __TG_CGAL_CONV_HXX__ */
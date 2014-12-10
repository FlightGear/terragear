#ifndef __TG_CGAL_CONV_HXX__
#define __TG_CGAL_CONV_HXX__

#include <simgear/math/SGMath.hxx>

#include "tg_cgal_epec.hxx"
#include "tg_euclidean.hxx"
#include "tg_rectangle.hxx"
#include "tg_misc.hxx"

// forward declarations
class tgLine;
class tgRay;
class tgSegment;

// abstract class for segments, rays, and lines
class tgCgalBase
{
public:    
    static EPECDirection_2 HeadingToDirection( double heading )
    {
        double angle = SGMiscd::normalizePeriodic(0, 360, 90-heading);
    
        double dy    = sin( SGMiscd::deg2rad(angle) );
        double dx    = cos( SGMiscd::deg2rad(angle) );
    
        return EPECDirection_2(dx,dy);
    }
        
    static double DirectionToHeading( EPECDirection_2 dir )
    {
        double angle = SGMiscd::rad2deg( atan2( CGAL::to_double(dir.dy()), CGAL::to_double(dir.dx()) ) );
    
        return SGMiscd::normalizePeriodic( 0, 360, -(angle-90) );
    }

    static SGGeod EPECPointToGeod( const EPECPoint_2& pt )
    {
        return SGGeod::fromDeg( CGAL::to_double( pt.x() ), CGAL::to_double( pt.y() ) );
    }
    
    double GetHeading( void ) const { return DirectionToHeading(dir); }

    EPECPoint_2     GetCGALStart( void ) const { return start; }
    EPECPoint_2     GetCGALEnd  ( void ) const { return end;   }
    
    SGGeod          GetGeodStart( void ) const { return EPECPointToGeod( start ); }
    SGGeod          GetGeodEnd  ( void ) const { return EPECPointToGeod( end   ); }
    
    // All internal data kept full CGAL precision
protected:
    EPECDirection_2 dir;
    EPECPoint_2     start;
    EPECPoint_2     end;
};

typedef std::vector <tgCgalBase>  tgcgalbase_list;
typedef tgcgalbase_list::iterator tgcgalbase_list_iterator;
typedef tgcgalbase_list::const_iterator const_tgcgalbase_list_iterator;

class tgSegment : public tgCgalBase
{
public:
    tgSegment() {}
    
    tgSegment( const SGGeod& s, const SGGeod& e ) {
        start = EPECPoint_2( s.getLongitudeDeg(), s.getLatitudeDeg() );
        end   = EPECPoint_2( e.getLongitudeDeg(), e.getLatitudeDeg() );
        dir   = EPECDirection_2( EPECSegment_2( start, end ) );
    }

    tgSegment( const EPECPoint_2& s, const EPECPoint_2& e ) {
        start = s;
        end   = e;
        dir   = EPECDirection_2( EPECSegment_2( start, end ) );
    }

    EPECSegment_2 toCgal( void ) const
    {    
        return EPECSegment_2( start, end );
    }    
    
    bool isEqual( const tgSegment& seg ) const {
        return ( ( start == seg.start ) && ( end == seg.end ) );
    }

    static tgSegment Reverse( const tgSegment& subject ) {
        return ( tgSegment( subject.end, subject.start ) );
    }

    tgRectangle GetBoundingBox( void ) const;
    
    bool Intersect( const tgSegment& seg, SGGeod& intersection) const; 
    bool Intersect( const tgRay& ray,     SGGeod& intersection) const;    
    bool Intersect( const tgLine& line,   SGGeod& intersection) const;
    
    bool Intersect( const tgSegment& seg, SGGeod* iPos, tgSegment* iSeg ) const;
    bool IsOnLine( const SGGeod& pos ) const;
    
    double DistanceFromOld( const SGGeod& pos ) const
    {
        EPECSegment_2 seg( start, end );
        EPECPoint_2   b(pos.getLongitudeDeg(), pos.getLatitudeDeg() );
        
        return sqrt( CGAL::to_double(CGAL::squared_distance(seg, b) ) );
    }
    
    double DistanceFrom( const SGGeod& pos, SGGeod& proj ) const
    {
        EPECPoint_2   b( pos.getLongitudeDeg(), pos.getLatitudeDeg() );
        
        EPECLine_2    lin( start, end );
        EPECSegment_2 seg( start, end );
        
        EPECPoint_2   p = lin.projection( b ); 
        
        if ( !CGAL::do_overlap( p.bbox(), seg.bbox() ) ) {
            double dist_start_sq = CGAL::to_double(CGAL::squared_distance( start, b ) );
            double dist_end_sq   = CGAL::to_double(CGAL::squared_distance( end,   b ) );
            
            if ( dist_start_sq < dist_end_sq ) {
                p = start;
            } else {
                p = end;
            }
        }
        
        proj = SGGeod::fromDeg( CGAL::to_double(p.x()), CGAL::to_double(p.y()) );
        
        return SGGeodesy::distanceM( pos, proj );
    }    
};

typedef std::vector <tgSegment>  tgsegment_list;
typedef tgsegment_list::iterator tgsegment_list_iterator;
typedef tgsegment_list::const_iterator const_tgsegment_list_iterator;

class tgRay : public tgCgalBase
{
public:
    tgRay( const SGGeod& s, double h ) {
        start = EPECPoint_2( s.getLongitudeDeg(), s.getLatitudeDeg() );
        dir   = HeadingToDirection( h );
    }

    tgRay( const EPECPoint_2& s, EPECDirection_2 d ) {
        start = s;
        dir   = d;
    }
    
    tgRay( const SGGeod& s, const SGGeod& e ) {
        start = EPECPoint_2( s.getLongitudeDeg(), s.getLatitudeDeg() );
        end   = EPECPoint_2( e.getLongitudeDeg(), e.getLatitudeDeg() );
        dir   = EPECDirection_2( EPECSegment_2( start, end ) );
    }

    static tgRay Reverse( const tgRay& subject ) {
        return ( tgRay( subject.start, -(subject.dir) ) );
    }

    EPECRay_2 toCgal( void ) const
    {    
        return EPECRay_2( start, dir );
    }    
    
    bool Intersect( const tgSegment& seg, SGGeod& intersection) const;
    bool Intersect( const tgRay& ray,     SGGeod& intersection) const;
    bool Intersect( const tgLine& line,   SGGeod& intersection) const;    
};

typedef std::vector <tgRay>  tgray_list;
typedef tgray_list::iterator tgray_list_iterator;
typedef tgray_list::const_iterator const_tgray_list_iterator;



class tgLine : public tgCgalBase
{
public:
    tgLine() {}
    tgLine( const SGGeod& s, const SGGeod& e ) {
        start = EPECPoint_2( s.getLongitudeDeg(), s.getLatitudeDeg() );
        end   = EPECPoint_2( e.getLongitudeDeg(), e.getLatitudeDeg() );
        dir   = EPECDirection_2( EPECSegment_2( start, end ) );
    }

    tgLine( const SGGeod& s, double h ) {
        EPECLine_2 line( EPECPoint_2(s.getLongitudeDeg(), s.getLatitudeDeg()), HeadingToDirection(h) );
        
        start = line.point(0);
        end   = line.point(1);
        dir   = line.direction();
    }
    
    tgLine( const EPECPoint_2& s, const EPECPoint_2& e ) {
        start = s;
        end   = e;
        dir   = EPECDirection_2( EPECSegment_2( start, end ) );
    }
    
    static tgLine Reverse( const tgLine& subject ) {
        return ( tgLine( subject.end, subject.start ) );
    }

    EPECLine_2 toCgal( void ) const
    {    
        return EPECLine_2( start, end );
    }    

    int OrientedSide( const SGGeod& pt ) {
        EPECLine_2  line( start, end );
        EPECPoint_2 point( pt.getLongitudeDeg(), pt.getLatitudeDeg() );
    
        return ( (int)line.oriented_side( point ) );
    }
    
    bool isOn( const SGGeod& pt ) const 
    {
        bool on = false;

        SGGeod s = SGGeod::fromDeg( CGAL::to_double(start.x()), CGAL::to_double(start.y()) );
        SGGeod e = SGGeod::fromDeg( CGAL::to_double(end.x()),   CGAL::to_double(end.y())   );
        
        on = IsNodeCollinear( s, e, pt );
        if (!on) {
            on = SGGeod_isEqual2D( s, pt );
        }
        if (!on) {
            on = SGGeod_isEqual2D( e, pt );
        }
        
        if (!on) {
            EPECLine_2  a = toCgal();
            EPECPoint_2 b( pt.getLongitudeDeg(), pt.getLatitudeDeg() );
            
            on = a.has_on( b );
        }
        
        return on;
    }
    
    bool Intersect( const tgSegment& seg, SGGeod& intersection) const;
    bool Intersect( const tgRay& ray, SGGeod& intersection) const;    
    bool Intersect( const tgLine& line, SGGeod& intersection) const;
};

typedef std::vector <tgLine>  tgline_list;
typedef tgline_list::iterator tgline_list_iterator;
typedef tgline_list::const_iterator const_tgline_list_iterator;

double Bisect( const SGGeod& p1, double h1, double h2, bool right );

#endif /* __TG_CGAL_CONV_HXX__ */
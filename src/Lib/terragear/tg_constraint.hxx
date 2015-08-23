#ifndef __TG_CONSTRAINT_HXX__
#define __TG_CONSTRAINT_HXX__

#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Arr_linear_traits_2.h>
#include <CGAL/Arrangement_with_history_2.h>
#include <CGAL/Arr_extended_dcel.h>
#include "tg_cgal_epec.hxx"

// class for CGAL consolidated curve data
class ConstraintData {
public:
    ConstraintData() {}
    ConstraintData( unsigned long i, const std::string& d ) : id(i), description(d) {}
    
    bool operator==(const ConstraintData& rhs) const {
        return id == rhs.id && peer == rhs.peer && description == rhs.description;
    }
    
    unsigned long   id;
    unsigned long   peer;
    std::string     description;    
};

typedef EPECKernel                                                      edgeArrKernel;
typedef CGAL::Aff_transformation_2<edgeArrKernel>                       edgeArrTransformation; 
typedef CGAL::Arr_linear_traits_2<edgeArrKernel>                        edgeArrLinearTraits;
typedef edgeArrLinearTraits::Curve_2                                    edgeArrLinearCurve;
typedef CGAL::Arr_consolidated_curve_data_traits_2<edgeArrLinearTraits, ConstraintData> edgeArrTraits;
typedef edgeArrTraits::Point_2                                          edgeArrPoint;
typedef edgeArrTraits::Segment_2                                        edgeArrSegment;
typedef edgeArrTraits::Ray_2                                            edgeArrRay;
typedef edgeArrTraits::Line_2                                           edgeArrLine;
typedef edgeArrTraits::Curve_2                                          edgeArrCurve;
typedef CGAL::Arr_extended_dcel<edgeArrTraits,bool, bool, bool>         edgeArrDcel;
typedef CGAL::Arrangement_with_history_2<edgeArrTraits, edgeArrDcel>    edgeArrangement;

typedef edgeArrangement::Ccb_halfedge_const_circulator                  edgeArrCcbHEConstCirc;
typedef edgeArrangement::Ccb_halfedge_circulator                        edgeArrCcbHECirc;
typedef edgeArrangement::Halfedge_const_handle                          edgeArrHEConstHandle;
typedef edgeArrangement::Curve_handle                                   edgeArrCurveHandle;
typedef edgeArrangement::Vertex_handle                                  edgeArrVertexHandle;
typedef edgeArrangement::Face_handle                                    edgeArrFaceHandle;

typedef std::vector<edgeArrCurveHandle>                                 ConstraintArray;

// todo : getchecklist is dumb.
#if 0
typedef enum {
    BOT_LEFT_CONSTRAINT = 0,
    LEFT_SIDE_CONSTRAINT,
    TOP_LEFT_CONSTRAINT,
    BOT_RIGHT_CONSTRAINT,
    RIGHT_SIDE_CONSTRAINT,
    TOP_RIGHT_CONSTRAINT,
    NUM_CONSTRAINTS
} ConstraintPos_e;
#else
typedef enum {
    BOT_LEFT_CONSTRAINT = 0,
    LEFT_SIDE_CONSTRAINT,
    TOP_LEFT_CONSTRAINT,
    BOT_RIGHT_CONSTRAINT,
    RIGHT_SIDE_CONSTRAINT,
    TOP_RIGHT_CONSTRAINT,
    NUM_CONSTRAINTS
} ConstraintPos_e;
#endif

class tgConstraint
{
public:
    typedef enum {
        consSegment = 0,
        consRay     = 1,
        consLine    = 2
    } consType_e;
    
    tgConstraint( consType_e t, const edgeArrPoint& s, const edgeArrPoint& e, unsigned long p, const std::string& desc ) {
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
        
        ConstraintData     d = ConstraintData( p, desc );
        curve = edgeArrCurve( c, d );
    }
    
    static tgConstraint fromSegment( const edgeArrPoint& s, const edgeArrPoint& e, unsigned long p, const std::string& desc ) {
        return tgConstraint( consSegment, s, e, p, desc );
    }
    
    static tgConstraint fromRay( const edgeArrPoint& s, const edgeArrPoint& e, unsigned long p, const std::string& desc ) {
        return tgConstraint( consRay, s, e, p, desc );
    }
    
    static tgConstraint fromRay( const edgeArrPoint& s, double h, unsigned long p, const std::string& desc ) {
        // Use Geodesy function to compute offset point
        SGGeod sg = SGGeod::fromDeg( CGAL::to_double( s.x() ), CGAL::to_double( s.y() ) );
        SGGeod eg = SGGeodesy::direct( sg, h, 0.1 );
        edgeArrPoint e = edgeArrPoint( eg.getLongitudeDeg(), eg.getLatitudeDeg() );
        
        return tgConstraint( consRay, s, e, p, desc );
    }
    
    static tgConstraint fromLine( const edgeArrPoint& s, edgeArrPoint& e, unsigned long p, const std::string& desc ) {
        
        return tgConstraint( consLine, s, e, p, desc );
    }
    
    bool breakWith( const tgConstraint& other ) {
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
    
    bool hasOn( edgeArrPoint& pt ) {
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
    
    void breakAt( edgeArrPoint& pt ) {
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
    
    bool Intersect( const tgConstraint& other, edgeArrPoint& intersectionLocation ) {
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
    
    // todo : this is stupid...
    static std::vector<ConstraintPos_e> GetChecklist( ConstraintPos_e pos ) {
        std::vector<ConstraintPos_e> checkList;
        
        if ( pos <= TOP_LEFT_CONSTRAINT ) {
            checkList.push_back( BOT_LEFT_CONSTRAINT );
            checkList.push_back( LEFT_SIDE_CONSTRAINT );
            checkList.push_back( TOP_LEFT_CONSTRAINT );
        } else {
            checkList.push_back( BOT_RIGHT_CONSTRAINT );
            checkList.push_back( RIGHT_SIDE_CONSTRAINT );
            checkList.push_back( TOP_RIGHT_CONSTRAINT );
        }
        
        return checkList;
    }
    
    
    void                        setId( unsigned long edgeid ) { curve.data().id = edgeid; }
    edgeArrPoint                getStart( void ) const { return start; }
    edgeArrPoint                getEnd( void ) const { return end; }
    
    consType_e                  getType( void )  const  { return type; }
    double                      getHeading( void ) const { return heading; }
    const std::string&          getDescription( void ) const { return curve.data().description; }
    unsigned int                getPeer( void ) const { return curve.data().peer; }
    
    bool                        is_line( void ) const  { return curve.is_line(); }
    edgeArrKernel::Line_2       line( void ) const { return curve.line(); }
    bool                        is_ray( void ) const  { return curve.is_ray(); }
    edgeArrKernel::Ray_2        ray( void ) const { return curve.ray(); }
    bool                        is_segment( void ) const  { return curve.is_segment(); }
    edgeArrKernel::Segment_2    segment( void ) const { return curve.segment(); }
    
    edgeArrCurve                getCurve( void ) const { return curve; }
    
private:
    edgeArrPoint    start;
    edgeArrPoint    end;
    edgeArrCurve    curve;
    double          heading;
    consType_e      type;
};

#endif /* __TG_CONSTRAINT_HXX__ */

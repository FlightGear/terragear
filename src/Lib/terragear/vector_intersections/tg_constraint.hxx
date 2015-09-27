#ifndef __TG_CONSTRAINT_HXX__
#define __TG_CONSTRAINT_HXX__

// tg_constraint.hxx
// 
// Define segments, lines, or rays to constrain a polygon.
// The polygon shape is derived from a network of edges.
//
// Written by Peter Sadrozinski, started March 2014.
//
// Copyright (C) 2014  Peter Sadrozinski - psadrozinski@gmail.com
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Arr_linear_traits_2.h>
#include <CGAL/Arrangement_with_history_2.h>
#include <CGAL/Arr_extended_dcel.h>

// With CGAL Arrangements with Consolidated Curve Data Traits, we can store data
// associated with each curve.
// Currently, we store the current edge identifier, the peer ( next edge 
// clockwise identifier ), and a description.
//
// we could just store a pointer to the containing (this) constraint object.
// There are positive and negetives with either approach.
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

// define how our edge arrangements are configured in CGAL.
// use exact predicates and exact constructions
// use tranform to translate points
// use ArrLinearTraits, so we have unbounded arrangements - rays and lines
// use Consolidated Data Traits to associate data to each curve
// use History to be able to find the original curve from halfedges
typedef CGAL::Exact_predicates_exact_constructions_kernel               edgeArrKernel;
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
    
    // Create a new constraint
    tgConstraint( consType_e t, const edgeArrPoint& s, const edgeArrPoint& e, unsigned long p, const std::string& desc );
    
    // Create a segment constraint from two points
    static tgConstraint fromSegment( const edgeArrPoint& s, const edgeArrPoint& e, unsigned long p, const std::string& desc ) {
        return tgConstraint( consSegment, s, e, p, desc );
    }
    
    // create a ray constraint from a source and target point
    static tgConstraint fromRay( const edgeArrPoint& s, const edgeArrPoint& e, unsigned long p, const std::string& desc ) {
        return tgConstraint( consRay, s, e, p, desc );
    }
    
    // create a ray constraint from a source and heading
    static tgConstraint fromRay( const edgeArrPoint& s, double h, unsigned long p, const std::string& desc ) {
        // Use Geodesy function to compute offset point
        SGGeod sg = SGGeod::fromDeg( CGAL::to_double( s.x() ), CGAL::to_double( s.y() ) );
        SGGeod eg = SGGeodesy::direct( sg, h, 0.1 );
        edgeArrPoint e = edgeArrPoint( eg.getLongitudeDeg(), eg.getLatitudeDeg() );
        
        return tgConstraint( consRay, s, e, p, desc );
    }
    
    // create a line constraint from two points
    static tgConstraint fromLine( const edgeArrPoint& s, edgeArrPoint& e, unsigned long p, const std::string& desc ) {
        return tgConstraint( consLine, s, e, p, desc );
    }

    // todo : this is stupid...
    // Whenever a new constraint is added, we need to decide to modify existing constraints, but
    // only those on the same side of the edge.
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

    // is the given point on the constraint.
    bool                        hasOn( edgeArrPoint& pt );
    // break a constraint in two at the given point.
    void                        breakAt( edgeArrPoint& pt );
    // break a constraint given another constraint.
    bool                        breakWith( const tgConstraint& other );
    // find the intersection ( if one exists ), between two constraints
    bool                        Intersect( const tgConstraint& other, edgeArrPoint& intersectionLocation );

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

    void                        toShapefile(const std::string& datasource, const std::string& layer) const;
    void                        toShapefile(void* lid) const;
    
private:
    edgeArrPoint                start;
    edgeArrPoint                end;
    edgeArrCurve                curve;
    double                      heading;
    consType_e                  type;
};

#endif /* __TG_CONSTRAINT_HXX__ */
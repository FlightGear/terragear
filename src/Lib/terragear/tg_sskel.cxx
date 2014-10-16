#include <iostream>
#include <cassert>

#include<boost/shared_ptr.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include<CGAL/Polygon_2.h>
#include<CGAL/create_straight_skeleton_2.h>

#include <simgear/debug/logstream.hxx>

#include "tg_polygon.hxx"
#include "tg_misc.hxx"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2                                          Point;
typedef CGAL::Polygon_2<K>                                  Polygon_2;
typedef CGAL::Straight_skeleton_2<K>                        Ss;
typedef boost::shared_ptr<Ss>                               SsPtr;

tgpolygon_list tgPolygon::StraightSkeleton(void)
{
    tgPolygon region;
    tgpolygon_list poly_list;
    Polygon_2 poly;
    
    // just one contour, for now - holes can be supported, though
    if ( Contours() == 1 ) {
        tgContour contour = GetContour(0);

        // verify Orientation
        if ( !contour.IsClockwise() ) {
            for (unsigned int i=0; i<contour.GetSize(); i++) {
                SGGeod node = contour.GetNode(i);
            
                poly.push_back( Point(node.getLongitudeDeg(), node.getLatitudeDeg() ) ) ;
            }

            SsPtr iss = CGAL::create_interior_straight_skeleton_2(poly);
            
            // now iterate the faces - for each generated poly, make sure the first segment
            // is the defining contour
            Ss::Face_iterator f_it;
            for ( f_it = iss->faces_begin(); f_it != iss->faces_end(); f_it++ ) {
                Ss::Face face = (*f_it);
                
                // each face is a poly
                region.Erase();
                
                Ss::Halfedge_handle he_h, done;
                he_h = face.halfedge();
                done = he_h;
                
                do { 
                    SGGeod p0 = SGGeod::fromDeg( he_h->vertex()->point().x(), he_h->vertex()->point().y() );
                    region.AddNode(0, p0);
                    
                    he_h = he_h->next(); 
                } while( he_h != done);
                
                poly_list.push_back( region );
            }
        } else {
            SG_LOG(SG_GENERAL, SG_ALERT, "SSKEL: contour must be  counter-clockwise!" );            
        }
    } else {
        SG_LOG(SG_GENERAL, SG_ALERT, "SSKEL: poly has more than 1 contour!" );                    
    }
    
    return poly_list;
}

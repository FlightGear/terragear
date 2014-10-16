#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <CGAL/assertions.h>
#include <simgear/debug/logstream.hxx>

#include "tg_arrangement.hxx"
#include "tg_shapefile.hxx"

static int add_id = 0;
void tgArrangement::Add( const tgPolygon& poly )
{
    char add_tag[32];
        
    if ( poly.Contours() == 1 ) {
        tgContour cont;
        
        for ( unsigned int i = 0; i < poly.Contours(); ++i ) {
            // make sure contour is ccw
            cont = poly.GetContour( i );                
            std::vector<arrPoint> pts;            
            pts.clear();

            for ( unsigned int j = 0; j < cont.GetSize(); ++j ) {
                pts.push_back( arrPoint( cont.GetNode(j).getLongitudeDeg(), cont.GetNode(j).getLatitudeDeg() ) );
            }
            pts.push_back( arrPoint( cont.GetNode(0).getLongitudeDeg(), cont.GetNode(0).getLatitudeDeg() ) );
            insert( arr, arrPolyline(pts.begin(), pts.end()) );
        }
                
        sprintf( add_tag, "add_%d", add_id++ );

        SG_LOG(SG_GENERAL, SG_INFO, "tgArrangment::added poly " << add_tag << " with " << poly.ContourSize(0) << " points" );
        
        tgShapefile::FromContour( cont, false, "./edge_dbg", add_tag, "poly" );
        //ToShapefiles( "./edge_dbg", add_tag );
    }
}

const double isEqual2D_Epsilon = 0.0000000001;

bool SGGeod_isReallyEqual2D( const SGGeod& g0, const SGGeod& g1 )
{
    return ( (fabs( g0.getLongitudeDeg() - g1.getLongitudeDeg() ) < isEqual2D_Epsilon) &&
             (fabs( g0.getLatitudeDeg()  - g1.getLatitudeDeg() )  < isEqual2D_Epsilon ) );
}


void tgArrangement::Add( const tgSegment& subject )
{
    std::vector<arrPoint> pts;            

    SG_LOG(SG_GENERAL, SG_INFO, "tgArrangment::add segment: start " << subject.start << " to end " << subject.end );
    
    if ( !SGGeod_isReallyEqual2D( subject.start, subject.end ) ) {
        pts.push_back( arrPoint( subject.start.getLongitudeDeg(), subject.start.getLatitudeDeg() ) );
        pts.push_back( arrPoint( subject.end.getLongitudeDeg(), subject.end.getLatitudeDeg() ) );
        
        insert( arr, arrPolyline(pts.begin(), pts.end()) );    
    } else {
        SG_LOG(SG_GENERAL, SG_INFO, "tgArrangment::add segment: start == end - ignore ");
    }
        
    SG_LOG(SG_GENERAL, SG_INFO, "tgArrangment::add segment: complete" );        
}

static int arr_id = 0;
void tgArrangement::ToShapefiles( const std::string& path, const std::string& layer_prefix )
{
    char layer[32];

    CGAL_precondition( arr.is_valid () );
        
    arr_id++;
    
    // first - write the vertex layer
    std::vector<SGGeod> vertex_list;
    typename arrArrangement::Vertex_const_iterator vit;
    for ( vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit ) {
        vertex_list.push_back( SGGeod::fromDeg( to_double(vit->point().x()), 
                                                to_double(vit->point().y())) );
    }
    
    sprintf( layer, "arr_%s_%d_vertex", layer_prefix.c_str(), arr_id );
    tgShapefile::FromGeodList( vertex_list, "./edge_dbg", layer, "vertex" );

    std::vector<tgSegment> segment_list;
    typename arrArrangement::Edge_const_iterator eit;
    
    for ( eit = arr.edges_begin(); eit != arr.edges_end(); ++eit ) {
        unsigned int numPoints = eit->curve().points();
        
        for ( unsigned int i=0; i<numPoints-1; i++ ) {
            arrSegment seg = eit->curve()[i];
            tgSegment tgseg( SGGeod::fromDeg( to_double( seg.source().x() ),
                                              to_double( seg.source().y() ) ),
                             SGGeod::fromDeg( to_double( seg.target().x() ),
                                              to_double( seg.target().y() ) ) );
            segment_list.push_back( tgseg );
        }
    }

    sprintf( layer, "arr_%s_%d_edges", layer_prefix.c_str(), arr_id );
    tgShapefile::FromSegmentList( segment_list, "./edge_dbg", layer, "vertex" );
    
    // Print the arrangement edges.
    std::cout << arr.number_of_edges() << "edges : " << std::endl;
    for ( eit = arr.edges_begin(); eit != arr.edges_end(); ++eit )
        std::cout << " [ " << eit->curve() << " ] " << std::endl;
    
#if 0
    typename arrArrangement::Edge_const_iterator eit;
    std::cout << arr.number_of_edges() << "edges : " << std::endl;
    for ( eit = arr.edges_begin(); eit != arr.edges_end(); ++eit )
        std::cout << " [ " << eit−>curve( ) << " ] " << std::endl;

    
    // Print the arrangement faces .
    typename arrArrangement::Face_const_iterator fit;
    std::cout << arr.number_of_faces() << "faces : " << std::endl;
    for ( fit = arr.faces_begin(); fit != arr.faces_end(); ++fit )
        print_ face<Arrangement>( fit );
#endif    
}
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <CGAL/assertions.h>
#include <simgear/debug/logstream.hxx>

#include "tg_arrangement.hxx"
#include "tg_shapefile.hxx"

void tgArrangement::Clear( void )
{
    arr.clear();
}

void tgArrangement::Add( const tgPolygon& poly )
{
    tgContour cont;
    std::vector<arrSegment> segs;            
    
    for ( unsigned int i = 0; i < poly.Contours(); ++i ) {
        arrPoint src, trg;
        cont = poly.GetContour( i );                
        segs.clear();
        
        src = arrPoint( cont.GetNode(0).getLongitudeDeg(), cont.GetNode(0).getLatitudeDeg() );
        for ( unsigned int j = 1; j < cont.GetSize(); ++j ) {
            trg = arrPoint( cont.GetNode(j).getLongitudeDeg(), cont.GetNode(j).getLatitudeDeg() );
            segs.push_back( arrSegment(src, trg) );
            src = trg;
        }
        trg = arrPoint( cont.GetNode(0).getLongitudeDeg(), cont.GetNode(0).getLatitudeDeg() );
        segs.push_back( arrSegment( src, trg ) );
        insert( arr, segs.begin(), segs.end() );
    }
}

const double isEqual2D_Epsilon = 0.0000000001;

bool SGGeod_isReallyEqual2D( const SGGeod& g0, const SGGeod& g1 )
{
    return ( (fabs( g0.getLongitudeDeg() - g1.getLongitudeDeg() ) < isEqual2D_Epsilon) &&
             (fabs( g0.getLatitudeDeg()  - g1.getLatitudeDeg() )  < isEqual2D_Epsilon ) );
}

void tgArrangement::Add( const tgContour& cont )
{
    std::vector<arrSegment> segs;    
    arrPoint src, trg;
        
    src = arrPoint( cont.GetNode(0).getLongitudeDeg(), cont.GetNode(0).getLatitudeDeg() );
    for ( unsigned int j = 1; j < cont.GetSize(); ++j ) {
        trg = arrPoint( cont.GetNode(j).getLongitudeDeg(), cont.GetNode(j).getLatitudeDeg() );
        segs.push_back( arrSegment(src, trg) );
        src = trg;
    }
    trg = arrPoint( cont.GetNode(0).getLongitudeDeg(), cont.GetNode(0).getLatitudeDeg() );
    segs.push_back( arrSegment( src, trg ) );
    insert( arr, segs.begin(), segs.end() );
}

void tgArrangement::Add( const tgSegment& subject )
{
    arrPoint src, trg;
    
    src = arrPoint( subject.GetGeodStart().getLongitudeDeg(), subject.GetGeodStart().getLatitudeDeg() );
    trg = arrPoint( subject.GetGeodEnd().getLongitudeDeg(), subject.GetGeodEnd().getLatitudeDeg() );
    insert( arr, arrSegment( src, trg ) );
}

#if 0
double tgContour::GetArea( void ) const
{
double area = 0.0;
SGVec2d a, b;
unsigned int i, j;

if ( node_list.size() >= 3 ) {
    j = node_list.size() - 1;
    for (i=0; i<node_list.size(); i++) {
        a = SGGeod_ToSGVec2d( node_list[i] );
        b = SGGeod_ToSGVec2d( node_list[j] );
        
        area += (b.x() + a.x()) * (b.y() - a.y());
        j=i;
        }
        } else {
            area = 0;
            }
            
            return fabs(area * 0.5);
            }
#endif

void tgArrangement::DumpPolys( void )
{
    arrArrangement::Face_const_iterator fit;
    
    for( fit = arr.faces_begin(); fit != arr.faces_end(); fit++ ) {
        arrArrangement::Face face = (*fit);
        if( face.has_outer_ccb() ) {
            arrCcbHEConstCirc ccb = face.outer_ccb();
            arrCcbHEConstCirc cur = ccb;
            arrHEConstHand    he;
            
            std::vector<tgSegment>  segList;
            double                  area = 0.0;
            SGVec2d                 a, b;
            
            segList.clear();
            do
            {
                he = cur;

                // ignore inner antenna
                if ( he->face() != he->twin()->face() ) {                    
                    //std::cout << "   [" << he->curve() << "]   " << "(" << he->target()->point() << ")";
                    SGGeod start = SGGeod::fromDeg( CGAL::to_double( he->source()->point().x() ), 
                                                    CGAL::to_double( he->source()->point().y() ) );
                    SGGeod end   = SGGeod::fromDeg( CGAL::to_double( he->target()->point().x() ), 
                                                    CGAL::to_double( he->target()->point().y() ) );
                
                    tgSegment seg( start, end );
                    segList.push_back( seg );
                    
                    a = SGGeod_ToSGVec2d( start );
                    b = SGGeod_ToSGVec2d( end );
                    
                    area += (b.x() + a.x()) * (b.y() - a.y());
                }
                
                ++cur;
            } while (cur != ccb);
            
            if ( fabs( area ) < 0.00000000001 ) {
                tgShapefile::FromSegmentList( segList, false, "./edge_dbg", "polys", "poly" );
            }
        }
    }
}

Polygon_set tgArrangement::ToPolygonSet( int contour )
{
    Polygon_set polygons;

    // first, get the number of holes in the unbounded face - this is the number of
    // Polygon_with_holes we need to start with
    arrArrangement::Face_handle fh = arr.unbounded_face();
    
    GetPolygons( fh, polygons, contour );
    
    return polygons;
}

static tgContour ToTgContour( const Polygon& p )
{
    tgContour contour;
    
    Polygon::Vertex_const_iterator vit;
    for (vit = p.vertices_begin(); vit != p.vertices_end(); ++vit) {
        SGGeod g = SGGeod::fromDeg( CGAL::to_double( vit->x()), 
                                    CGAL::to_double( vit->y()) );
        contour.AddNode( g );
    }
    
    return contour;
}

static void ToShapefile( const Polygon_with_holes& pwh, int c )
{
    char layer[256];
    
    // Add the boundary Contour
    if (!pwh.is_unbounded()) {
        tgContour cont = ToTgContour( pwh.outer_boundary() );
        sprintf( layer, "contour_%d_outer_boundary", c );
        tgShapefile::FromContour( cont, false, false, "./clip_dbg", layer, "cont" );
        
        Polygon_with_holes::Hole_const_iterator hit;
        int num_holes = 0;
        
        for (hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
            cont = ToTgContour( *hit );
            
            sprintf( layer, "contour_%d_hole_%d", c, num_holes );
            tgShapefile::FromContour( cont, false, false, "./clip_dbg", layer, "cont" );
            
            num_holes++;
        }
    } else {
        SG_LOG(SG_GENERAL, SG_INFO, "tgArrangmentPWH to Shapefile - pwh is unbounded" );
    }
}

#if 1
void tgArrangement::GetPolygons( const arrArrangement::Face_const_handle& fh, Polygon_set& polygons, int c )
{
    arrArrangement::Hole_const_iterator hit;
    static unsigned int num_contours = 0;
    
    for( hit = fh->holes_begin(); hit != fh->holes_end(); hit++ ) {
        arrCcbHEConstCirc ccbFirst = (*hit);
        arrCcbHEConstCirc ccbCur = ccbFirst;
        arrHEConstHand    heCur;        
        arrSegment        segCur;        
        Polygon           contour;
        arrArrangement::Face_const_handle face;
        
        
        // Both GetPolygons and GetHoles pass a face handle of an outer boundary.
        // the inner boundaries are always holes.
        // for GetPolygons, we need to convert the holes into boundaries. ( reverse ccb )
        heCur = ccbCur;
        face = heCur->twin()->face();
        
        do
        {
            heCur          = ccbCur;
            
            // ignore inner antenna
            if ( heCur->face() != heCur->twin()->face() ) {
                segCur = heCur->curve();                    
                contour.push_back( segCur.source() );
            } 

            ccbCur++;
            
        } while (ccbCur != ccbFirst);

        CGAL::Orientation orient = contour.orientation();
        if ( orient == CGAL::CLOCKWISE ) {
            // we generate each contour as a boundary 
            contour.reverse_orientation();
        }
        
#if 0        
        char filename[128];
        sprintf( filename, "poly_%04d", num_contours );
        std::ofstream output_file("filename");
        if (!output_file.is_open()) {
            std::cerr << "Failed to open the " << "./output_polys.txt" << std::endl;
            exit(0);
        }   
        output_file << std::setprecision(16) << contour;
        output_file.close();
#endif

        Polygon_with_holes pwh( contour );
        polygons.join( contour );
//        std::cout << "# pwhs: " << polygons.number_of_polygons_with_holes() << " valid " << polygons.is_valid() << std::endl;

        num_contours++;
    }
}
#else
void tgArrangement::GetPolygons( const arrArrangement::Face_const_handle& fh, Polygon_set& polygons, int c )
{
    arrArrangement::Hole_const_iterator hit;
    unsigned int num_contours = 0;
    char layer1[128];
    char layer2[128];
    
    for( hit = fh->holes_begin(); hit != fh->holes_end(); hit++ ) {
        arrCcbHEConstCirc ccb = (*hit);        
        arrHEConstHand    heFirst = ccb;
        arrHEConstHand    heCur   = heFirst;
        arrSegment        seg;
        
        Polygon           contour;
        arrArrangement::Face_const_handle face;
        
        
        // Both GetPolygons and GetHoles pass a face handle of an outer boundary.
        // the inner boundaries are always holes.
        // for GetPolygons, we need to convert the holes into boundaries. ( reverse ccb )        
        face = heCur->twin()->face();
        
        std::vector<SGGeod> geod_list;
        std::vector<SGGeod> geod_list2;
        
        int num_parts = 0;
        int numSegments;
        
        do
        {
            numSegments = heCur->curve().number_of_segments();
            
            // ignore inner antenna
            if ( heCur->face() != heCur->twin()->face() ) {
                // push back the segments of this curve
                for ( unsigned int i=0; i<numSegments; i++ ) {
                    seg = heCur->curve()[i];
                    
                    contour.push_back( seg.source() );
                    geod_list.push_back( SGGeod::fromDeg( CGAL::to_double( seg.source().x() ),
                                                          CGAL::to_double( seg.source().y() ) ) );
                    geod_list2.push_back( SGGeod::fromDeg( CGAL::to_double( seg.source().x() ),
                                                           CGAL::to_double( seg.source().y() ) ) );
                    
                }
                
                sprintf( layer1, "contour_%d_face_%d_part_%d", c, num_contours, num_parts );                
                sprintf( layer2, "cont_contour_%d_face_%d_part_%d", c, num_contours, num_parts );
            } 
            else 
            {
                for ( unsigned int i=0; i<numSegments; i++ ) {
                    seg = heCur->curve()[i];
                    
                    geod_list.push_back( SGGeod::fromDeg( CGAL::to_double( seg.source().x() ),
                                                          CGAL::to_double( seg.source().y() ) ) );
                    geod_list2.push_back( SGGeod::fromDeg( CGAL::to_double( seg.source().x() ),
                                                           CGAL::to_double( seg.source().y() ) ) );
                }
                                
                sprintf( layer1, "contour_%d_face_%d_part_%d_is_antenna", c, num_contours, num_parts );                
                sprintf( layer2, "cont_contour_%d_face_%d_part_%d_is_antenna", c, num_contours, num_parts );                
            }
            
            tgShapefile::FromGeodList( geod_list, true, "./clip_dbg", layer1, "cont" );
            geod_list.clear();          
            tgShapefile::FromGeodList( geod_list2, true, "./clip_dbg", layer2, "cont" );
            
            heCur++;            
            num_parts++;
            
        } while (heCur != heFirst);
        
        // make the contour a loop - use last seg target.
        contour.push_back( seg.target() );
        geod_list2.push_back( SGGeod::fromDeg( CGAL::to_double( seg.target().x() ),
                                               CGAL::to_double( seg.target().y() ) ) );
        
        sprintf( layer2, "cont_contour_%d_face_%d_complete", c, num_contours );
        tgShapefile::FromGeodList( geod_list2, true, "./clip_dbg", layer2, "cont" );
        
        // check orientation
        if ( c == 7 ) {
            SG_LOG(SG_GENERAL, SG_INFO, "tgArrangment::GetPolygons Reversing orientation" );
            contour.reverse_orientation();
        }
        
        SG_LOG(SG_GENERAL, SG_INFO, "tgArrangment::GetPolygons create PWH" );
        Polygon_with_holes pwh( contour );
        SG_LOG(SG_GENERAL, SG_INFO, "tgArrangment::GetPolygons create PWH complete" );
        
        if ( true ) {
            ToShapefile( pwh, num_contours );
            
            SG_LOG(SG_GENERAL, SG_INFO, "tgArrangment::GetPolygons insert PWH" );
            polygons.insert( pwh );
            SG_LOG(SG_GENERAL, SG_INFO, "tgArrangment::GetPolygons insert PWH complete" );
        } else {            
            SG_LOG(SG_GENERAL, SG_INFO, "tgArrangment::GetPolygons PWH invalid" );
        }
        #if 0        
        if ( !contour.is_empty() ) {
            std::list<Polygon> holes;
            
            // this was a hole, but we want outer boundary - reverse the orientation
            contour.reverse_orientation();
            
            GetHoles( face, holes, polygons );
            
            if ( !holes.empty() ) {
                polygons.insert( Polygon_with_holes( contour, holes.begin(), holes.end() ) );
    } else {
        polygons.insert( Polygon_with_holes( contour ) );
    }
    }
    #endif        
    
    num_contours++;
    }
}
#endif


void tgArrangement::GetHoles( const arrArrangement::Face_const_handle& fh, std::list<Polygon>& holes, Polygon_set& polygons )
{
    arrArrangement::Hole_const_iterator hit;
    
    for( hit = fh->holes_begin(); hit != fh->holes_end(); hit++ ) {
        arrCcbHEConstCirc ccb = (*hit);
        arrCcbHEConstCirc cur = ccb;
        arrHEConstHand    he;
        Polygon           contour;
        arrArrangement::Face_const_handle hole;
        
        // Both GetPolygons and GetHoles pass a face handle of an outer boundary.
        // the inner boundaries are always holes.
        // for GetHoles, we don't need to reverse anything
        he   = cur;
        hole = he->face();
        
        do
        {
            he = cur;

            // ignore inner antenna
            if ( he->face() != he->twin()->face() ) {                    
                // push back the segments of this curve
                arrSegment seg = he->curve();
                contour.push_back( seg.source() );
            }
           
            cur++;
        } while (cur != ccb);
        
        if ( !contour.is_empty() ) {
            // there may be polygons ( with holes ) inside this hole....
            GetPolygons( hole, polygons, 100 );
            
            holes.push_back( contour );
        }
    }
}

void tgArrangement::ToShapefiles( const std::string& path, const std::string& layer_prefix )
{
    char layer[256];

    CGAL_precondition( arr.is_valid () );
    
    // first - write the vertex layer
    std::vector<SGGeod> vertex_list;
    typename arrArrangement::Vertex_const_iterator vit;
    for ( vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit ) {
        vertex_list.push_back( SGGeod::fromDeg( CGAL::to_double(vit->point().x()), 
                                                CGAL::to_double(vit->point().y())) );
    }
    
    sprintf( layer, "arr_%s_vertex", layer_prefix.c_str() );
    tgShapefile::FromGeodList( vertex_list, false, path.c_str(), layer, "vertex" );

    std::vector<tgSegment> segment_list;
    typename arrArrangement::Edge_const_iterator eit;
    
    for ( eit = arr.edges_begin(); eit != arr.edges_end(); ++eit ) {        
        arrSegment seg = eit->curve();
        tgSegment tgseg( SGGeod::fromDeg( CGAL::to_double( seg.source().x() ),
                                          CGAL::to_double( seg.source().y() ) ),
                         SGGeod::fromDeg( CGAL::to_double( seg.target().x() ),
                                          CGAL::to_double( seg.target().y() ) ) );
        segment_list.push_back( tgseg );
    }

    sprintf( layer, "arr_%s_edges", layer_prefix.c_str() );
    tgShapefile::FromSegmentList( segment_list, true, path.c_str(), layer, "vertex" );    
}

void tgArrangement::ToShapefiles( const std::string& path, const std::string& layer_prefix, Polygon_set& polys )
{
    
}
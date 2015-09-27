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
    for ( unsigned int i = 0; i < poly.Contours(); ++i ) {
        Add( poly.GetContour(i), NULL );
    }
}

void tgArrangement::Add( const tgContour& cont, const char* layer_prefix )
{
    std::vector<tgSegment2> segs;    
    tgPoint src,  trg;

    src = cont.GetPoint(0);
    for ( unsigned int j = 1; j < cont.GetSize(); ++j ) {
        trg = cont.GetPoint(j);    
        if ( src != trg ) {
            segs.push_back( tgSegment2(src, trg) );
        }
        src  = trg;
    }
    trg = cont.GetPoint(0);    
    if ( src != trg ) {
        segs.push_back( tgSegment2( src, trg ) );
    }
    
    insert( arr, segs.begin(), segs.end() );
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

void tgArrangement::SaveFace( const arrArrangement::Face_const_handle& fh, const char* path, const char* layer )
{
    if ( fh->has_outer_ccb() ) {
        SaveCCB( fh->outer_ccb(), path, layer );
    } else {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgArrangement::SaveFace face has no outer ccb " );
    }
}

void tgArrangement::SaveCCB( const arrArrangement::Ccb_halfedge_const_circulator& ccb, const char* path, const char* layer )
{
    arrCcbHEConstCirc cur = ccb;
    arrHEConstHand    he;
        
    tgContour cont;
    cont.AddPoint( cur->source()->point() );
        
    do {
        he = cur;
        cont.AddPoint( he->target()->point() );
            
        ++cur;
    } while (cur != ccb);
        
    tgShapefile::FromContour( cont, true, false, path, layer, "poly" );
}

#if 1
void tgArrangement::GetPolygons( const arrArrangement::Face_const_handle& fh, Polygon_set& polygons, int c )
{
    arrArrangement::Hole_const_iterator hit;
    static unsigned int num_contours = 0;
    
    SG_LOG(SG_GENERAL, SG_ALERT, "tgArrangement::GetPolygons face has " << fh->number_of_holes() << " holes" );

    for( hit = fh->holes_begin(); hit != fh->holes_end(); hit++ ) {
        arrCcbHEConstCirc ccbFirst = (*hit);
        arrCcbHEConstCirc ccbCur = ccbFirst;
        arrHEConstHand    heCur;        
        arrSegment        segCur;        
        Polygon           contour;
        arrArrangement::Face_const_handle face;
    
        // dump the current hole
        //char layer[128];
        //sprintf( layer, "poly_%d_hole%d", c, num_contours );
        //SaveCCB( *hit, "./GetPolys", layer );
        
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
            } else {
                SG_LOG(SG_GENERAL, SG_ALERT, "tgArrangement::GetPolygons ignoring hw " );
            }                

            ccbCur++;
            
        } while (ccbCur != ccbFirst);

        CGAL::Orientation orient = contour.orientation();
        if ( orient == CGAL::CLOCKWISE ) {
            // we generate each contour as a boundary 
            contour.reverse_orientation();
        }
    
        Polygon_with_holes pwh( contour );

        //SG_LOG(SG_GENERAL, SG_ALERT, "tgArrangement::GetPolygons joining contour " );
        polygons.join( contour );
        //SG_LOG(SG_GENERAL, SG_ALERT, "tgArrangement::GetPolygons done " );
        
        //SG_LOG(SG_GENERAL, SG_ALERT, "# pwhs: " << polygons.number_of_polygons_with_holes() << " valid " << polygons.is_valid() );
        
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
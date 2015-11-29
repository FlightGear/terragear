#include <iostream>
#include <fstream>
#include <sstream>

#include <CGAL/Bbox_2.h>

#include <simgear/debug/logstream.hxx>

#include "tg_polygon_def.hxx"
#include "tg_polygon_accumulator.hxx"

#if 0
tgPolygon tgAccumulator::Diff( const tgContour& subject )
{
    tgPolygon  result;
    UniqueSGGeodSet all_nodes;

    /* before diff - gather all nodes */
    for ( unsigned int i = 0; i < subject.GetSize(); ++i ) {
        all_nodes.add( subject.GetNode(i) );
    }

    for ( unsigned int i = 0; i < nodes.size(); i++ ) {
        all_nodes.add( nodes[i] );
    }

    unsigned int  num_hits = 0;
    tgRectangle box1 = subject.GetBoundingBox();

    ClipperLib::Path  clipper_subject = tgContour::ToClipper( subject );
    ClipperLib::Paths clipper_result;

    ClipperLib::Clipper c;
    c.Clear();

    c.AddPath(clipper_subject, ClipperLib::ptSubject, true);

    // clip result against all polygons in the accum that intersect our bb
    for (unsigned int i=0; i < accum.size(); i++) {
        tgRectangle box2 = BoundingBox_FromClipper( accum[i] );

        if ( box2.intersects(box1) )
        {
            c.AddPaths(accum[i], ClipperLib::ptClip, true);
            num_hits++;
        }
    }

    if (num_hits) {
        if ( !c.Execute(ClipperLib::ctDifference, clipper_result, ClipperLib::pftNonZero, ClipperLib::pftNonZero) ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "Diff With Accumulator returned FALSE" );
            //exit(-1);
        }
        result = tgPolygon::FromClipper( clipper_result );
        result = tgPolygon::AddColinearNodes( result, all_nodes );
    } else {
        result.AddContour( subject );
    }

    return result;
}

tgPolygon tgAccumulator::Diff( const tgPolygon& subject )
{
    tgPolygon result;
    UniqueSGGeodSet all_nodes;

    /* before diff - gather all nodes */
    for ( unsigned int i = 0; i < subject.Contours(); ++i ) {
        for ( unsigned int j = 0; j < subject.ContourSize( i ); ++j ) {
            all_nodes.add( subject.GetNode(i, j) );
        }
    }

    for ( unsigned int i = 0; i < nodes.size(); i++ ) {
        all_nodes.add( nodes[i] );
    }

    unsigned int  num_hits = 0;
    tgRectangle box1 = subject.GetBoundingBox();

    ClipperLib::Paths clipper_subject = tgPolygon::ToClipper( subject );
    ClipperLib::Paths clipper_result;

    ClipperLib::Clipper c;
    c.Clear();

    c.AddPaths(clipper_subject, ClipperLib::ptSubject, true);

    // clip result against all polygons in the accum that intersect our bb
    for (unsigned int i=0; i < accum.size(); i++) {
        tgRectangle box2 = BoundingBox_FromClipper( accum[i] );

        if ( box2.intersects(box1) )
        {
            c.AddPaths(accum[i], ClipperLib::ptClip, true);
            num_hits++;
        }
    }

    if (num_hits) {
        if ( !c.Execute(ClipperLib::ctDifference, clipper_result, ClipperLib::pftNonZero, ClipperLib::pftNonZero) ) {
            SG_LOG(SG_GENERAL, SG_ALERT, "Diff With Accumulator returned FALSE" );
            //exit(-1);
        }

        result = tgPolygon::FromClipper( clipper_result );
        result = tgPolygon::AddColinearNodes( result, all_nodes );

        // Make sure we keep texturing info
        result.SetMaterial( subject.GetMaterial() );
        result.SetTexParams( subject.GetTexParams() );
        result.SetId( subject.GetId() );        
        result.va_int_mask = subject.va_int_mask;
        result.va_flt_mask = subject.va_flt_mask;
        result.int_vas = subject.int_vas;
        result.flt_vas = subject.flt_vas;
        
    } else {
        result = subject;
    }

    return result;
}

static tgContour ToTgContour( const Polygon& p, bool isHole )
{
    tgContour contour;
    
    Polygon::Vertex_const_iterator vit;
    for (vit = p.vertices_begin(); vit != p.vertices_end(); ++vit) {
        contour.AddPoint( *vit );
    }
    
    // remove antenna
    // contour.RemoveAntenna();
    contour.SetHole( isHole );
    
    return contour;
}

static void cgPolygonToSegList( const Polygon& p, std::vector<tgSegment2>& segs )
{
    Polygon::Vertex_const_iterator          src, trg;

    src = p.vertices_begin();
    trg = src; trg++;
    while( trg != p.vertices_end() ) {
        segs.push_back( tgSegment2(*src++, *trg++) );
    }
    trg = p.vertices_begin();
    segs.push_back( tgSegment2(*src, *trg) );    
}

static void FindIntersections( const Polygon_with_holes& pwh, const tgLine2& line, std::vector<tgPoint>& intersections )
{    
    // find the intersection of all segments and sorth them from bottom to top.
    Polygon                                 p  = pwh.outer_boundary();
    Polygon_with_holes::Hole_const_iterator hit;
    std::vector<tgSegment2>                 segs;

    cgPolygonToSegList( p, segs );
    for (hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
        cgPolygonToSegList( *hit, segs );
    }
    
    for ( unsigned int i=0; i<segs.size(); i++ ) {
        CGAL::Object result = CGAL::intersection(line, segs[i]);
        if (const tgPoint *ipoint = CGAL::object_cast<tgPoint>(&result)) {
            intersections.push_back( *ipoint );
        }
    }
    std::sort( intersections.begin(), intersections.end() );    
}

bool myfunction (boost::tuple<tgKernel::RT, tgKernel::RT> i, boost::tuple<tgKernel::RT, tgKernel::RT> j ) 
{ 
    // sort from largest to smallest
    return (  boost::get<0>(i) > boost::get<0>(j) ); 
}

static tgPoint GetInteriorPoint( const Polygon_with_holes& pwh )
{
    std::vector<tgKernel::RT>                               xcoords;
    std::vector< boost::tuple<tgKernel::RT, tgKernel::RT> > xbest;
    tgPoint      max_pos;
    
    // find the largest delta in x
    Polygon      p  = pwh.outer_boundary();
    CGAL::Bbox_2 bb = p.bbox();

    Polygon_with_holes::Hole_const_iterator hit;
    Polygon::Vertex_const_iterator          vit;
    for (vit = p.vertices_begin(); vit != p.vertices_end(); ++vit) {
        xcoords.push_back( vit->x() );
    }
    for (hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
        for (vit = hit->vertices_begin(); vit != hit->vertices_end(); ++vit) {
            xcoords.push_back( vit->x() );
        }
    }
    std::sort( xcoords.begin(), xcoords.end() );

    for (unsigned int i=0; i<xcoords.size()-1; i++) {
        tgKernel::RT delta = xcoords[i+1]-xcoords[i];
        xbest.push_back( boost::make_tuple( delta, xcoords[i]+delta/2 ) );
    }
    std::sort( xbest.begin(), xbest.end(), myfunction );
    
    // create a vertical line at the midpoint of the largest delta
    for ( unsigned int i=0; i<xbest.size(); i++ ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "GetInteriorPoint: i " << i << " width " << xbest[i].get<0>() << " location " << xbest[i].get<1>() );       
    }
    
#if 1
    for ( unsigned int i=0; i<xbest.size(); i++ ) {
        tgLine2 line( tgPoint( xbest[i].get<1>(), bb.ymin() ), tgPoint(xbest[i].get<1>(), bb.ymax()) );
        // get and sort the intersections with all segments of the pwh and this line
        
        std::vector<tgPoint> intersections;
        FindIntersections( pwh, line, intersections );
        // from 0-1 IN face, 1-2 OUT of face, 2-3 IN face, etccc.
        // we want the biggest delta between 0,1 2,3 4,5, etc, and the midpoint of the biggest.
        
        tgKernel::RT max_delta = 0.0;
        for ( unsigned int i=0; i<intersections.size(); i+=2 ) {
            if ( intersections[i+1].y() - intersections[i].y() > max_delta ) {
                max_delta = intersections[i+1].y()-intersections[i].y();
                max_pos   = tgPoint( intersections[i].x(), intersections[i].y()+max_delta/2 );
            }
        }
        
        if ( max_delta > 0.000001 ) {
            break;
        }
    }
#endif

    return max_pos;
}

static void ToTgPolygon( const Polygon_set& ps, tgPolygon& poly )
{
    tgcontour_list          contours;
    std::vector<tgPoint>    interiorPoints;

    std::list<Polygon_with_holes> pwh_list;
    std::list<Polygon_with_holes>::const_iterator it;

    ps.polygons_with_holes( std::back_inserter(pwh_list) );

    SG_LOG(SG_GENERAL, SG_ALERT, "ToTgPolygon : got " << pwh_list.size() << " polys with holes ");
    for (it = pwh_list.begin(); it != pwh_list.end(); ++it) {
        Polygon_with_holes pwh = (*it);

        // Add the boundary Contour
        if (!pwh.is_unbounded()) {
            interiorPoints.push_back( GetInteriorPoint( pwh ) );
            
            tgContour cont = ToTgContour( pwh.outer_boundary(), false );
            contours.push_back( cont );
            
            Polygon_with_holes::Hole_const_iterator hit;
            for (hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
                cont = ToTgContour( *hit, true );
                contours.push_back( cont );
            }
        } else {
            SG_LOG(SG_GENERAL, SG_ALERT, "ToTgPolygon : pwh is unbounded!");
        }
    }

    poly.SetInteriorPoints( interiorPoints );
    poly.SetContours( contours );
}

void ToShapefile( const Polygon_set& ps, const char* layer )
{
    tgPolygon poly;
    ToTgPolygon( ps, poly );
    
    SG_LOG(SG_GENERAL, SG_ALERT, "ToShapefile : got " << poly.Contours() << " contours" );
    
    tgShapefile::FromPolygon( poly, true, false, "./clip_dbg", layer, "poly" );
}

static CGAL::Bbox_2 GetBoundingBox( const Polygon_set& subject ) 
{
    std::list<Polygon_with_holes> pwh_list;    
    subject.polygons_with_holes( std::back_inserter(pwh_list) );
    
    return CGAL::bbox_2( pwh_list.begin(), pwh_list.end() );
}

#if 0
static Polygon_set ToCgalPolyWithHoles( const tgPolygon& subject )
{
    tgArrangement   arr;
    Polygon_set     boundaries;
    Polygon_set     holes;
    
    // for each boundary contour, we add it to its own arrangement
    // then read the resulting faces as a list of polygons with holes
    // note that a self intersecting face ( like a donut ) will generate
    // more than one face.  We need to determine for each face wether it is a 
    // hole or not.
    // ___________
    // | ______  |
    // | \    /  |
    // |  \  /   |
    // |___\/____|
    //
    // Example of a single self intersecting contour that should be represented by a polygon 
    
    // first, we need to get all of the contours in reletively simple format
    SG_LOG(SG_GENERAL, SG_ALERT, "ToCgalPolyWithHoles : Inserting " << subject.Contours() << " contours "  );

    for (unsigned int i=0; i<subject.Contours(); i++ ) {
        char layer[128];

        //sprintf( layer, "%04u_original_contour_%d", subject.GetId(), i );
        //tgShapefile::FromContour( subject.GetContour(i), false, true, "./clip_dbg", layer, "cont" );
        
        arr.Clear();
        arr.Add( subject.GetContour(i) );

        // retreive the new Contour(s) from traversing the outermost face first
        // any holes in this face are individual polygons
        // any holes in those faces are holes, etc...
        
        // dump the arrangement to see what we have.
        //sprintf( layer, "%04u_Arrangement_contour_%d", subject.GetId(), i );
        //arr.ToShapefiles( "./clip_dbg", layer );

        // Combine boundaries and holes into their sets
        Polygon_set face = arr.ToPolygonSet( i );
        //sprintf( layer, "%04u_face_contour_%d", subject.GetId(), i );
        //ToShapefile( face, layer );
        
        if ( subject.GetContour(i).GetHole() ) {
            //SG_LOG(SG_GENERAL, SG_ALERT, "ToCgalPolyWithHoles : Join with holes"  );
            
            //SG_LOG(SG_GENERAL, SG_ALERT, "ToCgalPolyWithHoles : before - face_valid " << face.is_valid() << " holes_valid " << holes.is_valid()  );
            holes.join( face );
            //SG_LOG(SG_GENERAL, SG_ALERT, "ToCgalPolyWithHoles : after - face_valid " << face.is_valid() << " holes_valid " << holes.is_valid()  );
        } else {
            //SG_LOG(SG_GENERAL, SG_ALERT, "ToCgalPolyWithHoles : Join with boundaries"  );
            
            //SG_LOG(SG_GENERAL, SG_ALERT, "ToCgalPolyWithHoles : before - face_valid " << face.is_valid() << " boundaries_valid " << boundaries.is_valid()  );
            boundaries.join( face );            
            //SG_LOG(SG_GENERAL, SG_ALERT, "ToCgalPolyWithHoles : after - face_valid " << face.is_valid() << " boundaries_valid " << boundaries.is_valid()  );
        }
        //SG_LOG(SG_GENERAL, SG_ALERT, "ToCgalPolyWithHoles : Join complete"  );
        
    }
    
    // now, generate the result
    boundaries.difference( holes );
    
    // dump to shapefile
    
    return boundaries;
}
#else
static bool ToCgalPolyWithHoles( const tgPolygon& subject, Polygon_set& cgSubject, CGAL::Bbox_2& bb )
{
    tgArrangement   arr;
    Polygon_set     boundaries;
    Polygon_set     holes;
    
    // for each boundary contour, we add it to its own arrangement
    // then read the resulting faces as a list of polygons with holes
    // note that a self intersecting face ( like a donut ) will generate
    // more than one face.  We need to determine for each face wether it is a 
    // hole or not.
    // ___________
    // | ______  |
    // | \    /  |
    // |  \  /   |
    // |___\/____|
    //
    // Example of a single self intersecting contour that should be represented by a polygon 
    
    // first, we need to get all of the contours in reletively simple format
    SG_LOG(SG_GENERAL, SG_ALERT, "ToCgalPolyWithHoles : Inserting " << subject.Contours() << " contours "  );

    for (unsigned int i=0; i<subject.Contours(); i++ ) {
        char layer[128];

        //sprintf( layer, "%04u_original_contour_%d", subject.GetId(), i );
        //tgShapefile::FromContour( subject.GetContour(i), false, true, "./clip_dbg", layer, "cont" );
    
        arr.Clear();
        arr.Add( subject.GetContour(i), layer );
    
        // retreive the new Contour(s) from traversing the outermost face first
        // any holes in this face are individual polygons
        // any holes in those faces are holes, etc...
        
        // dump the arrangement to see what we have.
        //sprintf( layer, "%04u_Arrangement_contour_%d", subject.GetId(), i );
        //arr.ToShapefiles( "./clip_dbg", layer );

        // Combine boundaries and holes into their sets
        Polygon_set face = arr.ToPolygonSet( i );
        //sprintf( layer, "%04u_face_contour_%d", subject.GetId(), i );
        //ToShapefile( face, layer );
        
        if ( subject.GetContour(i).GetHole() ) {
            //SG_LOG(SG_GENERAL, SG_ALERT, "ToCgalPolyWithHoles : Join with holes"  );
            
            //SG_LOG(SG_GENERAL, SG_ALERT, "ToCgalPolyWithHoles : before - face_valid " << face.is_valid() << " holes_valid " << holes.is_valid()  );
            holes.join( face );
            //SG_LOG(SG_GENERAL, SG_ALERT, "ToCgalPolyWithHoles : after - face_valid " << face.is_valid() << " holes_valid " << holes.is_valid()  );
        } else {
            //SG_LOG(SG_GENERAL, SG_ALERT, "ToCgalPolyWithHoles : Join with boundaries"  );
            
            //SG_LOG(SG_GENERAL, SG_ALERT, "ToCgalPolyWithHoles : before - face_valid " << face.is_valid() << " boundaries_valid " << boundaries.is_valid()  );
            boundaries.join( face );            
            //SG_LOG(SG_GENERAL, SG_ALERT, "ToCgalPolyWithHoles : after - face_valid " << face.is_valid() << " boundaries_valid " << boundaries.is_valid()  );
        }
        //SG_LOG(SG_GENERAL, SG_ALERT, "ToCgalPolyWithHoles : Join complete"  );
    }
    
    // now, generate the result
    boundaries.difference( holes );
    
    // dump to shapefile
    if ( boundaries.is_valid() ) {
        //SG_LOG(SG_GENERAL, SG_ALERT, "ToCgalPolyWithHoles : Boundary valid - get bounding box"  );
        
        cgSubject = boundaries;
        bb = GetBoundingBox( cgSubject );

        //SG_LOG(SG_GENERAL, SG_ALERT, "ToCgalPolyWithHoles : Boundary valid - bb is " << bb  );
        
        return true;
    } else {
        return false;
    }
}
#endif

typedef CGAL::Bbox_2    BBox;


void tgAccumulator::Diff_cgal( tgPolygon& subject )
{   
    // static int savepoly = 0;
    // char filename[32];
    
    Polygon_set  cgalSubject;
    CGAL::Bbox_2 cgalBbox;
    
    Polygon_set diff = accum_cgal;

    if ( ToCgalPolyWithHoles( subject, cgalSubject, cgalBbox ) ) {
        if ( !accumEmpty ) {
            cgalSubject.difference( diff );
        }

        ToTgPolygon( cgalSubject, subject );
    }
}

void tgAccumulator::Add_cgal( const tgPolygon& subject )
{
    // just add the converted PolygonWithHoles to the Polygon set
    Polygon_set  cgalSubject;
    CGAL::Bbox_2 cgalBbox;
    
    if ( ToCgalPolyWithHoles( subject, cgalSubject, cgalBbox ) ) {
        accum_cgal.join(cgalSubject);
        accumEmpty = false;
    }
}

#endif

// rewrite to use bounding boxes, and lists of polygons with holes 
// need a few functions:
// 1) generate a Polygon_set from the Polygons_with_holes in the list that intersect subject bounding box
// 2) Add to the Polygons_with_holes list with a Polygon set ( and the bounding boxes )
    
cgalPoly_PolygonSet tgAccumulator::GetAccumPolygonSet( const CGAL::Bbox_2& bbox ) 
{
    std::list<tgAccumEntry>::const_iterator it;
    std::list<cgalPoly_PolygonWithHoles> accum;
    cgalPoly_PolygonSet ps;
    
    // traverse all of the Polygon_with_holes and accumulate their union
    for ( it=accum_cgal_list.begin(); it!=accum_cgal_list.end(); it++ ) {
        if ( CGAL::do_overlap( bbox, (*it).bbox ) ) {
            accum.push_back( (*it).pwh );
        }
    }
    
    ps.join( accum.begin(), accum.end() );
    
    return ps;
}

void tgAccumulator::AddAccumPolygonSet( const cgalPoly_PolygonSet& ps )
{
    std::list<cgalPoly_PolygonWithHoles> pwh_list;
    std::list<cgalPoly_PolygonWithHoles>::const_iterator it;
    CGAL::Bbox_2 bbox;
    
    ps.polygons_with_holes( std::back_inserter(pwh_list) );
    for (it = pwh_list.begin(); it != pwh_list.end(); ++it) {
        tgAccumEntry entry;
        entry.pwh  = (*it);
        entry.bbox =  entry.pwh.outer_boundary().bbox();

        accum_cgal_list.push_back( entry );
    }    
}

void tgAccumulator::add( const tgPolygonSet& ps )
{
    AddAccumPolygonSet( ps.getPs() );
}

#define DEBUG_DIFF_AND_ADD 0
void tgAccumulator::Diff_and_Add_cgal( tgPolygonSet& subject )
{
#if DEBUG_DIFF_AND_ADD    
    char            layer[128];
#endif
    
    cgalPoly_PolygonSet subPs  = subject.getPs();
    cgalPoly_PolygonSet difPs = GetAccumPolygonSet( subject.getBoundingBox() );

#if DEBUG_DIFF_AND_ADD    
    sprintf( layer, "clip_%03ld_pre_subject", subject.getId() );
    toShapefile( add, layer );
        
    tgContour bb;
    bb.AddPoint( tgPoint( cgBoundingBox.xmin(), cgBoundingBox.ymin() ) );
    bb.AddPoint( tgPoint( cgBoundingBox.xmin(), cgBoundingBox.ymax() ) );
    bb.AddPoint( tgPoint( cgBoundingBox.xmax(), cgBoundingBox.ymax() ) );
    bb.AddPoint( tgPoint( cgBoundingBox.xmax(), cgBoundingBox.ymin() ) );
        
    sprintf( layer, "clip_%03ld_bbox", subject.getId() );
    tgShapefile::FromContour( bb, false, false, "./clip_dbg", layer, "bbox" );
        
    sprintf( layer, "clip_%03ld_pre_accum", subject.getId() );
    ToShapefile( diff, layer );
#endif

    if ( difPs.number_of_polygons_with_holes() ) {
        subPs.difference( difPs );
            
#if DEBUG_DIFF_AND_ADD    
        sprintf( layer, "clip_%03ld_post_subject", subject.getId() );
            ToShapefile( cgSubject, layer );            
#endif

        subject.setPs( subPs );            
    }

    // add the polygons_with_holes to the accumulator list
    AddAccumPolygonSet( subPs );
}

void tgAccumulator::toShapefile( const char* ds, const char* layer )
{
    CGAL::Bbox_2 bbox;
    tgPolygonSet all = GetAccumPolygonSet( bbox );
    
    all.toShapefile( ds, layer );    
}

#if 0
void tgAccumulator::Add( const tgContour& subject )
{
    tgPolygon poly;

    // Add the nodes
    for ( unsigned int i = 0; i < subject.GetSize(); ++i ) {
        nodes.add( subject.GetNode(i) );
    }

    poly.AddContour( subject );

    ClipperLib::Paths clipper_subject = tgPolygon::ToClipper( poly );
    accum.push_back( clipper_subject );
}

void tgAccumulator::Add( const tgPolygon& subject )
{
    if ( subject.Contours() ) {
        ClipperLib::Paths clipper_subject = tgPolygon::ToClipper( subject );
        
        for ( unsigned int i = 0; i < subject.Contours(); ++i ) {
            for ( unsigned int j = 0; j < subject.ContourSize( i ); ++j ) {
                nodes.add( subject.GetNode(i, j) );
            }
        }

        accum.push_back( clipper_subject );
    } else {
        SG_LOG(SG_GENERAL, SG_ALERT, "tgAccumulator::Add() - Adding poly with " << subject.Contours() << " contours " );
    }
}

tgPolygon tgAccumulator::Union()
{
    tgPolygon result;
    UniqueSGGeodSet all_nodes;
    
    /* before diff - gather all nodes */    
    for ( unsigned int i = 0; i < nodes.size(); i++ ) {
        all_nodes.add( nodes[i] );
    }
    
    ClipperLib::Paths clipper_result;
    
    ClipperLib::Clipper c;
    c.Clear();
    
    for (unsigned int i=0; i < accum.size(); i++) {
        c.AddPaths(accum[i], ClipperLib::ptSubject, true);
    }
    
    if ( !c.Execute(ClipperLib::ctUnion, clipper_result, ClipperLib::pftNonZero, ClipperLib::pftNonZero) ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "Union Accumulator returned FALSE" );
        //exit(-1);
    }
        
    result = tgPolygon::FromClipper( clipper_result );
    result = tgPolygon::AddColinearNodes( result, all_nodes );
        
    return result;
}
#endif

#if 0
void tgAccumulator::ToShapefiles( const std::string& path, const std::string& layer_prefix, bool individual )
{
    char shapefile[32];
    char layer[32];

    if ( accum.size() ) {
        if ( individual ) {
            for (unsigned int i=0; i < accum.size(); i++) {
                sprintf( layer, "%s_%d", layer_prefix.c_str(), i );
                sprintf( shapefile, "accum_%d", i );
                tgShapefile::FromClipper( accum[i], true, path, layer, std::string(shapefile) );
            }
        } else {
            ClipperLib::Paths clipper_result;
            ClipperLib::Clipper  c;
            c.Clear();

            for ( unsigned int i=0; i<accum.size(); i++ ) {
                c.AddPaths(accum[i], ClipperLib::ptSubject, true);
            }
        
            if ( c.Execute( ClipperLib::ctUnion, clipper_result, ClipperLib::pftNonZero, ClipperLib::pftNonZero) ) {
                tgShapefile::FromClipper( clipper_result, true, path, layer_prefix, "accum" );
            } else {
                SG_LOG(SG_GENERAL, SG_ALERT, "Clipper Failure in tgAccumulator::ToShapefiles()" );
            }
        }
    }
}
#else
//void tgAccumulator::ToShapefiles( const std::string& path, const std::string& layer_prefix, bool individual )
//{
//    ToShapefile( accum_cgal, layer_prefix.c_str() );
//}
#endif

#if 0
void tgAccumulator::ToClipperfiles( const std::string& path, const std::string& layer_prefix, bool individual )
{
    std::ofstream file;
    char filename[256];
    
    if ( accum.size() ) {
        if ( individual ) {
            for (unsigned int i=0; i < accum.size(); i++) {
                sprintf( filename, "%s/%s_%d", path.c_str(), layer_prefix.c_str(), i );
                                
                file.open (filename);
                file << accum[i];
                file.close();
            }
        } else {
            ClipperLib::Paths clipper_result;
            ClipperLib::Clipper  c;
            c.Clear();
            
            for ( unsigned int i=0; i<accum.size(); i++ ) {
                c.AddPaths(accum[i], ClipperLib::ptSubject, true);
            }
            
            if ( c.Execute( ClipperLib::ctUnion, clipper_result, ClipperLib::pftNonZero, ClipperLib::pftNonZero) ) {
                sprintf( filename, "%s/%s", path.c_str(), layer_prefix.c_str() );
                
                file.open (filename);
                file << clipper_result;
                file.close();
            } else {
                SG_LOG(SG_GENERAL, SG_ALERT, "Clipper Failure in tgAccumulator::ToClipperFiles()" );
            }
        }
    }
}
#endif
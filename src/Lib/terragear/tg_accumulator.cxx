#include <iostream>
#include <fstream>
#include <sstream>

#include <CGAL/Bbox_2.h>

#include <simgear/debug/logstream.hxx>

#include "tg_accumulator.hxx"
#include "tg_shapefile.hxx"
#include "tg_misc.hxx"

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
        SGGeod g = SGGeod::fromDeg( CGAL::to_double( vit->x()), 
                                    CGAL::to_double( vit->y()) );
        contour.AddNode( g );
    }
    
    // remove antenna
    contour.RemoveAntenna();
    contour.SetHole( isHole );
    
    return contour;
}
  
static tgcontour_list ToTgPolygon( const Polygon_set& ps )
{
    tgcontour_list contours;

    std::list<Polygon_with_holes> pwh_list;
    std::list<Polygon_with_holes>::const_iterator it;

    ps.polygons_with_holes( std::back_inserter(pwh_list) );
    for (it = pwh_list.begin(); it != pwh_list.end(); ++it) {
        Polygon_with_holes pwh = (*it);

        // Add the boundary Contour
        if (!pwh.is_unbounded()) {
            tgContour cont = ToTgContour( pwh.outer_boundary(), false );
            contours.push_back( cont );
            
            Polygon_with_holes::Hole_const_iterator hit;
            for (hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
                cont = ToTgContour( *hit, true );
                contours.push_back( cont );
            }
        }
    }
    
    return contours;
}

void ToShapefile( const Polygon_set& ps, const char* layer )
{
    tgcontour_list contours;

    contours = ToTgPolygon( ps );
    tgShapefile::FromContourList( contours, false, false, "./clip_dbg", layer, "poly" );
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
    // SG_LOG(SG_GENERAL, SG_ALERT, "ToCgalPolyWithHoles : Inserting " << subject.Contours() << " contours "  );

    for (unsigned int i=0; i<subject.Contours(); i++ ) {
        // char layer[128];

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
    if ( boundaries.is_valid() ) {
        cgSubject = boundaries;
        bb = GetBoundingBox( cgSubject );
        
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

        //sprintf( layer, "%04u_cgal_subject_after_diff", subject.GetId() );
        //ToShapefile( cgalSubject, layer );
    
        tgcontour_list contours;
        contours = ToTgPolygon( cgalSubject );
    
        subject.SetContours( contours );
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

// rewrite to use bounding boxes, and lists of polygons with holes 
// need a few functions:
// 1) generate a Polygon_set from the Polygons_with_holes in the list that intersect subject bounding box
// 2) Add to the Polygons_with_holes list with a Polygon set ( and the bounding boxes )
    
Polygon_set tgAccumulator::GetAccumPolygonSet( const CGAL::Bbox_2& bbox ) 
{
    std::list<tgAccumEntry>::const_iterator it;
    std::list<Polygon_with_holes> accum;
    Polygon_set ps;
    
    // traverse all of the Polygon_with_holes and accumulate their union
    for ( it=accum_cgal_list.begin(); it!=accum_cgal_list.end(); it++ ) {
        if ( CGAL::do_overlap( bbox, (*it).bbox ) ) {
            accum.push_back( (*it).pwh );
        }
    }
    
    ps.join( accum.begin(), accum.end() );
    
    return ps;
}

void tgAccumulator::AddAccumPolygonSet( const Polygon_set& ps )
{
    std::list<Polygon_with_holes> pwh_list;
    std::list<Polygon_with_holes>::const_iterator it;
    CGAL::Bbox_2 bbox;
    
    ps.polygons_with_holes( std::back_inserter(pwh_list) );
    for (it = pwh_list.begin(); it != pwh_list.end(); ++it) {
        tgAccumEntry entry;
        entry.pwh  = (*it);
        entry.bbox =  entry.pwh.outer_boundary().bbox();

        accum_cgal_list.push_back( entry );
    }    
}

void tgAccumulator::Diff_and_Add_cgal( tgPolygon& subject )
{
    Polygon_set     cgSubject;
    CGAL::Bbox_2    cgBoundingBox;

#if 0    
    char            layer[128];
#endif
    
    if ( ToCgalPolyWithHoles( subject, cgSubject, cgBoundingBox ) ) {
        Polygon_set add  = cgSubject;
        Polygon_set diff = GetAccumPolygonSet( cgBoundingBox );

#if 0        
        sprintf( layer, "clip_%03d_pre_subject", subject.GetId() );
        ToShapefile( add, layer );
        
        tgContour bb;
        bb.AddNode( SGGeod::fromDeg( cgBoundingBox.xmin(), cgBoundingBox.ymin() ) );
        bb.AddNode( SGGeod::fromDeg( cgBoundingBox.xmin(), cgBoundingBox.ymax() ) );
        bb.AddNode( SGGeod::fromDeg( cgBoundingBox.xmax(), cgBoundingBox.ymax() ) );
        bb.AddNode( SGGeod::fromDeg( cgBoundingBox.xmax(), cgBoundingBox.ymin() ) );
        
        sprintf( layer, "clip_%03d_bbox", subject.GetId() );
        tgShapefile::FromContour( bb, false, false, "./clip_dbg", layer, "bbox" );
        
        sprintf( layer, "clip_%03d_pre_accum", subject.GetId() );
        ToShapefile( diff, layer );
#endif

        if ( diff.number_of_polygons_with_holes() ) {
            cgSubject.difference( diff );
            
#if 0
            sprintf( layer, "clip_%03d_post_subject", subject.GetId() );
            ToShapefile( cgSubject, layer );            
#endif

        }

        // add the polygons_with_holes to the accumulator list
        AddAccumPolygonSet( add );
        
        tgcontour_list contours = ToTgPolygon( cgSubject );
        subject.SetContours( contours );
    } else {
        tgcontour_list contours;
        contours.clear();
        subject.SetContours( contours );
    }    
}

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
void tgAccumulator::ToShapefiles( const std::string& path, const std::string& layer_prefix, bool individual )
{
    ToShapefile( accum_cgal, layer_prefix.c_str() );
}
#endif

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
#include <iostream>
#include <fstream>

#include <simgear/debug/logstream.hxx>

#include "tg_polygon.hxx"
#include "tg_arrangement.hxx"

#define USE_CGAL    (1)

//#if USE_CGAL

static CGAL::Bbox_2 GetBoundingBox( const Polygon_set& subject ) 
{
    std::list<Polygon_with_holes> pwh_list;    
    subject.polygons_with_holes( std::back_inserter(pwh_list) );
    
    return CGAL::bbox_2( pwh_list.begin(), pwh_list.end() );
}

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
    if ( boundaries.is_valid() ) {
        cgSubject = boundaries;
        bb = GetBoundingBox( cgSubject );
        
        return true;
    } else {
        return false;
    }
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


tgPolygon tgPolygon::Union_cgal( const tgPolygon& subject, tgPolygon& clip )
{
    Polygon_set  cgalSubject;
    Polygon_set  cgalClip;
    CGAL::Bbox_2 cgalBbox;
    tgPolygon    result;
    
    // add clip to subject
    ToCgalPolyWithHoles( subject, cgalSubject, cgalBbox );
    ToCgalPolyWithHoles( clip, cgalClip, cgalBbox );
    
    return result;
}

tgPolygon tgPolygon::Union_cgal( const tgpolygon_list& polys )
{
    Polygon_set                     cgSubject;
    CGAL::Bbox_2                    cgBbox;
    std::list<Polygon_with_holes>   accum;
    Polygon_set                     cgResult;
    tgPolygon                       result;

    SG_LOG(SG_GENERAL, SG_ALERT, "Union_cgal : have " << polys.size() << " polys" );
    
    for (unsigned int i=0; i<polys.size(); i++) {    
        if ( ToCgalPolyWithHoles( polys[i], cgSubject, cgBbox ) ) {
            // add all PWHs to accum
            std::list<Polygon_with_holes> pwh_list;
            cgSubject.polygons_with_holes( std::back_inserter(pwh_list) );
    
            accum.insert( accum.end(), pwh_list.begin(), pwh_list.end() );
        }
    }
    
    SG_LOG(SG_GENERAL, SG_ALERT, "Union_cgal : have " << accum.size() << " poly_with_holes " );

    // perform the join
    cgResult.join( accum.begin(), accum.end() );
    tgcontour_list contours = ToTgPolygon( cgResult );
    
    SG_LOG(SG_GENERAL, SG_ALERT, "Union_cgal : have " << contours.size() << " contours " );

    result.SetContours( contours );

    return result;
}

static bool clipper_dump = false;
void tgPolygon::SetClipperDump( bool dmp )
{
    clipper_dump = dmp;
}

tgPolygon tgPolygon::Union( const tgPolygon& subject, tgPolygon& clip )
{
    tgPolygon       result;
    UniqueSGGeodSet all_nodes;
    std::ofstream   dmpfile;

    /* before union - gather all nodes */
    for ( unsigned int i = 0; i < subject.Contours(); ++i ) {
        for ( unsigned int j = 0; j < subject.ContourSize( i ); ++j ) {
            all_nodes.add( subject.GetNode(i, j) );
        }
    }

    for ( unsigned int i = 0; i < clip.Contours(); ++i ) {
        for ( unsigned int j = 0; j < clip.ContourSize( i ); ++j ) {
            all_nodes.add( clip.GetNode(i, j) );
        }
    }

    ClipperLib::Paths clipper_subject = tgPolygon::ToClipper( subject );
    ClipperLib::Paths clipper_clip    = tgPolygon::ToClipper( clip );
    ClipperLib::Paths clipper_result;

    if ( clipper_dump ) {
        dmpfile.open ("subject.txt");
        dmpfile << clipper_subject;
        dmpfile.close();

        dmpfile.open ("clip.txt");
        dmpfile << clipper_clip;
        dmpfile.close();
    }

    ClipperLib::Clipper c;
    c.Clear();
    c.AddPaths(clipper_subject, ClipperLib::ptSubject, true);
    c.AddPaths(clipper_clip, ClipperLib::ptClip, true);
    c.Execute(ClipperLib::ctUnion, clipper_result, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    if ( clipper_dump ) {
        dmpfile.open ("result.txt");
        dmpfile << clipper_result;
        dmpfile.close();
    }

    result = tgPolygon::FromClipper( clipper_result );
    result = tgPolygon::AddColinearNodes( result, all_nodes );

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );
    result.SetId( subject.GetId() );
    result.SetPreserve3D( subject.GetPreserve3D() );
    result.va_int_mask = subject.va_int_mask;
    result.va_flt_mask = subject.va_flt_mask;
    result.int_vas = subject.int_vas;
    result.flt_vas = subject.flt_vas;
    
    return result;
}

tgPolygon tgPolygon::Union( const tgpolygon_list& polys )
{
    ClipperLib::Paths clipper_result;
    ClipperLib::Clipper c;
    UniqueSGGeodSet all_nodes;
    tgPolygon  result;

    /* before union - gather all nodes */
    for ( unsigned int i=0; i<polys.size(); i++ ) {
        for ( unsigned int j = 0; j < polys[i].Contours(); ++j ) {
            for ( unsigned int k = 0; k < polys[i].ContourSize( j ); ++k ) {
                all_nodes.add( polys[i].GetNode(j, k) );
            }
        }
    }

    c.Clear();
    for (unsigned int i=0; i<polys.size(); i++) {
        ClipperLib::Paths clipper_clip = tgPolygon::ToClipper( polys[i] );
        c.AddPaths(clipper_clip, ClipperLib::ptSubject, true);
    }
    c.Execute(ClipperLib::ctUnion, clipper_result, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

    result = tgPolygon::FromClipper( clipper_result );
    result = tgPolygon::AddColinearNodes( result, all_nodes );

    return result;
}

tgPolygon tgPolygon::Diff( const tgPolygon& subject, tgPolygon& clip )
{
    tgPolygon result;
    UniqueSGGeodSet all_nodes;

    /* before diff - gather all nodes */
    for ( unsigned int i = 0; i < subject.Contours(); ++i ) {
        for ( unsigned int j = 0; j < subject.ContourSize( i ); ++j ) {
            all_nodes.add( subject.GetNode(i, j) );
        }
    }

    for ( unsigned int i = 0; i < clip.Contours(); ++i ) {
        for ( unsigned int j = 0; j < clip.ContourSize( i ); ++j ) {
            all_nodes.add( clip.GetNode(i, j) );
        }
    }

    ClipperLib::Paths clipper_subject = tgPolygon::ToClipper( subject );
    ClipperLib::Paths clipper_clip    = tgPolygon::ToClipper( clip );
    ClipperLib::Paths clipper_result;

    ClipperLib::Clipper c;
    c.Clear();
    c.AddPaths(clipper_subject, ClipperLib::ptSubject, true);
    c.AddPaths(clipper_clip, ClipperLib::ptClip, true);
    c.Execute(ClipperLib::ctDifference, clipper_result, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    result = tgPolygon::FromClipper( clipper_result );
    result = tgPolygon::AddColinearNodes( result, all_nodes );

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );
    result.SetId( subject.GetId() );
    result.SetPreserve3D( subject.GetPreserve3D() );
    result.va_int_mask = subject.va_int_mask;
    result.va_flt_mask = subject.va_flt_mask;
    result.int_vas = subject.int_vas;
    result.flt_vas = subject.flt_vas;
    
    return result;
}

tgPolygon tgPolygon::Intersect( const tgPolygon& subject, const tgPolygon& clip )
{
    tgPolygon result;
    UniqueSGGeodSet all_nodes;

    /* before intersect - gather all nodes */
    for ( unsigned int i = 0; i < subject.Contours(); ++i ) {
        for ( unsigned int j = 0; j < subject.ContourSize( i ); ++j ) {
            all_nodes.add( subject.GetNode(i, j) );
        }
    }

    for ( unsigned int i = 0; i < clip.Contours(); ++i ) {
        for ( unsigned int j = 0; j < clip.ContourSize( i ); ++j ) {
            all_nodes.add( clip.GetNode(i, j) );
        }
    }

    ClipperLib::Paths clipper_subject = tgPolygon::ToClipper( subject );
    ClipperLib::Paths clipper_clip    = tgPolygon::ToClipper( clip );
    ClipperLib::Paths clipper_result;

    ClipperLib::Clipper c;
    c.Clear();
    c.AddPaths(clipper_subject, ClipperLib::ptSubject, true);
    c.AddPaths(clipper_clip, ClipperLib::ptClip, true);
    c.Execute(ClipperLib::ctIntersection, clipper_result, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    result = tgPolygon::FromClipper( clipper_result );
    result = tgPolygon::AddColinearNodes( result, all_nodes );

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );
    result.SetId( subject.GetId() );
    result.SetPreserve3D( subject.GetPreserve3D() );
    result.va_int_mask = subject.va_int_mask;
    result.va_flt_mask = subject.va_flt_mask;
    result.int_vas = subject.int_vas;
    result.flt_vas = subject.flt_vas;
    
    return result;
}

ClipperLib::Paths tgPolygon::ToClipper( const tgPolygon& subject )
{
    ClipperLib::Paths result;

    for ( unsigned int i=0; i<subject.Contours(); i++ ) {
        result.push_back( tgContour::ToClipper( subject.GetContour(i) ) );
    }

    return result;
}

tgPolygon tgPolygon::FromClipper( const ClipperLib::Paths& subject )
{
    tgPolygon result;

    // for each polygon, we need to check the orientation, to set the hole flag...
    for ( unsigned int i=0; i<subject.size(); i++)
    {
        result.AddContour( tgContour::FromClipper( subject[i] ) );
    }

    return result;
}
#include <iostream>
#include <fstream>

#include <simgear/debug/logstream.hxx>

#include "tg_polygon.hxx"
#include "tg_misc.hxx"

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

ClipperLib::Path tgTriangle::ToClipper( const tgTriangle& subject )
{
    ClipperLib::Path  contour;

    for ( unsigned int i=0; i<3; i++)
    {
        SGGeod p = subject.GetNode( i );
        contour.push_back( SGGeod_ToClipper(p) );
    }

    // boundaries need to be orientation: true
    if ( !Orientation( contour ) ) {
        //SG_LOG(SG_GENERAL, SG_INFO, "Building clipper contour - boundary contour needs to be reversed" );
        ReversePath( contour );
    }

    return contour;
}

tgPolygon tgTriangle::Intersect( const tgTriangle& subject, const tgTriangle& clip )
{
    tgPolygon result;
    UniqueSGGeodSet all_nodes;

    /* before diff - gather all nodes */
    for ( unsigned int i = 0; i < 3; ++i ) {
        all_nodes.add( subject.GetNode(i) );
    }

    for ( unsigned int i = 0; i < 3; ++i ) {
        all_nodes.add( clip.GetNode(i) );
    }

    ClipperLib::Path  clipper_subject = tgTriangle::ToClipper( subject );
    ClipperLib::Path  clipper_clip    = tgTriangle::ToClipper( clip );
    ClipperLib::Paths clipper_result;

    ClipperLib::Clipper c;
    c.Clear();
    c.AddPath(clipper_subject, ClipperLib::ptSubject, true);
    c.AddPath(clipper_clip, ClipperLib::ptClip, true);
    c.Execute(ClipperLib::ctIntersection, clipper_result, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    result = tgPolygon::FromClipper( clipper_result );
    result = tgPolygon::AddColinearNodes( result, all_nodes );

    return result;
}

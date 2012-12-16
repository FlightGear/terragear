#include <iostream>
#include <fstream>

#include <simgear/debug/logstream.hxx>

#include "tg_polygon.hxx"

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

    ClipperLib::Polygons clipper_subject = tgPolygon::ToClipper( subject );
    ClipperLib::Polygons clipper_clip    = tgPolygon::ToClipper( clip );
    ClipperLib::Polygons clipper_result;

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
    c.AddPolygons(clipper_subject, ClipperLib::ptSubject);
    c.AddPolygons(clipper_clip, ClipperLib::ptClip);
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

    return result;
}

tgPolygon tgPolygon::Union( const tgpolygon_list& polys )
{
    ClipperLib::Polygons clipper_result;
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
        ClipperLib::Polygons clipper_clip = tgPolygon::ToClipper( polys[i] );
        c.AddPolygons(clipper_clip, ClipperLib::ptSubject);
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

    ClipperLib::Polygons clipper_subject = tgPolygon::ToClipper( subject );
    ClipperLib::Polygons clipper_clip    = tgPolygon::ToClipper( clip );
    ClipperLib::Polygons clipper_result;

    ClipperLib::Clipper c;
    c.Clear();
    c.AddPolygons(clipper_subject, ClipperLib::ptSubject);
    c.AddPolygons(clipper_clip, ClipperLib::ptClip);
    c.Execute(ClipperLib::ctDifference, clipper_result, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    result = tgPolygon::FromClipper( clipper_result );
    result = tgPolygon::AddColinearNodes( result, all_nodes );

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );

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

    ClipperLib::Polygons clipper_subject = tgPolygon::ToClipper( subject );
    ClipperLib::Polygons clipper_clip    = tgPolygon::ToClipper( clip );
    ClipperLib::Polygons clipper_result;

    ClipperLib::Clipper c;
    c.Clear();
    c.AddPolygons(clipper_subject, ClipperLib::ptSubject);
    c.AddPolygons(clipper_clip, ClipperLib::ptClip);
    c.Execute(ClipperLib::ctIntersection, clipper_result, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    result = tgPolygon::FromClipper( clipper_result );
    result = tgPolygon::AddColinearNodes( result, all_nodes );

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );

    return result;
}

ClipperLib::Polygons tgPolygon::ToClipper( const tgPolygon& subject )
{
    ClipperLib::Polygons result;

    for ( unsigned int i=0; i<subject.Contours(); i++ ) {
        result.push_back( tgContour::ToClipper( subject.GetContour(i) ) );
    }

    return result;
}

tgPolygon tgPolygon::FromClipper( const ClipperLib::Polygons& subject )
{
    tgPolygon result;

    // for each polygon, we need to check the orientation, to set the hole flag...
    for ( unsigned int i=0; i<subject.size(); i++)
    {
        result.AddContour( tgContour::FromClipper( subject[i] ) );
    }

    return result;
}

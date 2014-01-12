#include <iostream>
#include <fstream>
#include <sstream>

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
            exit(-1);
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
            exit(-1);
        }

        result = tgPolygon::FromClipper( clipper_result );
        result = tgPolygon::AddColinearNodes( result, all_nodes );

        // Make sure we keep texturing info
        result.SetMaterial( subject.GetMaterial() );
        result.SetTexParams( subject.GetTexParams() );
    } else {
        result = subject;
    }

    return result;
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

void tgAccumulator::ToShapefiles( const std::string& path, const std::string& layer_prefix, bool individual )
{
    char shapefile[32];
    char layer[32];

    if ( accum.size() ) {
        if ( individual ) {
            for (unsigned int i=0; i < accum.size(); i++) {
                sprintf( layer, "%s_%d", layer_prefix.c_str(), i );
                sprintf( shapefile, "accum_%d", i );
                tgShapefile::FromClipper( accum[i], path, layer, std::string(shapefile) );
            }
        } else {
            ClipperLib::Paths clipper_result;
            ClipperLib::Clipper  c;
            c.Clear();

            for ( unsigned int i=0; i<accum.size(); i++ ) {
                c.AddPaths(accum[i], ClipperLib::ptSubject, true);
            }
        
            if ( c.Execute( ClipperLib::ctUnion, clipper_result, ClipperLib::pftNonZero, ClipperLib::pftNonZero) ) {
                tgShapefile::FromClipper( clipper_result, path, layer_prefix, "accum" );
            } else {
                SG_LOG(SG_GENERAL, SG_ALERT, "Clipper Failure in tgAccumulator::ToShapefiles()" );
            }
        }
    }
}

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
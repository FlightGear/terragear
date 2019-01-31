#include <iostream>
#include <fstream>
#include <sstream>
#include <climits>

#include <simgear/debug/logstream.hxx>

#include "tg_accumulator.hxx"
#include "tg_shapefile.hxx"
#include "tg_misc.hxx"

tgPolygon tgAccumulator::Diff( const tgContour& subject )
{
    tgPolygon  result;
    UniqueSGGeodSet all_nodes;
    bool done = false;
    unsigned int max_hits = UINT_MAX;

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

    while ( !done && max_hits > 0 ) {
        ClipperLib::Clipper c;
        c.Clear();

        c.AddPath(clipper_subject, ClipperLib::PolyType::Subject, true);

        // clip result against all polygons in the accum that intersect our bb
        for (unsigned int i=0; i < accum.size(); i++) {
            tgRectangle box2 = BoundingBox_FromClipper( accum[i] );

            if ( box2.intersects(box1) )
            {
                if ( num_hits < max_hits ) {
                    c.AddPaths(accum[i], ClipperLib::PolyType::Clip, true);
                    num_hits++;
                }
            }
        }

        if (num_hits) {
            if ( !c.Execute(ClipperLib::ClipType::Difference, clipper_result, ClipperLib::PolyFillType::NonZero, ClipperLib::PolyFillType::NonZero) ) {
                SG_LOG(SG_GENERAL, SG_ALERT, "Diff With Accumulator returned FALSE - reducing accumulator" );
                max_hits = num_hits-1;

                FILE* fp = fopen( "./accumulator_fail.log", "a" );
                fprintf( fp, "%s : reduce from %u to %u\n", debugstr.c_str(), num_hits, max_hits );
                fclose(fp);
            } else {
                result = tgPolygon::FromClipper( clipper_result );
                result = tgPolygon::AddColinearNodes( result, all_nodes );
                done = true;
            }
        } else {
            result.AddContour( subject );
            done = true;
        }
    }

    return result;
}

tgPolygon tgAccumulator::Diff( const tgPolygon& subject )
{
    tgPolygon result;
    UniqueSGGeodSet all_nodes;
    bool done = false;
    unsigned int max_hits = UINT_MAX;

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

    while ( !done && max_hits > 0 ) {    
        ClipperLib::Clipper c;
        c.Clear();

        c.AddPaths(clipper_subject, ClipperLib::PolyType::Subject, true);

        // clip result against all polygons in the accum that intersect our bb
        for (unsigned int i=0; i < accum.size(); i++) {
            tgRectangle box2 = BoundingBox_FromClipper( accum[i] );

            if ( box2.intersects(box1) )
            {
                c.AddPaths(accum[i], ClipperLib::PolyType::Clip, true);
                num_hits++;
            }
        }

        if (num_hits) {
            if ( !c.Execute(ClipperLib::ClipType::Difference, clipper_result, ClipperLib::PolyFillType::NonZero, ClipperLib::PolyFillType::NonZero) ) {
                SG_LOG(SG_GENERAL, SG_ALERT, "Diff With Accumulator returned FALSE - reducing accumulator" );
                max_hits = num_hits-1;

                FILE* fp = fopen( "./accumulator_fail.log", "a" );
                fprintf( fp, "%s : reduce from %u to %u\n", debugstr.c_str(), num_hits, max_hits );
                fclose(fp);                
            } else {
                result = tgPolygon::FromClipper( clipper_result );
                result = tgPolygon::AddColinearNodes( result, all_nodes );

                // Make sure we keep texturing info
                result.SetMaterial( subject.GetMaterial() );
                result.SetTexParams( subject.GetTexParams() );
                done = true;
            }
        } else {
            result = subject;
            done = true;
        }
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
    if ( accum.size() ) {
        if ( individual ) {
            for (unsigned int i=0; i < accum.size(); i++) {
                char layer[32];
                sprintf( layer, "%s_%u", layer_prefix.c_str(), i );
                
                char shapefile[32];
                sprintf( shapefile, "accum_%u", i );
                tgShapefile::FromClipper( accum[i], path, layer, std::string(shapefile) );
            }
        } else {
            ClipperLib::Paths clipper_result;
            ClipperLib::Clipper  c;
            c.Clear();

            for ( unsigned int i=0; i<accum.size(); i++ ) {
                c.AddPaths(accum[i], ClipperLib::PolyType::Subject, true);
            }
        
            if ( c.Execute( ClipperLib::ClipType::Union, clipper_result, ClipperLib::PolyFillType::NonZero, ClipperLib::PolyFillType::NonZero) ) {
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
    
    if ( accum.size() ) {
        if ( individual ) {
            char filename[256];
            for (unsigned int i=0; i < accum.size(); i++) {
                sprintf( filename, "%s/%s_%u", path.c_str(), layer_prefix.c_str(), i );

                file.open (filename);
                file << accum[i];
                file.close();
            }
        } else {
            ClipperLib::Paths clipper_result;
            ClipperLib::Clipper  c;
            c.Clear();
            
            for ( unsigned int i=0; i<accum.size(); i++ ) {
                c.AddPaths(accum[i], ClipperLib::PolyType::Subject, true);
            }
            
            if ( c.Execute( ClipperLib::ClipType::Union, clipper_result, ClipperLib::PolyFillType::NonZero, ClipperLib::PolyFillType::NonZero) ) {
                char filename[256];
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
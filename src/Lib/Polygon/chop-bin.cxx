// chop-bin.hxx -- routine to chop a polygon up along tile boundaries and
//                 write the individual pieces to the TG working polygon
//                 file format.
//
// Written by Curtis Olson, started February 1999.
//
// Copyright (C) 1999-2004  Curtis L. Olson  - http://www.flightgear.org/~curt
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// $Id: chop-bin.cxx,v 1.6 2007-11-05 21:58:59 curt Exp $


#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>

#include <string>
#include <iostream>

#include <simgear/bucket/newbucket.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/structure/exception.hxx>
#include <simgear/misc/sg_path.hxx>

#include <Output/output.hxx>

#include "index.hxx"
#include "simple_clip.hxx"
#include "chop.hxx"

using std::string;

static void clip_and_write_poly( string root, long int p_index,
                                 const string &poly_type,
                                 SGBucket b, const TGPolygon& shape,
                                 bool preserve3d )
{
    Point3D c, min, max, p;

    c = Point3D( b.get_center_lon(), b.get_center_lat(), 0 );
    double span = b.get_width();
    TGPolygon base, result;
    char tile_name[256], poly_index[256];

    // calculate bucket dimensions
    if ( (c.y() >= -89.0) && (c.y() < 89.0) ) {
        min.setx( c.x() - span / 2.0 );
        max.setx( c.x() + span / 2.0 );
        min.sety( c.y() - SG_HALF_BUCKET_SPAN );
        max.sety( c.y() + SG_HALF_BUCKET_SPAN );
    } else if ( c.y() < -89.0) {
        min.setx( -90.0 );
        max.setx( -89.0 );
        min.sety( -180.0 );
        max.sety( 180.0 );
    } else if ( c.y() >= 89.0) {
        min.setx( 89.0 );
        max.setx( 90.0 );
        min.sety( -180.0 );
        max.sety( 180.0 );
    } else
        SG_LOG( SG_GENERAL, SG_ALERT,
                "Out of range latitude in clip_and_write_poly() = " << c.y() );

    min.setz( 0.0 );
    max.setz( 0.0 );

    SG_LOG( SG_GENERAL, SG_DEBUG, "  (" << min << ") (" << max << ")" );

    // set up clipping tile
    base.add_node( 0, Point3D(min.x(), min.y(), 0) );
    base.add_node( 0, Point3D(max.x(), min.y(), 0) );
    base.add_node( 0, Point3D(max.x(), max.y(), 0) );
    base.add_node( 0, Point3D(min.x(), max.y(), 0) );

    SG_LOG(SG_GENERAL, SG_DEBUG, "shape contours = " << shape.contours() );
    for ( int ii = 0; ii < shape.contours(); ii++ )
        SG_LOG(SG_GENERAL, SG_DEBUG, "   hole = " << shape.get_hole_flag(ii) );

    result = tgPolygonInt( base, shape );

    // write_polygon(shape, "shape");
    // write_polygon(result, "result");

    SG_LOG(SG_GENERAL, SG_DEBUG, "result contours = " << result.contours() );
    for ( int ii = 0; ii < result.contours(); ii++ ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "  hole = " << result.get_hole_flag(ii) );

    }

    if ( preserve3d )
        result.inherit_elevations( shape );

    if ( result.contours() > 0 ) {
        long int t_index = b.gen_index();
        string path = root + "/" + b.gen_base_path();

        SGPath sgp( path );
        sgp.append( "dummy" );
        sgp.create_dir( 0755 );

        sprintf( tile_name, "%ld", t_index );
        string polyfile = path + "/" + tile_name;

        sprintf( poly_index, "%ld", p_index );
        polyfile += ".";
        polyfile += poly_index;


        FILE *rfp = fopen( polyfile.c_str(), "w" );
        if ( preserve3d )
            fprintf( rfp, "#3D\n" );
        else
            fprintf( rfp, "#2D\n" );
        fprintf( rfp, "%s\n", poly_type.c_str() );

        fprintf( rfp, "%d\n", result.contours() );
        for ( int i = 0; i < result.contours(); ++i ) {
            fprintf( rfp, "%d\n", result.contour_size(i) );
            fprintf( rfp, "%d\n", result.get_hole_flag(i) );
            for ( int j = 0; j < result.contour_size(i); ++j ) {
                p = result.get_pt( i, j );
                if ( preserve3d )
                    fprintf( rfp, "%.15f  %.15f %.15f\n", p.x(), p.y(), p.z() );
                else
                    fprintf( rfp, "%.15f  %.15f\n", p.x(), p.y() );
            }
        }
        fclose( rfp );
    }
}

static void clip_and_write_polys_with_mask( string root, long int p_index,
                                            const string &poly_type,
                                            SGBucket b, const poly_list& segments,
                                            bool preserve3d )
{
    Point3D c, min, max, p;

    c = Point3D( b.get_center_lon(), b.get_center_lat(), 0 );
    double span = b.get_width();
    TGPolygon base, result;
    char tile_name[256], poly_index[256];

    // calculate bucket dimensions
    if ( (c.y() >= -89.0) && (c.y() < 89.0) ) {
        min.setx( c.x() - span / 2.0 );
        max.setx( c.x() + span / 2.0 );
        min.sety( c.y() - SG_HALF_BUCKET_SPAN );
        max.sety( c.y() + SG_HALF_BUCKET_SPAN );
    } else if ( c.y() < -89.0) {
        min.setx( -90.0 );
        max.setx( -89.0 );
        min.sety( -180.0 );
        max.sety( 180.0 );
    } else if ( c.y() >= 89.0) {
        min.setx( 89.0 );
        max.setx( 90.0 );
        min.sety( -180.0 );
        max.sety( 180.0 );
    } else {
        SG_LOG( SG_GENERAL, SG_ALERT,
                "Out of range latitude in clip_and_write_poly() = " << c.y() );
    }

    min.setz( 0.0 );
    max.setz( 0.0 );

    SG_LOG( SG_GENERAL, SG_DEBUG, "  (" << min << ") (" << max << ")" );

    // set up clipping tile
    base.add_node( 0, Point3D(min.x(), min.y(), 0) );
    base.add_node( 0, Point3D(max.x(), min.y(), 0) );
    base.add_node( 0, Point3D(max.x(), max.y(), 0) );
    base.add_node( 0, Point3D(min.x(), max.y(), 0) );

    poly_list    clipped_shapes;
    TGPolygon    shape;
    unsigned int s;

    for ( s = 0; s < segments.size(); s++ ) {
        shape = segments[s];

        // SG_LOG(SG_GENERAL, SG_DEBUG, "shape contours = " << shape.contours() );
        // for ( int ii = 0; ii < shape.contours(); ii++ )
        //    SG_LOG(SG_GENERAL, SG_DEBUG, "   hole = " << shape.get_hole_flag(ii) );

        result = tgPolygonInt( base, shape );

        // write_polygon(shape, "shape");
        // write_polygon(result, "result");

        // SG_LOG(SG_GENERAL, SG_DEBUG, "result contours = " << result.contours() );
        // for ( int ii = 0; ii < result.contours(); ii++ ) {
        //     SG_LOG(SG_GENERAL, SG_DEBUG, "  hole = " << result.get_hole_flag(ii) );
        // }

        if ( preserve3d )
            result.inherit_elevations( shape );

        if ( result.contours() > 0 ) {
            clipped_shapes.push_back( result );
        }
    }

    // Now we can write the file
    long int t_index = b.gen_index();
    string path = root + "/" + b.gen_base_path();

    SGPath sgp( path );
    sgp.append( "dummy" );
    sgp.create_dir( 0755 );

    sprintf( tile_name, "%ld", t_index );
    string polyfile = path + "/" + tile_name;

    sprintf( poly_index, "%ld", p_index );
    polyfile += ".";
    polyfile += poly_index;

    FILE *rfp = fopen( polyfile.c_str(), "w" );

    if ( preserve3d )
        fprintf( rfp, "#3D_WITH_MASK\n" );
    else
        fprintf( rfp, "#2D_WITH_MASK\n" );
    fprintf( rfp, "%s\n", poly_type.c_str() );

    fprintf( rfp, "%d\n", (int)clipped_shapes.size() );
    
    for (s=0; s<clipped_shapes.size(); s++) {
        result = clipped_shapes[s];
    
        fprintf( rfp, "%d\n", result.contours() );
        for ( int i = 0; i < result.contours(); ++i ) {
            fprintf( rfp, "%d\n", result.contour_size(i) );
            fprintf( rfp, "%d\n", result.get_hole_flag(i) );
            for ( int j = 0; j < result.contour_size(i); ++j ) {
                p = result.get_pt( i, j );
                if ( preserve3d )
                    fprintf( rfp, "%.15f  %.15f %.15f\n", p.x(), p.y(), p.z() );
                else
                    fprintf( rfp, "%.15f  %.15f\n", p.x(), p.y() );
            }
        }
    }

    fclose( rfp );
}

static void clip_and_write_polys_with_tps( string root, long int p_index,
                                           const string &poly_type,
                                           SGBucket b,
                                           const poly_list& segments,
                                           const texparams_list& tps,
                                           bool preserve3d )
{
    Point3D c, min, max, p;

    c = Point3D( b.get_center_lon(), b.get_center_lat(), 0 );
    double span = b.get_width();
    TGPolygon base, result;
    char tile_name[256], poly_index[256];

    // calculate bucket dimensions
    if ( (c.y() >= -89.0) && (c.y() < 89.0) ) {
        min.setx( c.x() - span / 2.0 );
        max.setx( c.x() + span / 2.0 );
        min.sety( c.y() - SG_HALF_BUCKET_SPAN );
        max.sety( c.y() + SG_HALF_BUCKET_SPAN );
    } else if ( c.y() < -89.0) {
        min.setx( -90.0 );
        max.setx( -89.0 );
        min.sety( -180.0 );
        max.sety( 180.0 );
    } else if ( c.y() >= 89.0) {
        min.setx( 89.0 );
        max.setx( 90.0 );
        min.sety( -180.0 );
        max.sety( 180.0 );
    } else {
        SG_LOG( SG_GENERAL, SG_ALERT,
                "Out of range latitude in clip_and_write_poly() = " << c.y() );
    }

    min.setz( 0.0 );
    max.setz( 0.0 );

    SG_LOG( SG_GENERAL, SG_DEBUG, "  (" << min << ") (" << max << ")" );

    // set up clipping tile
    base.add_node( 0, Point3D(min.x(), min.y(), 0) );
    base.add_node( 0, Point3D(max.x(), min.y(), 0) );
    base.add_node( 0, Point3D(max.x(), max.y(), 0) );
    base.add_node( 0, Point3D(min.x(), max.y(), 0) );

    poly_list       clipped_shapes;
    texparams_list  clipped_tps;
    TGPolygon       shape;
    TGTexParams     tp;
    unsigned int    s;

    for ( s = 0; s < segments.size(); s++ ) {
        shape = segments[s];

        // SG_LOG(SG_GENERAL, SG_DEBUG, "shape contours = " << shape.contours() );
        // for ( int ii = 0; ii < shape.contours(); ii++ )
        //    SG_LOG(SG_GENERAL, SG_DEBUG, "   hole = " << shape.get_hole_flag(ii) );

        result = tgPolygonInt( base, shape );

        // write_polygon(shape, "shape");
        // write_polygon(result, "result");

        // SG_LOG(SG_GENERAL, SG_DEBUG, "result contours = " << result.contours() );
        // for ( int ii = 0; ii < result.contours(); ii++ ) {
            //     SG_LOG(SG_GENERAL, SG_DEBUG, "  hole = " << result.get_hole_flag(ii) );
        // }

        if ( preserve3d )
            result.inherit_elevations( shape );

        if ( result.contours() > 0 ) {
            clipped_shapes.push_back( result );
            clipped_tps.push_back( tps[s] );
        }
    }

    // Now we can write the file
    long int t_index = b.gen_index();
    string path = root + "/" + b.gen_base_path();

    SGPath sgp( path );
    sgp.append( "dummy" );
    sgp.create_dir( 0755 );

    sprintf( tile_name, "%ld", t_index );
    string polyfile = path + "/" + tile_name;

    sprintf( poly_index, "%ld", p_index );
    polyfile += ".";
    polyfile += poly_index;

    FILE *rfp = fopen( polyfile.c_str(), "w" );

    if ( preserve3d )
        fprintf( rfp, "#3D_WITH_TPS\n" );
    else
        fprintf( rfp, "#2D_WITH_TPS\n" );
    fprintf( rfp, "%s\n", poly_type.c_str() );

    fprintf( rfp, "%d\n", (int)clipped_shapes.size() );

    for (s=0; s<clipped_shapes.size(); s++) {
        shape = clipped_shapes[s];
        tp    = clipped_tps[s];

        fprintf( rfp, "%.15f  %.15f  %.15f  %.15f  %.15f  %.15f  %.15f  %.15f  %.15f\n",
                 tp.get_ref().x(), tp.get_ref().y(), tp.get_width(), tp.get_length(),
                 tp.get_heading(),
                 tp.get_minu(), tp.get_maxu(), tp.get_minu(), tp.get_maxu());
        
        fprintf( rfp, "%d\n", shape.contours() );
        for ( int i = 0; i < shape.contours(); ++i ) {
            fprintf( rfp, "%d\n", shape.contour_size(i) );
            fprintf( rfp, "%d\n", shape.get_hole_flag(i) );
            for ( int j = 0; j < shape.contour_size(i); ++j ) {
                p = shape.get_pt( i, j );
                if ( preserve3d )
                    fprintf( rfp, "%.15f  %.15f %.15f\n", p.x(), p.y(), p.z() );
                else
                    fprintf( rfp, "%.15f  %.15f\n", p.x(), p.y() );
            }
        }
        fprintf( rfp, "\n");
    }

    fclose( rfp );
}

static void clip_and_write_poly_tp( string root, long int p_index,
                                    const string &poly_type,
                                    SGBucket b, const TGPolygon& shape,
                                    const TGTexParams& tp, bool preserve3d )
{
    Point3D c, min, max, p;

    c = Point3D( b.get_center_lon(), b.get_center_lat(), 0 );
    double span = b.get_width();
    TGPolygon base, result;
    char tile_name[256], poly_index[256];

    // calculate bucket dimensions
    if ( (c.y() >= -89.0) && (c.y() < 89.0) ) {
        min.setx( c.x() - span / 2.0 );
        max.setx( c.x() + span / 2.0 );
        min.sety( c.y() - SG_HALF_BUCKET_SPAN );
        max.sety( c.y() + SG_HALF_BUCKET_SPAN );
    } else if ( c.y() < -89.0) {
        min.setx( -90.0 );
        max.setx( -89.0 );
        min.sety( -180.0 );
        max.sety( 180.0 );
    } else if ( c.y() >= 89.0) {
        min.setx( 89.0 );
        max.setx( 90.0 );
        min.sety( -180.0 );
        max.sety( 180.0 );
    } else
        SG_LOG( SG_GENERAL, SG_ALERT,  "Out of range latitude in clip_and_write_poly() = " << c.y() );

    min.setz( 0.0 );
    max.setz( 0.0 );

    SG_LOG( SG_GENERAL, SG_DEBUG, "  (" << min << ") (" << max << ")" );

    // set up clipping tile
    base.add_node( 0, Point3D(min.x(), min.y(), 0) );
    base.add_node( 0, Point3D(max.x(), min.y(), 0) );
    base.add_node( 0, Point3D(max.x(), max.y(), 0) );
    base.add_node( 0, Point3D(min.x(), max.y(), 0) );

    SG_LOG(SG_GENERAL, SG_DEBUG, "shape contours = " << shape.contours() );
    for ( int ii = 0; ii < shape.contours(); ii++ )
        SG_LOG(SG_GENERAL, SG_DEBUG, "   hole = " << shape.get_hole_flag(ii) );

    result = tgPolygonInt( base, shape );

    SG_LOG(SG_GENERAL, SG_DEBUG, "result contours = " << result.contours() );
    for ( int ii = 0; ii < result.contours(); ii++ ) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "  hole = " << result.get_hole_flag(ii) );

    }

    if ( preserve3d )
        result.inherit_elevations( shape );

    if ( result.contours() > 0 ) {
        long int t_index = b.gen_index();
        string path = root + "/" + b.gen_base_path();

        SGPath sgp( path );
        sgp.append( "dummy" );
        sgp.create_dir( 0755 );

        sprintf( tile_name, "%ld", t_index );
        string polyfile = path + "/" + tile_name;

        sprintf( poly_index, "%ld", p_index );
        polyfile += ".";
        polyfile += poly_index;


        FILE *rfp = fopen( polyfile.c_str(), "w" );
        if ( preserve3d )
            fprintf( rfp, "#3D_TP\n" );
        else
            fprintf( rfp, "#2D_TP\n" );
        fprintf( rfp, "%s\n", poly_type.c_str() );

        fprintf( rfp, "%.15f  %.15f  %.15f  %.15f  %.15f  %.15f  %.15f  %.15f  %.15f\n",
                 tp.get_ref().x(), tp.get_ref().y(), tp.get_width(), tp.get_length(),
                 tp.get_heading(),
                 tp.get_minu(), tp.get_maxu(), tp.get_minu(), tp.get_maxu());

        fprintf( rfp, "%d\n", result.contours() );
        for ( int i = 0; i < result.contours(); ++i ) {
            fprintf( rfp, "%d\n", result.contour_size(i) );
            fprintf( rfp, "%d\n", result.get_hole_flag(i) );
            for ( int j = 0; j < result.contour_size(i); ++j ) {
                p = result.get_pt( i, j );
                if ( preserve3d )
                    fprintf( rfp, "%.15f  %.15f %.15f\n", p.x(), p.y(), p.z() );
                else
                    fprintf( rfp, "%.15f  %.15f\n", p.x(), p.y() );
            }
        }
        fclose( rfp );
    }
}


// process polygon shape (chop up along tile boundaries and write each
// polygon piece to a file)
void tgChopNormalPolygon( const string& path, const string& poly_type,
                          const TGPolygon& shape, bool preserve3d )
{
    Point3D min, max, p;
    // point2d min, max;
    long int index;
    int i, j;

    // bail out immediately if polygon is empty
    if ( shape.contours() == 0 )
        return;

    min = Point3D(  200.0 );
    max = Point3D( -200.0 );

    // find min/max of polygon
    for ( i = 0; i < shape.contours(); i++ ) {
        for ( j = 0; j < shape.contour_size(i); j++ ) {
            p = shape.get_pt( i, j );

            if ( p.x() < min.x() ) min.setx( p.x() ); if ( p.y() < min.y() ) min.sety( p.y() ); if ( p.x() > max.x() ) max.setx( p.x() ); if ( p.y() > max.y() ) max.sety( p.y() );
        }
    }

    // get next polygon index
    index = poly_index_next();

    SG_LOG( SG_GENERAL, SG_DEBUG, "  min = " << min << " max = " << max );

    // find buckets for min, and max points of convex hull.
    // note to self: self, you should think about checking for
    // polygons that span the date line
    SGBucket b_min( min.x(), min.y() );
    SGBucket b_max( max.x(), max.y() );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  Bucket min = " << b_min );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  Bucket max = " << b_max );

    if ( b_min == b_max ) {
        // shape entirely contained in a single bucket, write and bail
        clip_and_write_poly( path, index, poly_type, b_min, shape, preserve3d );
        return;
    }

    SGBucket b_cur;
    int dx, dy;

    sgBucketDiff(b_min, b_max, &dx, &dy);
    SG_LOG( SG_GENERAL, SG_DEBUG,
            "  polygon spans tile boundaries" );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  dx = " << dx
                                            << "  dy = " << dy );

    if ( (dx > 2880) || (dy > 1440) )
        throw sg_exception("something is really wrong in split_polygon()!!!!");

    if ( dy <= 1 ) {
        // we are down to at most two rows, write each column and then bail
        double min_center_lat = b_min.get_center_lat();
        double min_center_lon = b_min.get_center_lon();
        for ( j = 0; j <= dy; ++j ) {
            for ( i = 0; i <= dx; ++i ) {
                b_cur = sgBucketOffset(min_center_lon, min_center_lat, i, j);
                clip_and_write_poly( path, index, poly_type, b_cur, shape,
                                     preserve3d );
            }
        }
        return;
    }

    // we have two or more rows left, split in half (along a
    // horizontal dividing line) and recurse with each half

    // find mid point (integer math)
    int mid = (dy + 1) / 2 - 1;

    // determine horizontal clip line
    SGBucket b_clip = sgBucketOffset(min.x(), min.y(), 0, mid);
    double clip_line = b_clip.get_center_lat();
    if ( (clip_line >= -90.0 + SG_HALF_BUCKET_SPAN)
         && (clip_line < 90.0 - SG_HALF_BUCKET_SPAN) )
        clip_line += SG_HALF_BUCKET_SPAN;
    else if ( clip_line < -89.0 )
        clip_line = -89.0;
    else if ( clip_line >= 89.0 )
        clip_line = 90.0;
    else {
        SG_LOG( SG_GENERAL, SG_ALERT,
                "Out of range latitude in clip_and_write_poly() = "
                << clip_line );
    }

    {
        //
        // Crop bottom area (hopefully by putting this in it's own
        // scope we can shorten the life of some really large data
        // structures to reduce memory use)
        //

        SG_LOG( SG_GENERAL, SG_DEBUG,
                "Generating bottom half (" << min.y() << "-" <<
                clip_line << ")" );

        TGPolygon bottom, bottom_clip;

        bottom.erase();
        bottom_clip.erase();

        bottom.add_node( 0, Point3D(-180.0, min.y(), 0) );
        bottom.add_node( 0, Point3D(180.0, min.y(), 0) );
        bottom.add_node( 0, Point3D(180.0, clip_line, 0) );
        bottom.add_node( 0, Point3D(-180.0, clip_line, 0) );

        bottom_clip = tgPolygonInt( bottom, shape );

        tgChopNormalPolygon( path, poly_type, bottom_clip, preserve3d );
    }

    {
        //
        // Crop top area (hopefully by putting this in it's own scope
        // we can shorten the life of some really large data
        // structures to reduce memory use)
        //

        SG_LOG( SG_GENERAL, SG_DEBUG,
                "Generating top half (" << clip_line << "-" <<
                max.y() << ")" );

        TGPolygon top, top_clip;

        top.erase();
        top_clip.erase();

        top.add_node( 0, Point3D(-180.0, clip_line, 0) );
        top.add_node( 0, Point3D(180.0, clip_line, 0) );
        top.add_node( 0, Point3D(180.0, max.y(), 0) );
        top.add_node( 0, Point3D(-180.0, max.y(), 0) );

        top_clip = tgPolygonInt( top, shape );

        tgChopNormalPolygon( path, poly_type, top_clip, preserve3d );
    }
}

void tgChopNormalPolygonsWithMask(const std::string& path, const std::string& poly_type, 
                                  const poly_list& segments, bool preserve3d )
{
    Point3D min, max, p;
    TGPolygon shape;
    long int index;
    int i, j;
    unsigned int s;

    // bail out immediately if poly_list is empty
    if ( segments.size() == 0 )
        return;

    // get next polygon index
    index = poly_index_next();

    min = Point3D(  200.0 );
    max = Point3D( -200.0 );

    for ( s = 0; s < segments.size(); s++) {
        shape = segments[s];

        // find min/max of polygon
        for ( i = 0; i < shape.contours(); i++ ) {
            for ( j = 0; j < shape.contour_size(i); j++ ) {
                p = shape.get_pt( i, j );

                if ( p.x() < min.x() ) min.setx( p.x() ); if ( p.y() < min.y() ) min.sety( p.y() ); if ( p.x() > max.x() ) max.setx( p.x() ); if ( p.y() > max.y() ) max.sety( p.y() );
            }
        }
    }

    SG_LOG( SG_GENERAL, SG_DEBUG, "  min = " << min << " max = " << max );

    // find buckets for min, and max points of convex hull.
    // note to self: self, you should think about checking for
    // polygons that span the date line
    SGBucket b_min( min.x(), min.y() );
    SGBucket b_max( max.x(), max.y() );

    SG_LOG( SG_GENERAL, SG_DEBUG, "  Bucket min = " << b_min );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  Bucket max = " << b_max );

    if ( b_min == b_max ) {
        // shape entirely contained in a single bucket, write and bail
        clip_and_write_polys_with_mask( path, index, poly_type, b_min, segments, preserve3d );
        return;
    }

    SGBucket b_cur;
    int dx, dy;

    sgBucketDiff(b_min, b_max, &dx, &dy);
    SG_LOG( SG_GENERAL, SG_DEBUG, "  polygon spans tile boundaries" );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  dx = " << dx << "  dy = " << dy );

    if ( (dx > 2880) || (dy > 1440) )
        throw sg_exception("something is really wrong in split_polygon()!!!!");

    if ( dy <= 1 ) {
        // we are down to at most two rows, write each column and then bail
        double min_center_lat = b_min.get_center_lat();
        double min_center_lon = b_min.get_center_lon();
        for ( j = 0; j <= dy; ++j ) {
            for ( i = 0; i <= dx; ++i ) {
                b_cur = sgBucketOffset(min_center_lon, min_center_lat, i, j);
                clip_and_write_polys_with_mask( path, index, poly_type, b_cur, segments, preserve3d );
            }
        }
        return;
    }

    // we have two or more rows left, split in half (along a
    // horizontal dividing line) and recurse with each half

    // find mid point (integer math)
    int mid = (dy + 1) / 2 - 1;

    // determine horizontal clip line
    SGBucket b_clip = sgBucketOffset(min.x(), min.y(), 0, mid);
    double clip_line = b_clip.get_center_lat();
    if ( (clip_line >= -90.0 + SG_HALF_BUCKET_SPAN)
         && (clip_line < 90.0 - SG_HALF_BUCKET_SPAN) )
        clip_line += SG_HALF_BUCKET_SPAN;
    else if ( clip_line < -89.0 )
        clip_line = -89.0;
    else if ( clip_line >= 89.0 )
        clip_line = 90.0;
    else {
        SG_LOG( SG_GENERAL, SG_ALERT,
                "Out of range latitude in clip_and_write_poly() = "
                << clip_line );
    }

    {
        //
        // Crop bottom area (hopefully by putting this in it's own
        // scope we can shorten the life of some really large data
        // structures to reduce memory use)
        //
        SG_LOG( SG_GENERAL, SG_DEBUG,
                "Generating bottom half (" << min.y() << "-" <<
                clip_line << ")" );

        TGPolygon bottom, bottom_clip;
        poly_list bottom_clip_list;

        bottom.erase();
        bottom.add_node( 0, Point3D(-180.0, min.y(), 0) );
        bottom.add_node( 0, Point3D(180.0, min.y(), 0) );
        bottom.add_node( 0, Point3D(180.0, clip_line, 0) );
        bottom.add_node( 0, Point3D(-180.0, clip_line, 0) );

        for (s=0; s<segments.size(); s++) {            
            bottom_clip.erase();
            bottom_clip = tgPolygonInt( bottom, segments[s] );

            if ( bottom_clip.contours() > 0 ) {
                bottom_clip_list.push_back( bottom_clip );
            }
        }

        tgChopNormalPolygonsWithMask( path, poly_type, bottom_clip_list, preserve3d );
    }

    {
        //
        // Crop top area (hopefully by putting this in it's own scope
        // we can shorten the life of some really large data
        // structures to reduce memory use)
        //

        SG_LOG( SG_GENERAL, SG_DEBUG,
                "Generating top half (" << clip_line << "-" <<
                max.y() << ")" );

        TGPolygon top, top_clip;
        poly_list top_clip_list;

        top.erase();
        top.add_node( 0, Point3D(-180.0, clip_line, 0) );
        top.add_node( 0, Point3D(180.0, clip_line, 0) );
        top.add_node( 0, Point3D(180.0, max.y(), 0) );
        top.add_node( 0, Point3D(-180.0, max.y(), 0) );

        for (s=0; s<segments.size(); s++) {            
            top_clip.erase();
            top_clip = tgPolygonInt( top, segments[s] );

            if ( top_clip.contours() > 0 ) {
                top_clip_list.push_back( top_clip );
            }
        }

        tgChopNormalPolygonsWithMask( path, poly_type, top_clip_list, preserve3d );
    }
}

// process polygon shape (chop up along tile boundaries and write each
// polygon piece to a file)
void tgChopNormalPolygonTP( const string& path, const string& poly_type,
                            const TGPolygon& shape, const TGTexParams& tp, bool preserve3d )
{
    Point3D min, max, p;
    // point2d min, max;
    long int index;
    int i, j;

    // bail out immediately if polygon is empty
    if ( shape.contours() == 0 )
        return;

    min = Point3D(  200.0 );
    max = Point3D( -200.0 );

    // find min/max of polygon
    for ( i = 0; i < shape.contours(); i++ ) {
        for ( j = 0; j < shape.contour_size(i); j++ ) {
            p = shape.get_pt( i, j );

            if ( p.x() < min.x() ) min.setx( p.x() ); if ( p.y() < min.y() ) min.sety( p.y() ); if ( p.x() > max.x() ) max.setx( p.x() ); if ( p.y() > max.y() ) max.sety( p.y() );
        }
    }

    // get next polygon index
    index = poly_index_next();

    SG_LOG( SG_GENERAL, SG_DEBUG, "  min = " << min << " max = " << max );

    // find buckets for min, and max points of convex hull.
    // note to self: self, you should think about checking for
    // polygons that span the date line
    SGBucket b_min( min.x(), min.y() );
    SGBucket b_max( max.x(), max.y() );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  Bucket min = " << b_min );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  Bucket max = " << b_max );

    if ( b_min == b_max ) {
        // shape entirely contained in a single bucket, write and bail
        clip_and_write_poly_tp( path, index, poly_type, b_min, shape, tp, preserve3d );
        return;
    }

    SGBucket b_cur;
    int dx, dy;

    sgBucketDiff(b_min, b_max, &dx, &dy);
    SG_LOG( SG_GENERAL, SG_DEBUG,
            "  polygon spans tile boundaries" );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  dx = " << dx
                                            << "  dy = " << dy );

    if ( (dx > 2880) || (dy > 1440) )
        throw sg_exception("something is really wrong in split_polygon()!!!!");

    if ( dy <= 1 ) {
        // we are down to at most two rows, write each column and then bail
        double min_center_lat = b_min.get_center_lat();
        double min_center_lon = b_min.get_center_lon();
        for ( j = 0; j <= dy; ++j ) {
            for ( i = 0; i <= dx; ++i ) {
                b_cur = sgBucketOffset(min_center_lon, min_center_lat, i, j);
                clip_and_write_poly_tp( path, index, poly_type, b_cur, shape, tp,
                                        preserve3d );
            }
        }
        return;
    }

    // we have two or more rows left, split in half (along a
    // horizontal dividing line) and recurse with each half

    // find mid point (integer math)
    int mid = (dy + 1) / 2 - 1;

    // determine horizontal clip line
    SGBucket b_clip = sgBucketOffset(min.x(), min.y(), 0, mid);
    double clip_line = b_clip.get_center_lat();
    if ( (clip_line >= -90.0 + SG_HALF_BUCKET_SPAN)
         && (clip_line < 90.0 - SG_HALF_BUCKET_SPAN) )
        clip_line += SG_HALF_BUCKET_SPAN;
    else if ( clip_line < -89.0 )
        clip_line = -89.0;
    else if ( clip_line >= 89.0 )
        clip_line = 90.0;
    else {
        SG_LOG( SG_GENERAL, SG_ALERT,
                "Out of range latitude in clip_and_write_poly() = "
                << clip_line );
    }

    {
        //
        // Crop bottom area (hopefully by putting this in it's own
        // scope we can shorten the life of some really large data
        // structures to reduce memory use)
        //

        SG_LOG( SG_GENERAL, SG_DEBUG,
                "Generating bottom half (" << min.y() << "-" <<
                clip_line << ")" );

        TGPolygon bottom, bottom_clip;

        bottom.erase();
        bottom_clip.erase();

        bottom.add_node( 0, Point3D(-180.0, min.y(), 0) );
        bottom.add_node( 0, Point3D(180.0, min.y(), 0) );
        bottom.add_node( 0, Point3D(180.0, clip_line, 0) );
        bottom.add_node( 0, Point3D(-180.0, clip_line, 0) );

        bottom_clip = tgPolygonInt( bottom, shape );

        // the texparam should be constant over each clipped poly.
        // when they are reassembled, we want the texture map to
        // be seamless
        tgChopNormalPolygonTP( path, poly_type, bottom_clip, tp, preserve3d );
    }

    {
        //
        // Crop top area (hopefully by putting this in it's own scope
        // we can shorten the life of some really large data
        // structures to reduce memory use)
        //

        SG_LOG( SG_GENERAL, SG_DEBUG,
                "Generating top half (" << clip_line << "-" <<
                max.y() << ")" );

        TGPolygon top, top_clip;

        top.erase();
        top_clip.erase();

        top.add_node( 0, Point3D(-180.0, clip_line, 0) );
        top.add_node( 0, Point3D(180.0, clip_line, 0) );
        top.add_node( 0, Point3D(180.0, max.y(), 0) );
        top.add_node( 0, Point3D(-180.0, max.y(), 0) );

        top_clip = tgPolygonInt( top, shape );

        tgChopNormalPolygonTP( path, poly_type, top_clip, tp, preserve3d );
    }
}

void tgChopNormalPolygonsWithTP(const std::string& path, const std::string& poly_type,
                                const poly_list& segments, const texparams_list& tps, bool preserve3d )
{
    Point3D min, max, p;
    TGPolygon shape;
    long int index;
    int i, j;
    unsigned int s;

    // bail out immediately if poly_list is empty
    if ( segments.size() == 0 )
        return;

    // get next polygon index
    index = poly_index_next();

    min = Point3D(  200.0 );
    max = Point3D( -200.0 );

    for ( s = 0; s < segments.size(); s++) {
        shape = segments[s];

        // find min/max of polygon
        for ( i = 0; i < shape.contours(); i++ ) {
            for ( j = 0; j < shape.contour_size(i); j++ ) {
                p = shape.get_pt( i, j );

                if ( p.x() < min.x() ) min.setx( p.x() ); if ( p.y() < min.y() ) min.sety( p.y() ); if ( p.x() > max.x() ) max.setx( p.x() ); if ( p.y() > max.y() ) max.sety( p.y() );
            }
        }
    }

    SG_LOG( SG_GENERAL, SG_DEBUG, "  min = " << min << " max = " << max );

    // find buckets for min, and max points of convex hull.
    // note to self: self, you should think about checking for
    // polygons that span the date line
    SGBucket b_min( min.x(), min.y() );
    SGBucket b_max( max.x(), max.y() );

    SG_LOG( SG_GENERAL, SG_DEBUG, "  Bucket min = " << b_min );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  Bucket max = " << b_max );

    if ( b_min == b_max ) {
        // shape entirely contained in a single bucket, write and bail
        clip_and_write_polys_with_tps( path, index, poly_type, b_min, segments, tps, preserve3d );
        return;
    }

    SGBucket b_cur;
    int dx, dy;

    sgBucketDiff(b_min, b_max, &dx, &dy);
    SG_LOG( SG_GENERAL, SG_DEBUG, "  polygon spans tile boundaries" );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  dx = " << dx << "  dy = " << dy );

    if ( (dx > 2880) || (dy > 1440) )
        throw sg_exception("something is really wrong in split_polygon()!!!!");

    if ( dy <= 1 ) {
        // we are down to at most two rows, write each column and then bail
        double min_center_lat = b_min.get_center_lat();
        double min_center_lon = b_min.get_center_lon();
        for ( j = 0; j <= dy; ++j ) {
            for ( i = 0; i <= dx; ++i ) {
                b_cur = sgBucketOffset(min_center_lon, min_center_lat, i, j);
                clip_and_write_polys_with_tps( path, index, poly_type, b_cur, segments, tps, preserve3d );
            }
        }
        return;
    }

    // we have two or more rows left, split in half (along a
    // horizontal dividing line) and recurse with each half
    // find mid point (integer math)
    int mid = (dy + 1) / 2 - 1;

    // determine horizontal clip line
    SGBucket b_clip = sgBucketOffset(min.x(), min.y(), 0, mid);
    double clip_line = b_clip.get_center_lat();
    if ( (clip_line >= -90.0 + SG_HALF_BUCKET_SPAN) &&
         (clip_line < 90.0 - SG_HALF_BUCKET_SPAN) )
        clip_line += SG_HALF_BUCKET_SPAN;
    else if ( clip_line < -89.0 )
        clip_line = -89.0;
    else if ( clip_line >= 89.0 )
        clip_line = 90.0;
    else {
        SG_LOG( SG_GENERAL, SG_ALERT,
                "Out of range latitude in clip_and_write_poly() = " << clip_line );
    }

    {
        //
        // Crop bottom area (hopefully by putting this in it's own
        // scope we can shorten the life of some really large data
        // structures to reduce memory use)
        //
        SG_LOG( SG_GENERAL, SG_DEBUG,
                "Generating bottom half (" << min.y() << "-" << clip_line << ")" );

        TGPolygon bottom, bottom_clip;
        poly_list bottom_clip_list;
        texparams_list bottom_tp_list;

        bottom.erase();
        bottom.add_node( 0, Point3D(-180.0, min.y(), 0) );
        bottom.add_node( 0, Point3D(180.0, min.y(), 0) );
        bottom.add_node( 0, Point3D(180.0, clip_line, 0) );
        bottom.add_node( 0, Point3D(-180.0, clip_line, 0) );

        for (s=0; s<segments.size(); s++) {
            bottom_clip.erase();
            bottom_clip = tgPolygonInt( bottom, segments[s] );

            if ( bottom_clip.contours() > 0 ) {
                bottom_clip_list.push_back( bottom_clip );
                bottom_tp_list.push_back( tps[s] );
            }
        }

        tgChopNormalPolygonsWithTP( path, poly_type, bottom_clip_list, bottom_tp_list, preserve3d );
    }

    {
        //
        // Crop top area (hopefully by putting this in it's own scope
        // we can shorten the life of some really large data
        // structures to reduce memory use)
        //

        SG_LOG( SG_GENERAL, SG_DEBUG,
                "Generating top half (" << clip_line << "-" <<
                                max.y() << ")" );

        TGPolygon top, top_clip;
        poly_list top_clip_list;
        texparams_list top_tp_list;

        top.erase();
        top.add_node( 0, Point3D(-180.0, clip_line, 0) );
        top.add_node( 0, Point3D(180.0, clip_line, 0) );
        top.add_node( 0, Point3D(180.0, max.y(), 0) );
        top.add_node( 0, Point3D(-180.0, max.y(), 0) );

        for (s=0; s<segments.size(); s++) {
            top_clip.erase();
            top_clip = tgPolygonInt( top, segments[s] );

            if ( top_clip.contours() > 0 ) {
                top_clip_list.push_back( top_clip );
                top_tp_list.push_back( tps[s] );
            }
        }

        tgChopNormalPolygonsWithTP( path, poly_type, top_clip_list, top_tp_list, preserve3d );
    }
}

// process polygon shape (chop up along tile boundaries and write each
// polygon piece to a file) This has a front end to a crude clipper
// that doesn't handle holes so beware.  This routine is appropriate
// for breaking down really huge structures if needed.
void tgChopBigSimplePolygon( const string& path, const string& poly_type,
                             const TGPolygon& shape, bool preserve3d )
{
    Point3D min, max, p;
    // point2d min, max;
    long int index;
    int i, j;

    // bail out immediately if polygon is empty
    if ( shape.contours() == 0 )
        return;

    min = Point3D(  200.0 );
    max = Point3D( -200.0 );

    // find min/max of polygon
    for ( i = 0; i < shape.contours(); i++ ) {
        for ( j = 0; j < shape.contour_size(i); j++ ) {
            p = shape.get_pt( i, j );

            if ( p.x() < min.x() ) min.setx( p.x() ); if ( p.y() < min.y() ) min.sety( p.y() ); if ( p.x() > max.x() ) max.setx( p.x() ); if ( p.y() > max.y() ) max.sety( p.y() );
        }
    }

    // get next polygon index
    index = poly_index_next();

    SG_LOG( SG_GENERAL, SG_DEBUG, "  min = " << min << " max = " << max );

    // find buckets for min, and max points of convex hull.
    // note to self: self, you should think about checking for
    // polygons that span the date line
    SGBucket b_min( min.x(), min.y() );
    SGBucket b_max( max.x(), max.y() );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  Bucket min = " << b_min );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  Bucket max = " << b_max );

    if ( b_min == b_max ) {
        // shape entirely contained in a single bucket, write and bail
        clip_and_write_poly( path, index, poly_type, b_min, shape, preserve3d );
        return;
    }

    SGBucket b_cur;
    int dx, dy;

    sgBucketDiff(b_min, b_max, &dx, &dy);
    SG_LOG( SG_GENERAL, SG_INFO,
            "  polygon spans tile boundaries" );
    SG_LOG( SG_GENERAL, SG_DEBUG, "  dx = " << dx
                                            << "  dy = " << dy );

    if ( (dx > 2880) || (dy > 1440) )
        throw sg_exception("something is really wrong in split_polygon()!!!!");

    if ( dy <= 1 ) {
        // we are down to at most two rows, write each column and then
        // bail
        double min_center_lat = b_min.get_center_lat();
        double min_center_lon = b_min.get_center_lon();
        for ( j = 0; j <= 1; ++j ) {
            for ( i = 0; i <= dx; ++i ) {
                b_cur = sgBucketOffset(min_center_lon, min_center_lat, i, j);
                clip_and_write_poly( path, index, poly_type, b_cur, shape,
                                     preserve3d );
            }
        }
        return;
    }

    // we have more than one row left, split in half and recurse with
    // each chunk

    // find mid point (integer math)
    int mid = (dy + 1) / 2 - 1;

    // determine horizontal clip line
    SGBucket b_clip = sgBucketOffset(min.x(), min.y(), 0, mid);
    double clip_line = b_clip.get_center_lat();
    if ( (clip_line >= -89.0) && (clip_line < 89.0) )
        clip_line += SG_HALF_BUCKET_SPAN;
    else if ( clip_line < -89.0 )
        clip_line = -89.0;
    else if ( clip_line >= 89.0 )
        clip_line = 90.0;
    else {
        SG_LOG( SG_GENERAL, SG_ALERT,
                "Out of range latitude in clip_and_write_poly() = "
                << clip_line );
    }

    {
        //
        // Crop bottom area (hopefully by putting this in it's own
        // scope we can shorten the life of some really large data
        // structures to reduce memory use)
        //

        SG_LOG( SG_GENERAL, SG_DEBUG,
                "Generating bottom half (" << min.y() << "-" <<
                clip_line << ")" );

        TGPolygon bottom, bottom_clip;
        if ( shape.total_size() < 50000 ) {
            bottom.erase();
            bottom_clip.erase();

            bottom.add_node( 0, Point3D(-180.0, min.y(), 0) );
            bottom.add_node( 0, Point3D(180.0, min.y(), 0) );
            bottom.add_node( 0, Point3D(180.0, clip_line, 0) );
            bottom.add_node( 0, Point3D(-180.0, clip_line, 0) );

            bottom_clip = tgPolygonInt( bottom, shape );
        } else
            bottom_clip = horizontal_clip( shape, clip_line, Below );

        tgChopBigSimplePolygon( path, poly_type, bottom_clip, preserve3d );
    }

    {
        //
        // Crop top area (hopefully by putting this in it's own scope
        // we can shorten the life of some really large data
        // structures to reduce memory use)
        //

        SG_LOG( SG_GENERAL, SG_DEBUG,
                "Generating top half (" << clip_line << "-" <<
                max.y() << ")" );

        TGPolygon top, top_clip;
        if ( shape.total_size() < 50000 ) {
            top.erase();
            top_clip.erase();

            top.add_node( 0, Point3D(-180.0, clip_line, 0) );
            top.add_node( 0, Point3D(180.0, clip_line, 0) );
            top.add_node( 0, Point3D(180.0, max.y(), 0) );
            top.add_node( 0, Point3D(-180.0, max.y(), 0) );

            top_clip = tgPolygonInt( top, shape );
        } else
            top_clip = horizontal_clip( shape, clip_line, Above );

        tgChopBigSimplePolygon( path, poly_type, top_clip, preserve3d );
    }
}

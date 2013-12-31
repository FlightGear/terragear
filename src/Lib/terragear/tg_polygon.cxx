// polygon.cxx -- polygon (with holes) management class
//
// Written by Curtis Olson, started March 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - http://www.flightgear.org/~curt
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//

#include <simgear/constants.h>
#include <simgear/threads/SGThread.hxx>
#include <simgear/threads/SGGuard.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/io/lowlevel.hxx>
#include <simgear/misc/texcoord.hxx>
#include <simgear/structure/exception.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/bucket/newbucket.hxx>

#include "tg_misc.hxx"
#include "tg_polygon.hxx"

// tgPolygon static functions
unsigned int tgPolygon::TotalNodes( void ) const
{
    unsigned int total_nodes = 0;

    for (unsigned int c = 0; c < contours.size(); c++) {
        total_nodes += contours[c].GetSize();
    }

    return total_nodes;
}

tgPolygon tgPolygon::Expand( const tgPolygon& subject, double offset )
{
    ClipperLib::Paths clipper_src, clipper_dst;
    clipper_src = tgPolygon::ToClipper( subject );
    tgPolygon result;

    ClipperLib::ClipperOffset co(2.0, 2.0);
    co.AddPaths(clipper_src, ClipperLib::jtSquare, ClipperLib::etClosedPolygon); 
    co.Execute(clipper_dst, Dist_ToClipper(offset) );

    result = tgPolygon::FromClipper( clipper_dst );

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );
    result.SetId( subject.GetId() );

    return result;
}

tgPolygon tgPolygon::Expand( const SGGeod& subject, double offset )
{
    tgPolygon result;
    tgContour contour;
    SGGeod    pt;

    pt = SGGeodesy::direct( subject, 90, offset/2.0 );
    double dlon = pt.getLongitudeDeg() - subject.getLongitudeDeg();

    pt = SGGeodesy::direct( subject, 0, offset/2.0 );
    double dlat = pt.getLatitudeDeg() - subject.getLatitudeDeg();

    contour.AddNode( SGGeod::fromDeg( subject.getLongitudeDeg() - dlon, subject.getLatitudeDeg() - dlat ) );
    contour.AddNode( SGGeod::fromDeg( subject.getLongitudeDeg() + dlon, subject.getLatitudeDeg() - dlat ) );
    contour.AddNode( SGGeod::fromDeg( subject.getLongitudeDeg() + dlon, subject.getLatitudeDeg() + dlat ) );
    contour.AddNode( SGGeod::fromDeg( subject.getLongitudeDeg() - dlon, subject.getLatitudeDeg() + dlat ) );
    contour.SetHole(false);

    result.AddContour( contour );

    return result;
}

tgRectangle tgPolygon::GetBoundingBox( void ) const
{
    SGGeod min, max;

    double minx =  std::numeric_limits<double>::infinity();
    double miny =  std::numeric_limits<double>::infinity();
    double maxx = -std::numeric_limits<double>::infinity();
    double maxy = -std::numeric_limits<double>::infinity();

    for ( unsigned int i = 0; i < Contours(); i++ ) {
        for (unsigned int j = 0; j < ContourSize(i); j++) {
            SGGeod pt = GetNode(i,j);
            if ( pt.getLongitudeDeg() < minx ) { minx = pt.getLongitudeDeg(); }
            if ( pt.getLongitudeDeg() > maxx ) { maxx = pt.getLongitudeDeg(); }
            if ( pt.getLatitudeDeg()  < miny ) { miny = pt.getLatitudeDeg(); }
            if ( pt.getLatitudeDeg()  > maxy ) { maxy = pt.getLatitudeDeg(); }
        }
    }

    min = SGGeod::fromDeg( minx, miny );
    max = SGGeod::fromDeg( maxx, maxy );

    return tgRectangle( min, max );
}

tgPolygon tgPolygon::AddColinearNodes( const tgPolygon& subject, std::vector<SGGeod>& nodes )
{
    tgPolygon result;

    result.SetMaterial( subject.GetMaterial() );
    result.SetTexParams( subject.GetTexParams() );
    result.SetId( subject.GetId() );
    
    for ( unsigned int c = 0; c < subject.Contours(); c++ ) {
        result.AddContour( tgContour::AddColinearNodes( subject.GetContour(c), nodes ) );
    }

    return result;
}

tgPolygon tgPolygon::AddColinearNodes( const tgPolygon& subject, UniqueSGGeodSet& nodes )
{
    return AddColinearNodes( subject, nodes.get_list() );
}

// this is the opposite of FindColinearNodes - it takes a single SGGeode,
// and tries to find the line segment the point is colinear with
bool tgPolygon::FindColinearLine( const tgPolygon& subject, SGGeod& node, SGGeod& start, SGGeod& end )
{
    bool found = false;

    for ( unsigned int c = 0; c < subject.Contours() && !found; c++ ) {
        found = tgContour::FindColinearLine( subject.GetContour(c), node, start, end );
    }

    return found;
}

SGGeod InterpolateElevation( const SGGeod& dst_node, const SGGeod& start, const SGGeod& end )
{
    double total_dist = SGGeodesy::distanceM( start, end );
    double inter_dist = SGGeodesy::distanceM( start, dst_node );
    double delta = inter_dist/total_dist;

    double dest_elevation = start.getElevationM() + (delta * ( end.getElevationM() - start.getElevationM() ));

    return SGGeod::fromDegM( dst_node.getLongitudeDeg(), dst_node.getLatitudeDeg(), dest_elevation );
}


void tgPolygon::InheritElevations( const tgPolygon& source )
{
    UniqueSGGeodSet     src_nodes;

    // build a list of points from the source polygon
    for ( unsigned int i = 0; i < source.Contours(); ++i ) {
        for ( unsigned int j = 0; j < source.ContourSize(i); ++j ) {
            src_nodes.add( source.GetNode( i, j ) );
        }
    }

    // traverse the dest polygon and build a mirror image but with
    // elevations from the source polygon
    for ( unsigned int i = 0; i < contours.size(); ++i ) {
        for ( unsigned int j = 0; j < contours[i].GetSize(); ++j ) {
            SGGeod dst_node = GetNode(i,j);
            int index = src_nodes.find( dst_node );
            if ( index >= 0 ) {
                SetNode( i, j, src_nodes.get_list()[index] );
            } else {
                /* node not is source - we need to find the two points to interpolate from */
                SGGeod start, end, result;
                if ( FindColinearLine( source, dst_node, start, end ) ) {
                    dst_node = InterpolateElevation( dst_node, start, end );
                    SetNode( i, j, dst_node );
                }
            }
        }
    }
}

void tgPolygon::Texture( void )
{
    SGGeod  p;
    SGVec2f t;
    double  x, y;
    float   tx, ty;

    SG_LOG(SG_GENERAL, SG_DEBUG, "Texture Poly with material " << material << " method " << tp.method << " tpref " << tp.ref << " heading " << tp.heading );

    switch( tp.method ) {
        case TG_TEX_BY_GEODE:
        {
            // The Simgear General texture coordinate routine takes a fan.
            // Simgear could probably use a new function that just takes a Geod vector
            // For now, just create an identity fan...
            std::vector< int > node_idxs;
            for (int i = 0; i < 3; i++) {
                node_idxs.push_back(i);
            }

            for ( unsigned int i = 0; i < triangles.size(); i++ ) {
                std::vector< SGVec2f > tc_list;
                std::vector< SGGeod > nodes;

                nodes = triangles[i].GetNodeList();
                tc_list = sgCalcTexCoords( tp.center_lat, nodes, node_idxs );
                triangles[i].SetTexCoordList( tc_list );
            }
        }
        break;

        case TG_TEX_BY_TPS_NOCLIP:
        case TG_TEX_BY_TPS_CLIPU:
        case TG_TEX_BY_TPS_CLIPV:
        case TG_TEX_BY_TPS_CLIPUV:
        {
            for ( unsigned int i = 0; i < triangles.size(); i++ ) {
                for ( unsigned int j = 0; j < 3; j++ ) {
                    p = triangles[i].GetNode( j );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "point = " << p);

                    //
                    // 1. Calculate distance and bearing from the center of
                    // the poly
                    //

                    // given alt, lat1, lon1, lat2, lon2, calculate starting
                    // and ending az1, az2 and distance (s).  Lat, lon, and
                    // azimuth are in degrees.  distance in meters
                    double az1, az2, dist;
                    SGGeodesy::inverse( tp.ref, p, az1, az2, dist );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "basic course = " << az2);

                    //
                    // 2. Rotate this back into a coordinate system where Y
                    // runs the length of the poly and X runs crossways.
                    //

                    double course = SGMiscd::normalizePeriodic(0, 360, az2 - tp.heading);
                    SG_LOG( SG_GENERAL, SG_DEBUG,"  course = " << course << "  dist = " << dist );

                    //
                    // 3. Convert from polar to cartesian coordinates
                    //

                    x = sin( course * SGD_DEGREES_TO_RADIANS ) * dist;
                    y = cos( course * SGD_DEGREES_TO_RADIANS ) * dist;
                    SG_LOG(SG_GENERAL, SG_DEBUG, "  x = " << x << " y = " << y);

                    //
                    // 4. Map x, y point into texture coordinates
                    //
                    float tmp;

                    tmp = (float)x / (float)tp.width;
                    tx = tmp * (float)(tp.maxu - tp.minu) + (float)tp.minu;
                    SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << tx << ")");

                    // clip u?
                    if ( (tp.method == TG_TEX_BY_TPS_CLIPU) || (tp.method == TG_TEX_BY_TPS_CLIPUV) ) {
                        if ( tx < (float)tp.min_clipu ) { tx = (float)tp.min_clipu; }
                        if ( tx > (float)tp.max_clipu ) { tx = (float)tp.max_clipu; }
                    }

                    tmp = (float)y / (float)tp.length;
                    ty = tmp * (float)(tp.maxv - tp.minv) + (float)tp.minv;
                    SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << ty << ")");

                    // clip v?
                    if ( (tp.method == TG_TEX_BY_TPS_CLIPV) || (tp.method == TG_TEX_BY_TPS_CLIPUV) ) {
                        if ( ty < (float)tp.min_clipv ) { ty = (float)tp.min_clipv; }
                        if ( ty > (float)tp.max_clipv ) { ty = (float)tp.max_clipv; }
                    }

                    t = SGVec2f( tx, ty );
                    SG_LOG(SG_GENERAL, SG_DEBUG, "  (" << tx << ", " << ty << ")");

                    triangles[i].SetTexCoord( j, t );
                }
            }
        }
        break;
    }
}

void tgPolygon::SaveToGzFile( gzFile& fp ) const
{
    // Save the contours
    sgWriteUInt( fp, contours.size() );
    for (unsigned int i = 0; i < contours.size(); i++) {
        contours[i].SaveToGzFile( fp );
    }

    // Save the triangles
    sgWriteUInt( fp, triangles.size() );
    for (unsigned int i = 0; i < triangles.size(); i++) {
        triangles[i].SaveToGzFile( fp );
    }

    // Save the tex params
    tp.SaveToGzFile( fp );

    // and the rest
    sgWriteString( fp, material.c_str() );
    sgWriteString( fp, flag.c_str() );
    sgWriteInt( fp, (int)preserve3d );
}

void tgPolygon::LoadFromGzFile( gzFile& fp )
{
    unsigned int count;
    tgContour contour;
    tgTriangle triangle;
    char *strbuff;

    // Start clean
    Erase();

    // Load the contours
    sgReadUInt( fp, &count );
    for (unsigned int i = 0; i < count; i++) {
        contour.LoadFromGzFile( fp );
        AddContour(contour);
    }

    // load the triangles
    sgReadUInt( fp, &count );
    for (unsigned int i = 0; i < count; i++) {
        triangle.LoadFromGzFile( fp );
        AddTriangle(triangle);
    }

    // Load the tex params
    tp.LoadFromGzFile( fp );

    // and the rest
    sgReadString( fp, &strbuff );
    if ( strbuff ) {
        material = strbuff;
        delete[] strbuff;
    }

    sgReadString( fp, &strbuff );
    if ( strbuff ) {
        flag = strbuff;
        delete[] strbuff;
    }

    sgReadInt( fp, (int *)&preserve3d );
}

// Friends for serialization
std::ostream& operator<< ( std::ostream& output, const tgPolygon& subject )
{
    // Save the data
    output << "NumContours: " << subject.contours.size() << "\n";

    for( unsigned int c=0; c<subject.contours.size(); c++) {
        output << subject.contours[c];
    }

    output << "NumTriangles: " << subject.triangles.size() << "\n";
    for( unsigned int t=0; t<subject.triangles.size(); t++) {
        output << subject.triangles[t];
    }

    output << "Material: " << subject.material;
    output << "Flag: " << subject.flag;
    output << subject.tp;

    return output;
}

std::ostream& operator<< ( std::ostream& output, const tgTriangle& subject )
{
    output << "nodes\n";
    if ( subject.node_list.size() == 3 ) {
        output << subject.node_list[0] << ", " << subject.node_list[1] << ", " << subject.node_list[2] << "\n";
    } else {
        output << "empty\n";
    }

    output << "normals\n";
    if ( subject.norm_list.size() == 3 ) {
        output << subject.norm_list[0] << ", " << subject.norm_list[1] << ", " << subject.norm_list[2] << "\n";
    } else {
        output << "empty\n";
    }

    output << "texture coords\n";
    if ( subject.tc_list.size() == 3 ) {
        output << subject.tc_list[0] << ", " << subject.tc_list[1] << ", " << subject.tc_list[2] << "\n";
    } else {
        output << "empty\n";
    }

    output << "node indexes\n";
    if ( subject.idx_list.size() == 3 ) {
        output << subject.idx_list[0] << ", " << subject.idx_list[1] << ", " << subject.idx_list[2] << "\n";
    } else {
        output << "empty\n";
    }

    output << "Face normal: " << subject.face_normal << "\n";
    output << "Face area: "   << subject.face_area << "\n";

    return output;
}

void tgTriangle::SaveToGzFile( gzFile& fp ) const
{
    // Save the three nodes, and their attributes
    for (unsigned int i = 0; i < 3; i++) {
        sgWriteGeod( fp, node_list[i] );
        // sgWriteVec2( fp, tc_list[i] );
        // sgWritedVec3( fp, norm_list[i] ); // not calculated until stage 3
        sgWriteInt( fp, idx_list[i] );
    }
}

void tgTriangle::LoadFromGzFile( gzFile& fp )
{
    // Load the nodelist
    for (unsigned int i = 0; i < 3; i++) {
        sgReadGeod( fp, node_list[i] );
        // sgReadVec2( fp, tc_list[i] );
        // sgReaddVec3( fp, norm_list[i] );
        sgReadInt( fp, &idx_list[i] );
    }
}

std::ostream& operator<< ( std::ostream& output, const tgTexParams& subject )
{
    // Save the data
    output << "Ref    : " << subject.ref;
    output << "Width  : " << subject.width;
    output << "Length : " << subject.length;
    output << "Heading: " << subject.heading;

    output << "u: (" << subject.minu << "," << subject.maxu << ")";
    output << "v: (" << subject.minv << "," << subject.maxv << ")";

    output << "method: " << subject.method;

    return output;
}

void tgTexParams::SaveToGzFile( gzFile& fp ) const
{
    // Save the parameters
    sgWriteInt( fp, (int)method );

    if ( method == TG_TEX_BY_GEODE ) {
        sgWriteDouble( fp, center_lat );
    } else {
        sgWriteGeod( fp, ref );
        sgWriteDouble( fp, width );
        sgWriteDouble( fp, length );
        sgWriteDouble( fp, heading );

        sgWriteDouble( fp, minu );
        sgWriteDouble( fp, maxu );
        sgWriteDouble( fp, minv );
        sgWriteDouble( fp, maxv );

        if ( (method == TG_TEX_BY_TPS_CLIPU) ||
             (method == TG_TEX_BY_TPS_CLIPUV) ) {
            sgWriteDouble( fp, min_clipu );
            sgWriteDouble( fp, max_clipu );
        }

        if ( (method == TG_TEX_BY_TPS_CLIPV) ||
             (method == TG_TEX_BY_TPS_CLIPUV) ) {
            sgWriteDouble( fp, min_clipv );
            sgWriteDouble( fp, max_clipv );
        }
    }
}

void tgTexParams::LoadFromGzFile( gzFile& fp )
{
    // Load the parameters
    sgReadInt( fp, (int*)&method );

    if ( method == TG_TEX_BY_GEODE ) {
        sgReadDouble( fp, &center_lat );
    } else {
        sgReadGeod( fp, ref );
        sgReadDouble( fp, &width );
        sgReadDouble( fp, &length );
        sgReadDouble( fp, &heading );

        sgReadDouble( fp, &minu );
        sgReadDouble( fp, &maxu );
        sgReadDouble( fp, &minv );
        sgReadDouble( fp, &maxv );

        if ( (method == TG_TEX_BY_TPS_CLIPU) ||
             (method == TG_TEX_BY_TPS_CLIPUV) ) {
            sgReadDouble( fp, &min_clipu );
            sgReadDouble( fp, &max_clipu );
        }

        if ( (method == TG_TEX_BY_TPS_CLIPV) ||
             (method == TG_TEX_BY_TPS_CLIPUV) ) {
            sgReadDouble( fp, &min_clipv );
            sgReadDouble( fp, &max_clipv );
        }
    }
}
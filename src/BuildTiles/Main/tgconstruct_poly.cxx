// tgconstruct_poly.cxx -- load and handle polygon data
//
// Written by Curtis Olson, started May 1999.
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
// $Id: construct.cxx,v 1.4 2004-11-19 22:25:49 curt Exp $

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <boost/foreach.hpp>

#include <simgear/misc/sg_dir.hxx>
#include <simgear/debug/logstream.hxx>

#include <Geometry/poly_support.hxx>
#include <Osgb36/osgb36.hxx>

#include "tgconstruct.hxx"

using std::string;

static unsigned int cur_poly_id = 0;

// Add a polygon to the clipper. - only used by load_osgb36_poly - make that function more like ogr load
void TGConstruct::add_poly( int area, const TGPolygon &poly, string material ) {
    TGShape shape;
    TGSuperPoly sp;

    if ( area < TG_MAX_AREA_TYPES ) {
        sp.set_poly( poly );
        sp.set_material( material );

        shape.sps.push_back( sp );

        polys_in.add_shape( area, shape );
    } else {
        SG_LOG( SG_CLIPPER, SG_ALERT, "Polygon type out of range = " << area);
        exit(-1);
    }
}

bool TGConstruct::load_poly(const string& path) {
    bool poly3d = false;
    bool with_tp = false;
    string first_line;
    string poly_name;
    AreaType poly_type;
    int contours, count, i, j, k;
    int hole_flag;
    int num_polys;
    double startx, starty, startz, x, y, z, lastx, lasty, lastz;

    sg_gzifstream in( path );

    if ( !in ) {
        SG_LOG( SG_CLIPPER, SG_ALERT, "Cannot open file: " << path );
        exit(-1);
    }

    TGPolygon   poly;
    TGTexParams tp;
    Point3D     p;

    // (this could break things, why is it here) in >> skipcomment;
    while ( !in.eof() ) {
        in >> first_line;
        if ( first_line == "#2D" ) {
            poly3d = false;
            with_tp = false;

            in >> poly_name;
            num_polys = 1;
        } else if ( first_line == "#2D_WITH_MASK" ) {
            poly3d = false;
            with_tp = false;

            in >> poly_name;
            in >> num_polys;
        } else if ( first_line == "#2D_WITH_TPS" ) {
            poly3d = false;
            with_tp = true;

            in >> poly_name;
            in >> num_polys;
        } else if ( first_line == "#3D" ) {
            poly3d = true;
            with_tp = false;

            in >> poly_name;
            num_polys = 1;
        } else {
            // support old format (default to 2d)
            poly3d = false;
            with_tp = false;

            poly_name = first_line;
            num_polys = 1;
        }
        poly_type = get_area_type( poly_name );

        int area = (int)poly_type;
        string material;

        // only allow 1000 shapes per material
        int extension = polys_in.area_size( area ) / 1000;

        if (extension)
        {
            char buff[32];
            sprintf( buff, "%s_%d", get_area_name( area ).c_str(), extension );
            material = buff;
        }
        else
        {
            material = get_area_name( area );
        }


        // Generate a new Shape for the poly
        TGShape     shape;
        TGSuperPoly sp;

        for (k=0; k<num_polys;k++) {

            if ( with_tp ) {
                double width, length;
                double heading;
                double minu, maxu;
                double minv, maxv;

                in >> x;
                in >> y;
                in >> width;
                in >> length;
                in >> heading;
                in >> minu;
                in >> maxu;
                in >> minv;
                in >> maxv;

                tp.set_ref( Point3D(x, y, 0.0f) );
                tp.set_width( width );
                tp.set_length( length );
                tp.set_heading( heading );
                tp.set_minu( minu );
                tp.set_maxu( maxu );
                tp.set_minv( minv );
                tp.set_maxv( maxv );
            }

            in >> contours;

            poly.erase();

            for ( i = 0; i < contours; ++i ) {
                in >> count;

                if ( count < 3 ) {
                    SG_LOG( SG_CLIPPER, SG_ALERT, "Polygon with less than 3 data points." );
                    exit(-1);
                }

                in >> hole_flag;

                in >> startx;
                in >> starty;
                if ( poly3d ) {
                    in >> startz;
                } else {
                    startz = -9999.0;
                }

                p = Point3D(startx+nudge, starty+nudge, startz);
                p.snap( gSnap );
                poly.add_node( i, p );

                if ( poly3d ) {
                    nodes.unique_add_fixed_elevation( p );
                } else {
                    nodes.unique_add( p );
                }

                for ( j = 1; j < count - 1; ++j ) {
                    in >> x;
                    in >> y;
                    if ( poly3d ) {
                        in >> z;
                    } else {
                        z = -9999.0;
                    }
                    p = Point3D( x+nudge, y+nudge, z );
                    p.snap( gSnap );
                    poly.add_node( i, p );
                    if ( poly3d ) {
                        nodes.unique_add_fixed_elevation( p );
                    } else {
                        nodes.unique_add( p );
                    }
                }

                in >> lastx;
                in >> lasty;
                if ( poly3d ) {
                    in >> lastz;
                } else {
                    lastz = -9999.0;
                }

                if ( (fabs(startx - lastx) < SG_EPSILON) &&
                     (fabs(starty - lasty) < SG_EPSILON) &&
                     (fabs(startz - lastz) < SG_EPSILON) ) {
                    // last point same as first, discard
                } else {
                    p = Point3D( lastx+nudge, lasty+nudge, lastz );
                    p.snap( gSnap );
                    poly.add_node( i, p );
                    if ( poly3d ) {
                        nodes.unique_add_fixed_elevation( p );
                    } else {
                        nodes.unique_add( p );
                    }
                }
            }

            poly = remove_dups( poly );

            sp.set_poly( poly );
            sp.set_material( material );
            shape.sps.push_back( sp );

            if ( with_tp ) {
                shape.textured = true;
                shape.tps.push_back( tp );
            }
            else
            {
                shape.textured = false;
            }

            in >> skipcomment;
        }

        // Once the full poly is loaded, build the clip mask
        shape.BuildMask();
        shape.area = area;
        shape.id = cur_poly_id++;

        polys_in.add_shape( area, shape );

        if ( IsDebugShape( shape.id ) ) {
            WriteDebugShape( "loaded", shape );
        }
    }

    return true;
}


// Load a polygon definition file containing osgb36 Eastings and Northings
// and convert them to WGS84 Latitude and Longitude
bool TGConstruct::load_osgb36_poly(const string& path) {
    string poly_name;
    AreaType poly_type;
    int contours, count, i, j;
    int hole_flag;
    double startx, starty, x, y, lastx, lasty;

    SG_LOG( SG_CLIPPER, SG_INFO, "Loading " << path << " ..." );

    sg_gzifstream in( path );

    if ( !in ) {
        SG_LOG( SG_CLIPPER, SG_ALERT, "Cannot open file: " << path );
        exit(-1);
    }

    TGPolygon poly;

    Point3D p;
    Point3D OSRef;
    in >> skipcomment;
    while ( !in.eof() ) {
        in >> poly_name;
        SG_LOG( SG_CLIPPER, SG_INFO, "poly name = " << poly_name);
        poly_type = get_area_type( poly_name );
        SG_LOG( SG_CLIPPER, SG_INFO, "poly type (int) = " << (int)poly_type);
        in >> contours;
        SG_LOG( SG_CLIPPER, SG_INFO, "num contours = " << contours);

        poly.erase();

        for ( i = 0; i < contours; ++i ) {
            in >> count;

            if ( count < 3 ) {
                SG_LOG( SG_CLIPPER, SG_ALERT, "Polygon with less than 3 data points." );
                exit(-1);
            }

            in >> hole_flag;

            in >> startx;
            in >> starty;
            OSRef = Point3D(startx, starty, -9999.0);

            //Convert from OSGB36 Eastings/Northings to WGS84 Lat/Lon
            //Note that startx and starty themselves must not be altered since we compare them with unaltered lastx and lasty later
            p = OSGB36ToWGS84(OSRef);

            poly.add_node( i, p );
            nodes.unique_add( p );

            SG_LOG( SG_CLIPPER, SG_BULK, "0 = " << startx << ", " << starty );

            for ( j = 1; j < count - 1; ++j ) {
                in >> x;
                in >> y;
                OSRef = Point3D( x, y, -9999.0 );
                p = OSGB36ToWGS84(OSRef);

                poly.add_node( i, p );
                nodes.unique_add( p );
                SG_LOG( SG_CLIPPER, SG_BULK, j << " = " << x << ", " << y );
            }

            in >> lastx;
            in >> lasty;

            if ( (fabs(startx - lastx) < SG_EPSILON) &&
                 (fabs(starty - lasty) < SG_EPSILON) ) {
                // last point same as first, discard
            } else {
                OSRef = Point3D( lastx, lasty, -9999.0 );
                p = OSGB36ToWGS84(OSRef);

                poly.add_node( i, p );
                nodes.unique_add( p );
                SG_LOG( SG_CLIPPER, SG_BULK, count - 1 << " = " << lastx << ", " << lasty );
            }
        }

        // TODO : Make like OGR
        int area = (int)poly_type;
        string material = get_area_name( area );
        add_poly(area, poly, material);
        // END TODO

        in >> skipcomment;
    }

    return true;
}

// load all 2d polygons from the specified load disk directories and
// clip against each other to resolve any overlaps
int TGConstruct::LoadLandclassPolys( void ) {
    int i;

    string base = bucket.gen_base_path();
    string poly_path;
    int count = 0;

    polys_in.clear();

    // load 2D polygons from all directories provided
    for ( i = 0; i < (int)load_dirs.size(); ++i ) {
        poly_path = work_base + "/" + load_dirs[i] + '/' + base;

        string tile_str = bucket.gen_index_str();
        simgear::Dir d(poly_path);
        if (!d.exists()) {
            SG_LOG(SG_GENERAL, SG_DEBUG, "directory not found: " << poly_path);
            continue;
        }

        simgear::PathList files = d.children(simgear::Dir::TYPE_FILE);
        SG_LOG( SG_CLIPPER, SG_ALERT, files.size() << " Polys in " << d.path() );

        BOOST_FOREACH(const SGPath& p, files) {
            if (p.file_base() != tile_str) {
                continue;
            }

            string lext = p.complete_lower_extension();
            if ((lext == "arr") || (lext == "arr.gz") || (lext == "btg.gz") ||
                (lext == "fit") || (lext == "fit.gz") || (lext == "ind"))
            {
                // skipped!
            } else if (lext == "osgb36") {
                SG_LOG(SG_GENERAL, SG_ALERT, " Loading osgb36 poly definition file " << p.file());
                load_osgb36_poly( p.str() );
                ++count;
            } else {
                load_poly( p.str() );
                SG_LOG(SG_GENERAL, SG_ALERT, " Loaded " << p.file());
                ++count;
            }
        } // of directory file children
    }
    SG_LOG(SG_GENERAL, SG_ALERT, " Total polys used for this tile: " << count );
    return count;
}

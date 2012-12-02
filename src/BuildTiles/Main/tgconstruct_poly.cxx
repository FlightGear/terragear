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

#include "tgconstruct.hxx"

using std::string;

static unsigned int cur_poly_id = 0;

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

    tgPolygon   poly;
    tgTexParams tp;

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
        string material = get_area_name( area );


        // Generate a new Shape for the poly
        tgPolygon   poly;
        SGGeod      p;

        for (k=0; k<num_polys;k++) {
            poly.Erase();

            if ( with_tp ) {
                in >> x;
                in >> y;
                tp.ref    = SGGeod::fromDeg(x,y);

                in >> tp.width;
                in >> tp.length;
                in >> tp.heading;
                in >> tp.minu;
                in >> tp.maxu;
                in >> tp.minv;
                in >> tp.maxv;
                poly.SetTexParams( tp );
            }

            in >> contours;

            poly.Erase();
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

                p = SGGeod::fromDegM(startx+nudge, starty+nudge, startz );
                poly.AddNode( i, p );

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

                    p = SGGeod::fromDegM( x+nudge, y+nudge, z );
                    poly.AddNode( i, p );

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
                    p = SGGeod::fromDegM( lastx+nudge, lasty+nudge, lastz );
                    poly.AddNode( i, p );

                    if ( poly3d ) {
                        nodes.unique_add_fixed_elevation( p );
                    } else {
                        nodes.unique_add( p );
                    }
                }
            }

            poly = tgPolygon::Snap( poly, gSnap );
            poly = tgPolygon::RemoveDups( poly );
            poly.SetMaterial( material );

            if ( with_tp ) {
                poly.SetTexMethod( TG_TEX_BY_TPS_CLIPU, -1, 0, 1, 0 );
            } else {
                poly.SetTexMethod( TG_TEX_BY_GEODE, bucket.get_center_lat() );
            }

            in >> skipcomment;

            poly.SetId( cur_poly_id++ );
            polys_in.add_poly( area, poly );

            if ( IsDebugShape( poly.GetId() ) ) {
                tgPolygon::ToShapefile( poly, ds_name, "loaded", "" );
            }
        }
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
        SG_LOG( SG_CLIPPER, SG_DEBUG, files.size() << " Polys in " << d.path() );

        BOOST_FOREACH(const SGPath& p, files) {
            if (p.file_base() != tile_str) {
                continue;
            }

            string lext = p.complete_lower_extension();
            if ((lext == "arr") || (lext == "arr.gz") || (lext == "btg.gz") ||
                (lext == "fit") || (lext == "fit.gz") || (lext == "ind"))
            {
                // skipped!
            } else {
                load_poly( p.str() );
                SG_LOG(SG_GENERAL, SG_DEBUG, " Loaded " << p.file());
                ++count;
            }
        } // of directory file children
    }
    SG_LOG(SG_GENERAL, SG_ALERT, " Total polys used for this tile: " << count );
    return count;
}

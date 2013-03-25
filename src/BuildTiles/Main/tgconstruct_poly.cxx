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

#include <terragear/tg_shapefile.hxx>

#include "tgconstruct.hxx"

using std::string;

static unsigned int cur_poly_id = 0;

// load all 2d polygons from the specified load disk directories and
// clip against each other to resolve any overlaps
int TGConstruct::LoadLandclassPolys( void ) {
    int i;

    string base = bucket.gen_base_path();
    string poly_path;
    int    total_polys_read = 0;

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
                int area;
                std::string material;
                gzFile fp = gzopen( p.c_str(), "rb" );
                unsigned int count;

                sgReadUInt( fp, &count );
                SG_LOG( SG_GENERAL, SG_DEBUG, " Load " << count << " polys from " << p.realpath() );

                for ( unsigned int i=0; i<count; i++ ) {
                    tgPolygon poly;
                    poly.LoadFromGzFile( fp );
                    area     = area_defs.get_area_priority( poly.GetFlag() );
                    material = area_defs.get_area_name( area );

                    poly.SetMaterial( material );
                    poly.SetId( cur_poly_id++ );

                    if ( poly.Contours() ) {
                        polys_in.add_poly( area, poly );
                        total_polys_read++;

                        // add the nodes
                        for (unsigned int j=0; j<poly.Contours(); j++) {
                            for (unsigned int k=0; k<poly.ContourSize(j); k++) {
                                SGGeod const& node  = poly.GetNode( j, k );

                                if ( poly.GetPreserve3D() ) {
                                    nodes.unique_add_fixed_elevation( node );
                                } else {
                                    nodes.unique_add( node );
                                }
                            }
                        }

                        if (IsDebugShape( poly.GetId() )) {
                            char layer[32];
                            sprintf(layer, "loaded_%d", poly.GetId() );

                            tgShapefile::FromPolygon( poly, ds_name, layer, material.c_str() );
                        }
                    }
                }

                gzclose( fp );
                SG_LOG(SG_GENERAL, SG_DEBUG, " Loaded " << p.file());
            }
        } // of directory file children
    }

    SG_LOG(SG_GENERAL, SG_ALERT, " Total polys read in this tile: " <<  total_polys_read );

    return total_polys_read;
}

// tg_areas.cxx -- Class toSimnplify dealing with shape heiarchy:
//                    landclass contains each area (layer) of a tile
//                    Each area is a list of shapes
//                    A shape has 1 or more segments
//                    (when the shape represents line data)
//                    And the segment is a superpoly, containing
//                      - a polygon, triangulation, point normals, face normals, etc.
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
// $Id: construct.hxx,v 1.13 2004-11-19 22:25:49 curt Exp $

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/io/lowlevel.hxx>

#include "tg_areas.hxx"
#include "tg_polygon.hxx"
#include "tg_shapefile.hxx"

void tgAreas::clear(void)
{
    for (unsigned int i=0; i<polys.size(); i++) {
        polys[i].clear();
    }
    // keep the number of arrays intact - it's constant throughout construct
    polys.clear();
}

// TODO for tile merging, areas need to know the area defs file.  bring in from tgconstruct
#if 0
void tgAreas::SyncNodes( TGNodes& nodes )
{
    for (unsigned int area=0; area<polys.size(); area++) {
        //bool isRoad = area_defs.is_road_area( area );
        for (unsigned int p=0; p<polys[area].size(); p++ ) {
            tgPolygonSet& poly = polys[area][p];
        
            for (unsigned int con=0; con < poly.Contours(); con++) {
                for (unsigned int n = 0; n < poly.ContourSize( con ); n++) {
                    // ensure we have all nodes...
                    SGGeod node = poly.GetNode( con, n );
                    nodes.unique_add( node );
                    poly.SetNode( con, n, node );
                }
            }
        }
    }
    
    nodes.init_spacial_query();    
}
#endif

#if 0
void tgAreas::LoadFromGzFile(gzFile& fp)
{
    int i, j, num_layers, num_polys;

    // Load all landclass shapes
    sgReadInt( fp, &num_layers );

    polys.clear();
    for (i=0; i<num_layers; i++) {
        tgpolygon_list lc;
        sgReadInt( fp, &num_polys );

        lc.clear();
        for (j=0; j<num_polys; j++) {
            tgPolygon poly;
            poly.LoadFromGzFile( fp );
            lc.push_back( poly );
        }
        polys.push_back(lc);
    }
}

std::ostream& operator<< ( std::ostream& out, const tgAreas& lc )
{
    unsigned int count;
    tgPolygon poly;

    // Save all landclass shapes
    for (unsigned int i=0; i<lc.polys.size(); i++) {
        count = lc.polys[i].size();
        out << count << "\n";
        for (unsigned int j=0; j<count; j++) {
            out << lc.polys[i][j] << " ";
        }
        out << "\n";
    }

    return out;
}

void tgAreas::SaveToGzFile(gzFile& fp)
{
    int i, j, num_layers, num_polys;
    tgPolygon shape;

    // Save all landclass shapes
    num_layers = polys.size();
    sgWriteInt( fp, num_layers );
    for (i=0; i<num_layers; i++) {
        num_polys = polys[i].size();
        sgWriteInt( fp, num_polys );

        for (j=0; j<num_polys; j++) {
            polys[i][j].SaveToGzFile( fp );
        }
    }
}

void tgAreas::ToShapefile( const std::string& datasource )
{
    for (unsigned int area=0; area<polys.size(); area++) {
        //for each area, write a polygon list        
        tgShapefile::FromPolygonList( polys[area], false, false, datasource, area_names[area], "area" );
    }
}
#endif

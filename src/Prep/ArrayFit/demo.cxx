// demo.cxx
//
// Loads a .arr file (chopped intermediate form of DEM) and leverages
// portions of gts to impliment the terrain simplification algorithm
// in Michael Garlands paper located here:
//
//   http://graphics.cs.uiuc.edu/~garland/software/terra.html
//
// Essentially start with two triangles forming the bounding surface.
// Then add the point that has the greatest error.  Retriangulate.
// Recalcuate errors for each remaining point, add the one with the
// greatest error.  Lather, rinse, repeat.
//
// The resulting fitted set of nodes is written out to a file so the
// main tile builder can later load these nodes and incorporate them
// into the tile surface.
//
// Written by Curtis Olson, started March 2003.
//
// Copyright (C) 2003  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id: demo.cxx,v 1.2 2004-11-19 22:25:51 curt Exp $


#include <iostream>

#include <simgear/math/sg_types.hxx>

#include <gts.h>

SG_USING_STD(cin);
SG_USING_STD(cout);
SG_USING_STD(endl);


// transform point to lat/lon degree coordinates and append to list
static void add_point( point_list &list, Point3D p ) {
    Point3D tp( p.x() / 3600.0, p.y() / 3600.0, p.z() );
    list.push_back( tp );
}


static void pick_first_face( GtsFace *f, GtsFace **first ) {
    if ( *first == NULL )
        *first = f;
}


// if p lies inside plane (in terms of x,y position) return the
// distance from the point to the triangle in the z direction.
// Otherwise return 0.
double calc_error( GtsTriangle *t, GtsPoint *p ) {
    if ( gts_point_is_in_triangle( p, t ) == GTS_OUT ) {
        // point outside triangle, bail
        return 0;
    }

    double a, b, c, d;
    gts_triangle_normal( t, &a, &b, &c );
    GtsVertex *v1, *v2, *v3;
    gts_triangle_vertices( t, &v1, &v2, &v3 );
    GtsPoint *v = (GtsPoint *)v1;
    d = a * v->x + b * v->y + c * v->z;

    // cout << "p = " << Point3D( p->x, p->y, p->z ) << endl;
    // cout << "coeff = " << Point3D( a, b, c ) << endl;
    if ( c < 0.00000001 ) {
        cout << "Really small C coefficient" << endl;
        exit(-1);
    }

    double e = ( d - a * p->x - b * p->y ) / c;

    return fabs( e - p->z );
}


int main( int argc, char **argv ) {
    // option defaults
    int min_nodes = 50;
    int max_nodes = 6000;
    double error_threshold = 0.1;
    bool verbose = false;

    cout << "Minimum nodes = " << min_nodes << endl;
    cout << "Maximum nodes = " << max_nodes << endl;
    cout << "Error Threshold = " << error_threshold << endl;
    cout << endl;

    // Load the DEM data and make a list of points
    point_list pending;
    pending.clear();

    double x, y, z;

    while ( true ) {
        cin >> x >> y >> z;
        if ( x == 9999 && y == 9999 && z == 9999 ) {
            break;
        } else {
            pending.push_back( Point3D(x, y, z) );
        }
    }

    // Create the empty fitted list (this is the list we are working
    // so hard to create.) :-)
    point_list fitted;
    fitted.clear();

    // Make the corner vertices (enclosing exactly the DEM coverage area)
    add_point( fitted, pending[0] );
    GtsVertex *v1 = gts_vertex_new( gts_vertex_class(),
                                    pending[0].x(),
                                    pending[0].y(),
                                    pending[0].z() );

    add_point( fitted, pending[1] );
    GtsVertex *v2 = gts_vertex_new( gts_vertex_class(),
                                    pending[1].x(),
                                    pending[1].y(),
                                    pending[1].z() );

    add_point( fitted, pending[2] );
    GtsVertex *v3 = gts_vertex_new( gts_vertex_class(),
                                    pending[2].x(),
                                    pending[2].y(),
                                    pending[2].z() );

    add_point( fitted, pending[3] );
    GtsVertex *v4 = gts_vertex_new( gts_vertex_class(),
                                    pending[3].x(),
                                    pending[3].y(),
                                    pending[3].z() );

    GSList *list = NULL;
    list = g_slist_prepend( list, v1 );
    list = g_slist_prepend( list, v2 );
    list = g_slist_prepend( list, v3 );
    list = g_slist_prepend( list, v4 );

    // make a triangle the completely encloses the 4 corners of our
    // terrain data
    GtsTriangle *t = gts_triangle_enclosing( gts_triangle_class(),
                                             list, 2.0 );

    // Make the (empty) surface
    GtsSurface *surface = gts_surface_new( gts_surface_class(),
                                           gts_face_class(),
                                           gts_edge_class(),
                                           gts_vertex_class() );

    // add the enclosing surface
    gts_surface_add_face( surface, gts_face_new(gts_face_class(),
                                                t->e1, t->e2, t->e3) );

    // Add the four corners
    GtsVertex *result;
    result = gts_delaunay_add_vertex( surface, v1, NULL );
    result = gts_delaunay_add_vertex( surface, v2, NULL );
    result = gts_delaunay_add_vertex( surface, v3, NULL );
    result = gts_delaunay_add_vertex( surface, v4, NULL );

    if ( verbose ) {
        gts_surface_print_stats( surface, stdout );
    }

    // add the remaining points incrementally (from the pending list)

    bool done = false;
    int count = 4;
    GtsPoint *p = gts_point_new( gts_point_class(), 0, 0, 0 );

    while ( !done ) {
        // iterate through all the surface faces

        if ( verbose ) {
            gts_surface_print_stats( surface, stdout );
        }
        cout << "points left = " << pending.size()
             << " points added = " << count
             << " fitted list size = " << fitted.size() << endl;
        GtsFace *first = NULL;
        GtsFace *guess = NULL;
        GtsFace *found = NULL;
        gts_surface_foreach_face( surface, (GtsFunc)pick_first_face, &first );

        double max_error = 0;
        point_list_iterator mark = pending.end();

        // iterate through all remaining points
        point_list_iterator current = pending.begin();
        const_point_list_iterator last = pending.end();
        for ( ; current != last; ++current ) {
            // cout << *current << endl;
            gts_point_set( p, current->x(), current->y(),
                           current->z() );

            guess = gts_point_locate( p, surface, guess );

            double error = calc_error( (GtsTriangle *)guess, p );
            if ( error > max_error ) {
                max_error = error;
                mark = current;
                found = guess;
            }
        }

        // check stop conditions
        if ( (max_error < error_threshold && (int)fitted.size() >= min_nodes) ||
             (int)fitted.size() >= max_nodes )
        {
            // we are done
            done = true;
        } else {
            // add the next point and keep going
            cout << "adding " << *mark << " ("
                 << max_error << ")" << endl;
            add_point( fitted, *mark );
            GtsVertex *v = gts_vertex_new( gts_vertex_class(),
                                           mark->x(),
                                           mark->y(),
                                           mark->z() );
            GtsVertex *result = gts_delaunay_add_vertex( surface, v, guess );
            if ( result != NULL ) {
                cout << "  error adding vertex! " << *mark << endl;
            } else {
                ++count;
            }
            pending.erase( mark );

            // GtsFace *f = gts_delaunay_check( surface );
            // if ( f == NULL ) {
            //     cout << "valid delauney triangulation" << endl;
            // } else {
            //     cout << "NOT VALID DELAUNEY TRIANGULATION" << endl;
            // }
        }

        if ( verbose ) {
            FILE *fp = fopen( "surface.gts", "w" );
            gts_surface_write( surface, fp );
            fclose(fp);
        }

        if ( verbose ) {
            cout << endl;
        }
    }

    if ( verbose ) {
        FILE *fp = fopen( "surface.gts", "w" );
        gts_surface_write( surface, fp );
        fclose(fp);
    }

    return 0;
}

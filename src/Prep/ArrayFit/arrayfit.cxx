// arrayfit.cxx
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
// Copyright (C) 2003  Curtis L. Olson  - curt@flightgear.org
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
// $Id$


#include <simgear/bucket/newbucket.hxx>
#include <simgear/misc/sg_path.hxx>

#include <gts.h>

#include <Array/array.hxx>

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


static void usage( char *prog ) {
    cout << "Usage: " << prog << " [ --options ]" << endl;
    cout << "\t--input=file.arr" << endl;
    cout << "\t--minnodes=50" << endl;
    cout << "\t--maxnodes=600" << endl;
    cout << "\t--maxerror=50" << endl;
    cout << "\t--verbose" << endl;
    cout << endl;
    cout << "Algorithm will produce at least 50 fitted nodes, but no" << endl;
    cout << "more than 600.  Within that range, the algorithm will stop"<< endl;
    cout << "if the maximum elevation error for any remaining point" << endl;
    cout << "drops below 50 meters." << endl;
    cout << endl;
    cout << "Increasing the maxnodes value and/or decreasing maxerror" << endl;
    cout << "will produce a better surface approximation." << endl;
    cout << endl;
    cout << "The input file must be a .arr file such as that produced" << endl;
    cout << "by demchop or hgtchop utils." << endl;
    cout << endl;
    cout << "The output file is called .fit and is simply a list of" << endl;
    cout << "from the resulting fitted surface nodes.  The user of the" << endl;
    cout << ".fit file will need to triangulate the surface." << endl;
    exit(-1);
}


int main( int argc, char **argv ) {
    // option defaults
    SGPath infile;
    int min_nodes = 50;
    int max_nodes = 600;
    double error_threshold = 50.0;
    bool verbose = false;

    // Parse command line arguments
    unsigned int i;
    for ( i = 1; i < argc; ++i ) {
        string arg = argv[i];

        if ( arg.find("--input=") == 0 ) {
            infile.set( arg.substr(8) );
        } else if ( arg.find("--minnodes=") == 0 ) {
            min_nodes = atoi( arg.substr(11).c_str() );
        } else if ( arg.find("--maxnodes=") == 0 ) {
            max_nodes = atoi( arg.substr(11).c_str() );
        } else if ( arg.find("--maxerror=") == 0 ) {
            error_threshold = atof( arg.substr(11).c_str() );
        } else if ( arg.find("--verbose") == 0 ) {
            verbose = true;
        } else {
            usage( argv[0] );
        }
    }

    if ( ! infile.str().length() ) {
        usage( argv[0] );
    }

    SGPath outfile = infile;
    outfile.concat( ".fit.gz" );

    cout << "Input file = " << infile.str() << endl;
    cout << "Outfile file = " << outfile.str() << endl;
    cout << "Minimum nodes = " << min_nodes << endl;
    cout << "Maximum nodes = " << max_nodes << endl;
    cout << "Error Threshold = " << error_threshold << endl;
    cout << endl;

    SGBucket b(0.0, 0.0);       // build a dummy bucket
    TGArray a( infile.str() );
    a.parse( b );

    // Load the DEM data and make a list of points
    point_list pending;
    pending.clear();

    double x, y, z;
    double basex = a.get_originx();
    double basey = a.get_originy();
    double dx = a.get_col_step();
    double dy = a.get_row_step();

    for ( i = 0; i < a.get_cols(); ++i ) {
        for ( int j = 0; j < a.get_rows(); ++j ) {
            if ( (i == 0 && j == 0) ||
                 (i == a.get_cols() - 1 && j == 0 ) ||
                 (i == a.get_cols() - 1 && j == a.get_rows() - 1 ) ||
                 (i == 0 && j == a.get_rows() - 1 ) )
            {
                // skip corners since they will be added seperately
            } else {
                x = basex + i * dx;
                y = basey + j * dy;
                z = a.get_array_elev( i, j );
                if ( z > -9000 ) {
                    pending.push_back( Point3D(x, y, z) );
                } else {
                    // just ignore voids, better just not include
                    // them, rather than making a stupid guess at a
                    // value
                }
            }
        }
    }

    // Create the empty fitted list (this is the list we are working
    // so hard to create.) :-)
    point_list fitted;
    fitted.clear();
    double_list errors;
    errors.clear();

    // Make the corner vertices (enclosing exactly the DEM coverage area)
    x = basex;
    y = basey;
    z = a.altitude_from_grid( x, y );
    cout << "adding = " << Point3D( x, y, z) << endl;
    add_point( fitted, Point3D( x, y, z) );
    errors.push_back( 13000.0 );
    GtsVertex *v1 = gts_vertex_new( gts_vertex_class(), x, y, z );

    x = basex + dx * (a.get_cols() - 1);
    y = basey;
    z = a.altitude_from_grid( x, y );
    cout << "adding = " << Point3D( x, y, z) << endl;
    add_point( fitted, Point3D( x, y, z) );
    errors.push_back( 13000.0 );
    GtsVertex *v2 = gts_vertex_new( gts_vertex_class(), x, y, z );

    x = basex + dx * (a.get_cols() - 1);
    y = basey + dy * (a.get_rows() - 1);
    z = a.altitude_from_grid( x, y );
    cout << "adding = " << Point3D( x, y, z) << endl;
    add_point( fitted, Point3D( x, y, z) );
    errors.push_back( 13000.0 );
    GtsVertex *v3 = gts_vertex_new( gts_vertex_class(), x, y, z );

    x = basex;
    y = basey + dy * (a.get_rows() - 1);
    z = a.altitude_from_grid( x, y );
    cout << "adding = " << Point3D( x, y, z) << endl;
    add_point( fitted, Point3D( x, y, z) );
    errors.push_back( 13000.0 );
    GtsVertex *v4 = gts_vertex_new( gts_vertex_class(), x, y, z );

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

    while ( !done && pending.size() > 0 ) {
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
            errors.push_back( max_error );
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

    gzFile gzfp;
    if ( (gzfp = gzopen( outfile.c_str(), "wb9" )) == NULL ) {
        cout << "ERROR:  cannot open " << outfile.str() << " for writing!"
             << endl;
        exit(-1);
    }

    gzprintf( gzfp, "%d\n", fitted.size() );
    for ( i = 0; i < fitted.size(); ++i ) {
        gzprintf( gzfp, "%.15f %.15f %.2f %.1f\n",
                  fitted[i].x(), fitted[i].y(), fitted[i].z(), errors[i] );
    }
    gzclose( gzfp );

#if 0

    // Make the corner vertices
    GtsVertex *v1 = gts_vertex_new( gts_vertex_class(), 0, 0, 0 );
    GtsVertex *v2 = gts_vertex_new( gts_vertex_class(), 10, 0, 5 );
    GtsVertex *v3 = gts_vertex_new( gts_vertex_class(), 10, 10, 10 );
    GtsVertex *v4 = gts_vertex_new( gts_vertex_class(), 0, 10, 5 );

    // Make the 5 edges
    GtsEdge *e1 = gts_edge_new( gts_edge_class(), v1, v2 );
    GtsEdge *e2 = gts_edge_new( gts_edge_class(), v2, v3 );
    GtsEdge *e3 = gts_edge_new( gts_edge_class(), v3, v4 );
    GtsEdge *e4 = gts_edge_new( gts_edge_class(), v4, v1 );
    GtsEdge *e5 = gts_edge_new( gts_edge_class(), v2, v4 );

    // Make the two faces
    GtsFace *f1 = gts_face_new( gts_face_class(), e1, e5, e4 );
    GtsFace *f2 = gts_face_new( gts_face_class(), e2, e3, e5 );

    // Make the (empty) surface
    GtsSurface *surface = gts_surface_new( gts_surface_class(),
                                           gts_face_class(),
                                           gts_edge_class(),
                                           gts_vertex_class() );

    // Add the two faces
    gts_surface_add_face( surface, f1 );
    gts_surface_add_face( surface, f2 );

    // Add some vertices
    for ( int j = 0; j <= 10; ++j ) {
        for ( int i = 0; i <= 10; ++i ) {
            GtsVertex *v = gts_vertex_new( gts_vertex_class(), i, j, (i - j) );
            GtsVertex *result = gts_delaunay_add_vertex (surface, v, NULL);
            if ( result != NULL ) {
                cout << "  error adding vertex! " << i << " " << j << endl;
            } 
        }
    }
#endif

    return 0;
}

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
//  Outputs to a file that can be visualized with gtsview (available
//  from http://gts.sf.net)

#include <simgear/bucket/newbucket.hxx>

#include <gts.h>

#include "array.hxx"

SG_USING_STD(cout);
SG_USING_STD(endl);

static void pick_first_face( GtsFace *f, GtsFace **first ) {
    if ( *first == NULL )
        *first = f;
}


static GtsPoint *global_pt;

static void find_enclosing_face( GtsFace *f, GtsFace **first ) {
    if ( gts_point_is_in_triangle( global_pt, (GtsTriangle *)f ) != GTS_OUT ) {
        *first = f;
    }
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
    bool verbose = false;
    double error_threshold = 20;

    if ( argc != 2 ) {
	cout << "Usage: " << argv[0] << " work_dir" << endl;
	exit(-1);
    }

    string work_dir = argv[1];
   
    double lon, lat;
    lon = -146.248360; lat = 61.133950;  // PAVD (Valdez, AK)
    lon = -110.664244; lat = 33.352890;  // P13
    lon = -122.374843; lat = 37.619002;  // KSFO

    SGBucket b( lon, lat );
    string base = b.gen_base_path();
    string path = work_dir + "/" + base;

    string arrayfile = path + "/" + b.gen_index_str() + ".arr";
    cout << "arrayfile = " << arrayfile << endl;
    
    TGArray a(arrayfile);
    a.parse( b );

    // Old fit
    cout << "Old fit(200) = " << a.fit(200) << endl;
    cout << "Old fit(100) = " << a.fit(100) << endl;
    cout << "Old fit(50) = " << a.fit(50) << endl;
    cout << "Old fit(25) = " << a.fit(25) << endl;

    // Test libgts (gnu triangulation library functions)

    // Load the DEM data and make a list of points
    point_list pending;
    pending.clear();

    double x, y, z;
    double basex = a.get_originx();
    double basey = a.get_originy();
    double dx = a.get_col_step();
    double dy = a.get_row_step();

    for ( int i = 0; i < a.get_cols(); ++i ) {
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
                pending.push_back( Point3D(x, y, z) );
            }
        }
    }

    // Make an (empty) surface

    // Make the corner vertices (enclosing exactly the DEM coverage area)
    x = basex;
    y = basey;
    z = a.interpolate_altitude( x, y );
    cout << "adding = " << Point3D( x, y, z) << endl;
    GtsVertex *v1 = gts_vertex_new( gts_vertex_class(), x, y, z );

    x = basex + dx * (a.get_cols() - 1);
    y = basey;
    z = a.interpolate_altitude( x, y );
    cout << "adding = " << Point3D( x, y, z) << endl;
    GtsVertex *v2 = gts_vertex_new( gts_vertex_class(), x, y, z );

    x = basex + dx * (a.get_cols() - 1);
    y = basey + dy * (a.get_rows() - 1);
    z = a.interpolate_altitude( x, y );
    cout << "adding = " << Point3D( x, y, z) << endl;
    GtsVertex *v3 = gts_vertex_new( gts_vertex_class(), x, y, z );

    x = basex;
    y = basey + dy * (a.get_rows() - 1);
    z = a.interpolate_altitude( x, y );
    cout << "adding = " << Point3D( x, y, z) << endl;
    GtsVertex *v4 = gts_vertex_new( gts_vertex_class(), x, y, z );

    GSList *list = NULL;
    list = g_slist_prepend( list, v1 );
    list = g_slist_prepend( list, v2 );
    list = g_slist_prepend( list, v3 );
    list = g_slist_prepend( list, v4 );

    // make a triangle the completely encloses the 4 corners of our
    // DEM
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

    gts_surface_print_stats( surface, stdout );

    // add points incrementally from the pending list

    bool done = false;
    int count = 4;
    GtsPoint *p = gts_point_new( gts_point_class(), 0, 0, 0 );
    global_pt = gts_point_new( gts_point_class(), 0, 0, 0 );

    while ( !done ) {
        // iterate through all the surface faces

        if ( verbose ) {
            gts_surface_print_stats( surface, stdout );
        }
        cout << "points left = " << pending.size()
             << " points added = " << count << endl;
        GtsFace *first = NULL;
        GtsFace *guess = NULL;
        GtsFace *found = NULL;
        gts_surface_foreach_face( surface, (GtsFunc)pick_first_face, &first );

        double max_error = 0;
        point_list_iterator mark = NULL;

        // iterate through all remaining points
        point_list_iterator current = pending.begin();
        const_point_list_iterator last = pending.end();
        for ( ; current != last; ++current ) {
            // cout << *current << endl;
            gts_point_set( p, current->x(), current->y(),
                           current->z() );
            gts_point_set( global_pt, current->x(), current->y(),
                           current->z() );

            guess = gts_point_locate( p, surface, guess );
            // gts_surface_foreach_face( surface, (GtsFunc)find_enclosing_face,
            //                           &guess );

            double error = calc_error( (GtsTriangle *)guess, p );
            if ( error > max_error ) {
                max_error = error;
                mark = current;
                found = guess;
            }
        }

        if ( max_error > error_threshold ) {
            cout << "adding " << *mark << " ("
                 << max_error << ")" << endl;
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
        } else {
            done = true;
        }

        FILE *fp = fopen( "surface.gts", "w" );
        gts_surface_write( surface, fp );
        fclose(fp);

        cout << endl;
    }

    FILE *fp = fopen( "surface.gts", "w" );
    gts_surface_write( surface, fp );
    fclose(fp);

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

#include "osgb36.hxx"
#include "uk.hxx"

#include "osgbtc.hxx"


// traverse the specified fan/strip/list of vertices and attempt to
// calculate "none stretching" texture coordinates
point_list UK_calc_tex_coords( const SGBucket& b, const point_list& geod_nodes,
			    const int_list& fan, double scale )
{
    // find min/max of fan
    Point3D t, tmin, tmax;
    Point3D OSGridRef;
    bool first = true;

    int i;

    for ( i = 0; i < (int)fan.size(); ++i ) {
	
	OSGridRef = WGS84ToOSGB36(geod_nodes[ fan[i] ]);
	
	t = UK_basic_tex_coord( OSGridRef );
	// cout << "basic_tex_coord = " << t << endl;

	if ( first ) {
	    tmin = tmax = t;
	    first = false;
	} else {
	    if ( t.x() < tmin.x() ) {
		tmin.setx( t.x() );
	    }
	    if ( t.y() < tmin.y() ) {
		tmin.sety( t.y() );
	    }
	    if ( t.x() > tmax.x() ) {
		tmax.setx( t.x() );
	    }
	    if ( t.y() > tmax.y() ) {
		tmax.sety( t.y() );
	    }
	}

//	cout << "tmin.x = " << tmin.x() << " tmax.x = " << tmax.x() << " tmin.y = " << tmin.y() << " tmax.y = " << tmax.y() << '\n';
    }

    double dx = fabs( tmax.x() - tmin.x() );
    double dy = fabs( tmax.y() - tmin.y() );
//    cout << "dx = " << dx << " dy = " << dy << endl;

    bool do_shift = false;
    // Point3D mod_shift;
    if ( (dx > HALF_MAX_TEX_COORD) || (dy > HALF_MAX_TEX_COORD) ) {
	// structure is too big, we'll just have to shift it so that
	// tmin = (0,0).  This messes up subsequent texture scaling,
	// but is the best we can do.
//	cout << "SHIFTING" << endl;
	do_shift = true;
	if ( tmin.x() < 0 ) {
	    tmin.setx( (double)( (int)tmin.x() - 1 ) );
	} else {
	    tmin.setx( (int)tmin.x() );
	}
	if ( tmin.y() < 0 ) {
	    tmin.sety( (double)( (int)tmin.y() - 1 ) );
	} else {
	    tmin.sety( (int)tmin.y() );
	}
//	cout << "found tmin = " << tmin << endl;
    } else {
	if ( tmin.x() < 0 ) {
	    tmin.setx( ( (int)(tmin.x() / HALF_MAX_TEX_COORD) - 1 )
		       * HALF_MAX_TEX_COORD );
	} else {
	    tmin.setx( ( (int)(tmin.x() / HALF_MAX_TEX_COORD) )
		       * HALF_MAX_TEX_COORD );
	}
	if ( tmin.y() < 0 ) {
	    tmin.sety( ( (int)(tmin.y() / HALF_MAX_TEX_COORD) - 1 )
		       * HALF_MAX_TEX_COORD );
	} else {
	    tmin.sety( ( (int)(tmin.y() / HALF_MAX_TEX_COORD) )
		       * HALF_MAX_TEX_COORD );
	}
#if 0
	// structure is small enough ... we can mod it so we can
	// properly scale the texture coordinates later.
	// cout << "MODDING" << endl;
	double x1 = fmod(tmin.x(), MAX_TEX_COORD);
	while ( x1 < 0 ) { x1 += MAX_TEX_COORD; }

	double y1 = fmod(tmin.y(), MAX_TEX_COORD);
	while ( y1 < 0 ) { y1 += MAX_TEX_COORD; }

	double x2 = fmod(tmax.x(), MAX_TEX_COORD);
	while ( x2 < 0 ) { x2 += MAX_TEX_COORD; }

	double y2 = fmod(tmax.y(), MAX_TEX_COORD);
	while ( y2 < 0 ) { y2 += MAX_TEX_COORD; }
	
	// At this point we know that the object is < 16 wide in
	// texture coordinate space.  If the modulo of the tmin is >
	// the mod of the tmax at this point, then we know that the
	// starting tex coordinate for the tmax > 16 so we can shift
	// everything down by 16 and get it within the 0-32 range.

	if ( x1 > x2 ) {
	    mod_shift.setx( HALF_MAX_TEX_COORD );
	} else {
	    mod_shift.setx( 0.0 );
	}

	if ( y1 > y2 ) {
	    mod_shift.sety( HALF_MAX_TEX_COORD );
	} else {
	    mod_shift.sety( 0.0 );
	}
#endif
	// cout << "mod_shift = " << mod_shift << endl;
    }

    // generate tex_list
    Point3D adjusted_t;
    point_list tex;
    tex.clear();
    for ( i = 0; i < (int)fan.size(); ++i ) {
	
	OSGridRef = WGS84ToOSGB36(geod_nodes[ fan[i] ]);

	t = UK_basic_tex_coord(OSGridRef);
//	cout << "second t = " << t << endl;

	// if ( do_shift ) {
	adjusted_t = t - tmin;
//        cout << "adjusted_t = " << adjusted_t << '\n';
#if 0
	} else {
	    adjusted_t.setx( fmod(t.x() + mod_shift.x(), MAX_TEX_COORD) );
	    while ( adjusted_t.x() < 0 ) { 
		adjusted_t.setx( adjusted_t.x() + MAX_TEX_COORD );
	    }
	    adjusted_t.sety( fmod(t.y() + mod_shift.y(), MAX_TEX_COORD) );
	    while ( adjusted_t.y() < 0 ) {
		adjusted_t.sety( adjusted_t.y() + MAX_TEX_COORD );
	    }
	    // cout << "adjusted_t " << adjusted_t << endl;
	}
#endif
	if ( adjusted_t.x() < SG_EPSILON ) {
	    adjusted_t.setx( 0.0 );
	}
	if ( adjusted_t.y() < SG_EPSILON ) {
	    adjusted_t.sety( 0.0 );
	}
	adjusted_t.setz( 0.0 );
	//cout << "adjusted_t after check for SG_EPSILON = " << adjusted_t << endl;
	
	tex.push_back( adjusted_t );
    }
//    	char dcl_pause2;
//	cin >> dcl_pause2;

    return tex;
}

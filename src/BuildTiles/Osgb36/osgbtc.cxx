#include "osgb36.hxx"
#include "uk.hxx"

#include "osgbtc.hxx"


// traverse the specified fan/strip/list of vertices and attempt to
// calculate "none stretching" texture coordinates
point_list UK_calc_tex_coords( const SGBucket& b, const point_list& geod_nodes,
			    const int_list& fan, double scale )
{
    // cout << "calculating texture coordinates for a specific fan of size = "
    //      << fan.size() << endl;

    // calculate perimeter based on center of this degree (not center
    // of bucket)
/*    double clat = (int)b.get_center_lat();
    if ( clat > 0 ) {
	clat = (int)clat + 0.5;
    } else {
	clat = (int)clat - 0.5;
    }
*/
//    double clat_rad = clat * DEG_TO_RAD;
//    double cos_lat = cos( clat_rad );
//    double local_radius = cos_lat * EQUATORIAL_RADIUS_M;
//    double local_perimeter = 2.0 * local_radius * FG_PI;
//    double degree_width = local_perimeter / 360.0;

    // cout << "clat = " << clat << endl;
    // cout << "clat (radians) = " << clat_rad << endl;
    // cout << "cos(lat) = " << cos_lat << endl;
    // cout << "local_radius = " << local_radius << endl;
    // cout << "local_perimeter = " << local_perimeter << endl;
    // cout << "degree_width = " << degree_width << endl;

//    double perimeter = 2.0 * EQUATORIAL_RADIUS_M * FG_PI;
//    double degree_height = perimeter / 360.0;
    // cout << "degree_height = " << degree_height << endl;

    // find min/max of fan
    Point3D tmin, tmax, WGS84LatLon, t;
    Point3D WGS84xyz, OSGB36xyz, OSGB36LatLon;
    Point3D OSGridRef;
    bool first = true;

    int i;

    for ( i = 0; i < (int)fan.size(); ++i ) {
	WGS84LatLon = geod_nodes[ fan[i] ];
//	cout << "point WGS84LatLon = " << WGS84LatLon << endl;
	WGS84xyz = ConvertGRS80PolarToCartesian(WGS84LatLon);
//	cout << "WGS84XYZ = " << WGS84XYZ.x << ", " << WGS84XYZ.y << ", " << WGS84XYZ.z << '\n';
	OSGB36xyz = ConvertWGS84ToOSGB36(WGS84xyz);
//	cout << "OSXYZ = " << OSXYZ.x << ", " << OSXYZ.y << ", " << OSXYZ.z << '\n';
	OSGB36LatLon = ConvertAiry1830CartesianToPolar(OSGB36xyz);
//	cout << "OSGB36LatLon = " << OSGB36LatLon.x() << ", " << OSGB36LatLon.y() << ", " << OSGB36LatLon.z() << '\n';
	OSGridRef = ConvertLatLonToEastingsNorthings(OSGB36LatLon);
//	cout << "OS Eastings and Northings = " << OSGridRef << '\n';



	t = UK_basic_tex_coord( OSGridRef );
	// cout << "basic_tex_coord = " << t << endl;

//	cout << "p = " << p << '\n';
//	cout << "t = " << t << '\n';
//	cout << "Degree width = " << degree_width << '\n';
//	cout << "Degree height = " << degree_height << '\n';
//	char dcl_pause;
//	cin >> dcl_pause;

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
	WGS84LatLon = geod_nodes[ fan[i] ];
//        cout << "About to do second run through tex coords\n";
//	cout << "point WGS84LatLon = " << WGS84LatLon << endl;
	WGS84xyz = ConvertGRS80PolarToCartesian(WGS84LatLon);
//	cout << "WGS84XYZ = " << WGS84XYZ.x << ", " << WGS84XYZ.y << ", " << WGS84XYZ.z << '\n';
	OSGB36xyz = ConvertWGS84ToOSGB36(WGS84xyz);
//	cout << "OSXYZ = " << OSXYZ.x << ", " << OSXYZ.y << ", " << OSXYZ.z << '\n';
	OSGB36LatLon = ConvertAiry1830CartesianToPolar(OSGB36xyz);
//	cout << "OSGB36LatLon = " << OSGB36LatLon.x() << ", " << OSGB36LatLon.y() << ", " << OSGB36LatLon.z() << '\n';
	OSGridRef = ConvertLatLonToEastingsNorthings(OSGB36LatLon);
//	cout << "OS Eastings and Northings = " << OSGridRef << '\n';

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
	cout << "adjusted_t after check for SG_EPSILON = " << adjusted_t << endl;
	
	tex.push_back( adjusted_t );
    }
//    	char dcl_pause2;
//	cin >> dcl_pause2;

    return tex;
}

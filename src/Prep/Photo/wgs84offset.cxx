// calc wgs84 offset given starting lon/lat/alt, radial and distance

#include <stdlib.h>

#include <simgear/math/sg_geodesy.hxx>

SG_USING_STD(cout);
SG_USING_STD(endl);

int main( int argc, char **argv ) {

    if ( argc != 6 ) {
        cout << "bad usage ..." << endl;
    }

    double lon1 = atof( argv[1] );
    double lat1 = atof( argv[2] );
    double alt = atof( argv[3] );
    double az1 = atof( argv[4] );
    double s =   atof( argv[5] ) * .3048;

    /**
     * Given a starting position and an offset radial and distance,
     * calculate an ending positon on a wgs84 ellipsoid.
     * @param alt (in) meters
     * @param lat1 (in) degrees
     * @param lon1 (in) degrees
     * @param az1 (in) degrees
     * @param s (in) distance in meters
     * @param lat2 (out) degrees
     * @param lon2 (out) degrees
     * @param az2 (out) return course in degrees
     */

    double lat2, lon2, az2;

    geo_direct_wgs_84 ( alt, lat1, lon1, az1, s, &lat2, &lon2, &az2 );

    printf("pos = %.6f %6f\n", lon2, lat2 );
}

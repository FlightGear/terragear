// find (lat/lon) of each corner of an image

#include <simgear/compiler.h>

#include <stdlib.h>

#include <iostream>

#include <plib/sg.h>

using std::cout;
using std::endl;

#include <simgear/math/sg_geodesy.hxx>

int main( int argc, char **argv ) {

    if ( argc != 11 ) {
        cout << "Usage: " << argv[0]
             << "xres yres xrwy1(px) yrwy1(px) xrwy2(px) yrwy2(px) rwy_len(ft) rwy_angle(deg) rwy_cent_lon(deg) rwy_cent_lat(deg)" << endl;
        cout << "if you are at rwy1 looking at rwy2 you will be facing rwy_angle" << endl;
	exit(-1);
    }

    int xres = atoi( argv[1] );
    int yres = atoi( argv[2] );
    int xrwy1 = atoi( argv[3] );
    int yrwy1 = atoi( argv[4] );
    int xrwy2 = atoi( argv[5] );
    int yrwy2 = atoi( argv[6] );
    double len = atof( argv[7] );
    double true_angle = atof( argv[8] ) * SGD_DEGREES_TO_RADIANS;
    double clon = atof( argv[9] );
    double clat = atof( argv[10] );

    // find runway center in pixel space
    double xcen = (double)(xrwy1 + xrwy2) / 2.0f;
    double ycen = (double)(yrwy1 + yrwy2) / 2.0f;
    cout << "runway center (pixels) = " << xcen << ", " << ycen << endl;

    // find runway distance in pixels
    int dx = xrwy2 - xrwy1;
    int dy = yrwy2 - yrwy1;
    cout << "dx = " << dx << " dy = " << dy << endl;
    double distp = sqrt( double(dx*dx + dy*dy) );
    cout << "runway distance (pixels) = " << distp << endl;

    // find pixel resolution
    double pixel_to_feet = len / distp;
    double feet_to_pixel = distp / len;
    cout << "feet per pixel = " << pixel_to_feet << endl;
    cout << "pixel per feet = " << feet_to_pixel << endl;

    // find runway angle in image space
    double img_angle = atan2( double(dy), double(dx) );
    cout << "runway angle in image space = "
         << img_angle * SGD_RADIANS_TO_DEGREES << endl;
    // angle offset
    double angle_offset = true_angle - img_angle;
    cout << "angle offset = " << angle_offset * SGD_RADIANS_TO_DEGREES << endl;

    cout << "true runway hdg = "
         << (img_angle + angle_offset) * SGD_RADIANS_TO_DEGREES << endl;

    // setup image corner coordinates
    int corner[4][2];
    corner[0][0] = 0;    corner[0][1] = 0;
    corner[1][0] = xres; corner[1][1] = 0;
    corner[2][0] = xres; corner[2][1] = yres;
    corner[3][0] = 0;    corner[3][1] = yres;

    // calculate corresponding corner angles from runway center
    double angles_image[4];     // angles in image space
    double angles_true[4];      // true / real world angles of corners
    double dist_px[4];          // distance in pixels from rwy center to corner
    double dist_m[4];           // distance in meters
    for ( int i = 0; i < 4; ++i ) {
        double xdist = corner[i][0] - xcen;
        double ydist = corner[i][1] - ycen;
        angles_image[i] = atan2( ydist, xdist );
        angles_true[i] = angles_image[i] + angle_offset;
        dist_px[i] = sqrt( xdist*xdist + ydist*ydist );
        dist_m[i] = dist_px[i] * pixel_to_feet * 0.3048;
        cout << "corner " << i << " (" << corner[i][0] << "," << corner[i][1]
             << ")" << endl;
        cout << "  angle to (image space ) = "
             << angles_image[i] * SGD_RADIANS_TO_DEGREES << endl;
        cout << "  angle to (real world ) = "
             << angles_true[i] * SGD_RADIANS_TO_DEGREES << endl;
        cout << "  distance to = " << dist_px[i] << " (px)  "
             << dist_m[i] << " (m)" << endl;

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
        double alt = 0;
        geo_direct_wgs_84 ( alt, clat, clon,
                            angles_true[i] * SGD_RADIANS_TO_DEGREES, dist_m[i],
                            &lat2, &lon2, &az2 );
        printf("  pos = %.6f %6f\n", lon2, lat2 );
    }
    
}

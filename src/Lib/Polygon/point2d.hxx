/**
 * Simple 2d point class where members can be accessed as x, dist, or lon
 * and y, theta, or lat
 */

#ifndef TG_POINT2D_HXX
#define TG_POINT2D_HXX

class point2d {
public:
union {
    double x;
    double dist;
    double lon;
};
union {
    double y;
    double theta;
    double lat;
};
};

#endif // TG_POINT2D_HXX

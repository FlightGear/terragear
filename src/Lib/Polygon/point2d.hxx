/**
 * Simple 2d point class where members can be accessed as x, dist, or lon
 * and y, theta, or lat
 */

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


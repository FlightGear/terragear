#include <simgear/compiler.h>
#include <Geometry/point3d.hxx>

#include <iostream>

#include "osgb36.hxx"

using std::cout;

static void Usage() {
    cout << "Usage is testosgb36 <lon> <lat>\n";
    exit(-1);
}

//Test harness for WGS84 to OSGB36 conversion.
int main(int argc, char** argv)
{
    if(argc != 3) Usage();
    
    cout << "Test OSGB36\n";
    
    Point3D p1;
    p1.setlon(atof(argv[1]));
    p1.setlat(atof(argv[2]));
    p1.setelev(0.0);
    
    cout << "WGS84 position is " << p1 << '\n';
    
    Point3D p2 = WGS84ToOSGB36(p1);
    
    cout << "OS coords are " << p2 << '\n';
    
    p1 = OSGB36ToWGS84(p2);
    
    cout << "Conversion back to WGS84 gives " << p1 << '\n';
    
    return(0);
}

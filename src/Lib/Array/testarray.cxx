#include <simgear/bucket/newbucket.hxx>

#include "array.hxx"

SG_USING_STD(cout);
SG_USING_STD(endl);

int main( int argc, char **argv ) {
    double lon, lat;

    if ( argc != 2 ) {
	cout << "Usage: " << argv[0] << " work_dir" << endl;
	exit(-1);
    }

    string work_dir = argv[1];
   
    lon = -146.248360; lat = 61.133950;  // PAVD (Valdez, AK)
    lon = -110.664244; lat = 33.352890;  // P13

    SGBucket b( lon, lat );
    string base = b.gen_base_path();
    string path = work_dir + "/" + base;

    string arrayfile = path + "/" + b.gen_index_str() + ".dem";
    cout << "arrayfile = " << arrayfile << endl;
    
    TGArray a(arrayfile);
    a.parse( b );

    lon *= 3600;
    lat *= 3600;
    cout << "  " << a.interpolate_altitude(lon, lat) << endl;

    a.fit( 100 );

    return 0;
}

#include <simgear/math/point3d.hxx>

#include "osgb36.hxx"

//Test harness for various WGS84 and OSGB36 conversion functions
int main()
{
	cout << "Test OSGB36\n";
/*    Point3D initial_position;
    Point3D final_position;

    initial_position.setlat(52.0 + ((39.0 + 27.2531/60.0)/60.0));
    initial_position.setlon(1.0 + ((43.0 + 4.5177/60.0)/60.0));
    initial_position.setelev(0.0);

    final_position = ConvertLatLonToEastingsNorthings(initial_position);

    cout << "Eastings = " << final_position.x() << '\n';
    cout << "Northings = " << final_position.y() << '\n';

    Point3D check_position;

    check_position = ConvertEastingsNorthingsToLatLon(final_position);

    cout << "Lat = " << check_position.lat() << '\n';
    cout << "Lon = " << check_position.lon() << '\n';
    cout << "Elipsiodal height = " << check_position.elev() << '\n';

    check_position.setelev(24.7);

    Point3D cartesian_position = ConvertAiry1830PolarToCartesian(check_position);

    cout << "X = " << cartesian_position.x() << '\n';
    cout << "Y = " << cartesian_position.y() << '\n';
    cout << "Z = " << cartesian_position.z() << '\n';

    cout << "\nTesting ConvertAiry1830CartesianToPolar.....\n";

    Point3D XYZPosition;

    XYZPosition.setx(3874938.849);
    XYZPosition.sety(116218.624);
    XYZPosition.setz(5047168.208);

    Point3D LatLonPosition = ConvertAiry1830CartesianToPolar(XYZPosition);

    cout << "Lat = " << LatLonPosition.lat() << '\n';
    cout << "Lon = " << LatLonPosition.lon() << '\n';
    cout << "Elipsiodal height = " << LatLonPosition.elev() << '\n';

    cout << "\nNow attempting to convert WGS84 lat & lon to OS grid reference .....\n";

    Point3D WGS84LatLon;
    Point3D WGS84XYZ;
    Point3D OSXYZ;
    Point3D OSLatLon;
    Point3D OSGridRef;

    WGS84LatLon.setlat(52.0);
    WGS84LatLon.setlon(-2.0);
    WGS84LatLon.setelev(0.0);

    WGS84XYZ = ConvertGRS80PolarToCartesian(WGS84LatLon);
    cout << "WGS84XYZ = " << WGS84XYZ.x() << ", " << WGS84XYZ.y() << ", " << WGS84XYZ.z() << '\n';
    OSXYZ = ConvertWGS84ToOSGB36(WGS84XYZ);
    cout << "OSXYZ = " << OSXYZ.x() << ", " << OSXYZ.y() << ", " << OSXYZ.z() << '\n';
    OSLatLon = ConvertAiry1830CartesianToPolar(OSXYZ);
    cout << "OSLatLon = " << OSLatLon.lat() << ", " << OSLatLon.lon() << ", " << OSLatLon.elev() << '\n';
    OSGridRef = ConvertLatLonToEastingsNorthings(OSLatLon);

    cout << "OS Grid Reference = " << OSGridRef.x() << ", " << OSGridRef.y() << "\n\n";
*/
    return(0);
}

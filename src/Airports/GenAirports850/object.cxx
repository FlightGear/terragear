#include <simgear/math/sg_geodesy.hxx>
#include <simgear/debug/logstream.hxx>
#include "object.hxx"

LightingObj::LightingObj( char* definition )
{
    sscanf(definition, "%lf %lf %d %lf %lf %s", &lat, &lon, &type, &heading, &glideslope, &assoc_rw);

    SG_LOG(SG_GENERAL, SG_DEBUG, "Read lighting object: (" << lon << "," << lat << ") heading: " << heading << " type: " << type  );
}

void LightingObj::BuildBtg( int alt_m, superpoly_list* lights )
{
    point_list lightobj; lightobj.clear();
    point_list normals; normals.clear();

    Point3D ref;
    double lon2 = 0, lat2 = 0, r;
    double left_hdg = heading - 90.0;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }

    ref.setlat( lat );
    ref.setlon( lon );

    if ( glideslope < 0.5 ) {
        glideslope = 3.0;
    }

    // Calculate the normal once for all object parts.
    // SG takes care of the angle.

    // calculate a second point in the object heading direction
    geo_direct_wgs_84 ( lat, lon, heading,
                        100, &lat2, &lon2, &r);

    Point3D end1, end2;

    end1.setlat( lat2 );
    end1.setlon( lon2 );

    end2.setlat( lat);
    end2.setlon( lon);

    Point3D cart1 = sgGeodToCart( end1 * SG_DEGREES_TO_RADIANS );
    Point3D cart2 = sgGeodToCart( end2 * SG_DEGREES_TO_RADIANS );

    Point3D up = cart1;
    double length = up.distance3D( Point3D(0.0) );
    up = up / length;

    Point3D obj_vec = cart2 - cart1;

    // angle up specified amount
    length = obj_vec.distance3D( Point3D(0.0) );
    double up_length = length * tan( glideslope * SG_DEGREES_TO_RADIANS);
    Point3D light_vec = obj_vec + (up * up_length);

    length = light_vec.distance3D( Point3D(0.0) );
    Point3D normal = light_vec / length;

    SG_LOG(SG_GENERAL, SG_DEBUG, "obj_normal = " << normal);


    // We know our normal, now create the lights
    Point3D pt1;

    if (type == 2)
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Generating PAPI 4L = " << assoc_rw);

        // unit1
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg,
                            -45 * SG_FEET_TO_METER, &lat2, &lon2, &r );
        pt1 = Point3D( lon2, lat2, 0.0 );
        lightobj.push_back( pt1 );
        normals.push_back( normal );

        // unit2
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg,
                            30 * SG_FEET_TO_METER, &lat2, &lon2, &r );
        pt1 = Point3D( lon2, lat2, 0.0 );
        lightobj.push_back( pt1 );
        normals.push_back( normal );

        // unit3
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg,
                            30 * SG_FEET_TO_METER, &lat2, &lon2, &r );
        pt1 = Point3D( lon2, lat2, 0.0 );
        lightobj.push_back( pt1 );
        normals.push_back( normal );

        // unit4
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg,
                            30 * SG_FEET_TO_METER, &lat2, &lon2, &r );
        pt1 = Point3D( lon2, lat2, 0.0 );
        lightobj.push_back( pt1 );
        normals.push_back( normal );
    }

    if (type == 3)
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Generating PAPI 4R = " << assoc_rw);

        // unit1
        geo_direct_wgs_84 ( alt_m, ref.lat(), ref.lon(), left_hdg,
                            45 * SG_FEET_TO_METER, &lat2, &lon2, &r );
        pt1 = Point3D( lon2, lat2, 0.0 );
        lightobj.push_back( pt1 );
        normals.push_back( normal );

        // unit2
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg,
                            -30 * SG_FEET_TO_METER, &lat2, &lon2, &r );
        pt1 = Point3D( lon2, lat2, 0.0 );
        lightobj.push_back( pt1 );
        normals.push_back( normal );

        // unit3
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg,
                            -30 * SG_FEET_TO_METER, &lat2, &lon2, &r );
        pt1 = Point3D( lon2, lat2, 0.0 );
        lightobj.push_back( pt1 );
        normals.push_back( normal );

        // unit4
        geo_direct_wgs_84 ( alt_m, pt1.lat(), pt1.lon(), left_hdg,
                            -30 * SG_FEET_TO_METER, &lat2, &lon2, &r );
        pt1 = Point3D( lon2, lat2, 0.0 );
        lightobj.push_back( pt1 );
        normals.push_back( normal );
    }

    TGPolygon lights_poly; lights_poly.erase();
    TGPolygon normals_poly; normals_poly.erase();
    lights_poly.add_contour( lightobj, false );
    normals_poly.add_contour( normals, false );

    TGSuperPoly result;
    result.set_poly( lights_poly );
    result.set_normals( normals_poly );
    result.set_material( "RWY_VASI_LIGHTS" );

    lights->push_back( result);
}

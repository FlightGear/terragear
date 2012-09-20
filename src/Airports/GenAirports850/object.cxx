#include <simgear/math/sg_geodesy.hxx>
#include <simgear/debug/logstream.hxx>
#include "object.hxx"

LightingObj::LightingObj( char* definition )
{
    sscanf(definition, "%lf %lf %d %lf %lf %s", &lat, &lon, &type, &heading, &glideslope, &assoc_rw);

    SG_LOG(SG_GENERAL, SG_DEBUG, "Read lighting object: (" << lon << "," << lat << ") heading: " << heading << " type: " << type  );
}

void LightingObj::BuildBtg( superpoly_list* lights )
{
    point_list lightobj; lightobj.clear();
    point_list normals; normals.clear();

    double left_hdg = heading - 90.0;
    if ( left_hdg < 0 ) { left_hdg += 360.0; }

    Point3D ref(lon, lat, 0.0);

    // Calculate the normal direction once for all objects.
    // SimGear takes care of the glideslope angle calculation from the normal

    // calculate a second point in the object heading direction
    double lon2 = 0, lat2 = 0, r;
    geo_direct_wgs_84 ( lat, lon, heading,
                        10, &lat2, &lon2, &r);

    Point3D cart1 = sgGeodToCart( Point3D(lon2, lat2, 0.0) * SG_DEGREES_TO_RADIANS );
    Point3D cart2 = sgGeodToCart( ref * SG_DEGREES_TO_RADIANS );

    Point3D normal = cart2 - cart1;
    double length = normal.distance3D( Point3D(0.0) );
    normal = normal / length;

    // We know our normal, now create the lights
    Point3D pt1;

    if (type == 1)
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Generating VASI = " << assoc_rw);

        // VASI coordinates describe the center between the two bars.
        // Space between the bars is 200m

        // Go to downwind bar
        geo_direct_wgs_84 ( ref.lat(), ref.lon(), heading,
                            -100, &lat2, &lon2, &r );
        pt1 = Point3D( lon2, lat2, 0.0 );

        // unit1
        geo_direct_wgs_84 ( pt1.lat(), pt1.lon(), left_hdg,
                            -5, &lat2, &lon2, &r );
        pt1 = Point3D( lon2, lat2, 0.0 );
        lightobj.push_back( pt1 );
        normals.push_back( normal );

        // unit2+3
        for (int i = 0; i < 2; ++i)
        {
            geo_direct_wgs_84 ( pt1.lat(), pt1.lon(), left_hdg,
                                5, &lat2, &lon2, &r );
            pt1 = Point3D( lon2, lat2, 0.0 );
            lightobj.push_back( pt1 );
            normals.push_back( normal );
        }

        // Go to upwind bar
        geo_direct_wgs_84 ( ref.lat(), ref.lon(), heading,
                            100, &lat2, &lon2, &r );
        pt1 = Point3D( lon2, lat2, 0.0 );

        // unit4
        geo_direct_wgs_84 ( pt1.lat(), pt1.lon(), left_hdg,
                            -5, &lat2, &lon2, &r );
        pt1 = Point3D( lon2, lat2, 0.0 );
        lightobj.push_back( pt1 );
        normals.push_back( normal );

        // unit5+6
        for (int i = 0; i < 2; ++i)
        {
            geo_direct_wgs_84 ( pt1.lat(), pt1.lon(), left_hdg,
                                5, &lat2, &lon2, &r );
            pt1 = Point3D( lon2, lat2, 0.0 );
            lightobj.push_back( pt1 );
            normals.push_back( normal );
        }
    }
    else if (type == 2)
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Generating PAPI 4L = " << assoc_rw);

        // unit1
        geo_direct_wgs_84 ( ref.lat(), ref.lon(), left_hdg,
                            -12, &lat2, &lon2, &r );
        pt1 = Point3D( lon2, lat2, 0.0 );
        lightobj.push_back( pt1 );
        normals.push_back( normal );

        // unit2-4
        for (int i = 0; i < 3; ++i)
        {
            geo_direct_wgs_84 ( pt1.lat(), pt1.lon(), left_hdg,
                                8, &lat2, &lon2, &r );
            pt1 = Point3D( lon2, lat2, 0.0 );
            lightobj.push_back( pt1 );
            normals.push_back( normal );
        }
    }
    else if (type == 3)
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Generating PAPI 4R = " << assoc_rw);

        // unit1
        geo_direct_wgs_84 ( ref.lat(), ref.lon(), left_hdg,
                            12, &lat2, &lon2, &r );
        pt1 = Point3D( lon2, lat2, 0.0 );
        lightobj.push_back( pt1 );
        normals.push_back( normal );

        // unit2-4
        for (int i = 0; i < 3; ++i)
        {
            geo_direct_wgs_84 ( pt1.lat(), pt1.lon(), left_hdg,
                                -8, &lat2, &lon2, &r );
            pt1 = Point3D( lon2, lat2, 0.0 );
            lightobj.push_back( pt1 );
            normals.push_back( normal );
        }
    }
    else if (type == 4)
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Space Shuttle PAPI is deprecated. Use the normal PAPI and set the glideslope accordingly");
    }
    else if (type == 5)
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Generating tri-colour VASI = " << assoc_rw);

        // only one light here
        lightobj.push_back( ref );
        normals.push_back( normal );
    }
    else
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Unknown lighting object (PAPI/VASI...) code: " << type);
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

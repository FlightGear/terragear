#include <simgear/math/SGMathFwd.hxx>
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

    SGGeod ref = SGGeod::fromDeg(lon, lat);

    // Calculate the light normal with the help of a second point
    // in the object heading direction.
    // SimGear takes care of the glideslope angle calculation from the normal
    SGVec3d cart1 = SGVec3d::fromGeod(SGGeodesy::direct( ref, heading, 10));
    SGVec3d cart2 = SGVec3d::fromGeod(ref);

    Point3D normal = Point3D::fromSGVec3(normalize(cart2 - cart1));

    // We know our normal, now create the lights
    SGGeod pt1;

    if (type == 1)
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Generating VASI = " << assoc_rw);

        // VASI coordinates describe the center between the two bars.
        // Space between the bars is 200m

        // Go to downwind bar
        pt1 = SGGeodesy::direct( ref, heading, -100 );

        // unit1
        pt1 = SGGeodesy::direct( pt1, left_hdg, -5 );
        lightobj.push_back( Point3D::fromSGGeod(pt1) );
        normals.push_back( normal );

        // unit2+3
        for (int i = 0; i < 2; ++i)
        {
            pt1 = SGGeodesy::direct( pt1, left_hdg, 5 );
            lightobj.push_back( Point3D::fromSGGeod(pt1) );
            normals.push_back( normal );
        }

        // Go to upwind bar
        pt1 = SGGeodesy::direct( ref, heading, 100 );

        // unit4
        pt1 = SGGeodesy::direct( pt1, left_hdg, -5 );
        lightobj.push_back( Point3D::fromSGGeod(pt1) );
        normals.push_back( normal );

        // unit5+6
        for (int i = 0; i < 2; ++i)
        {
            pt1 = SGGeodesy::direct( pt1, left_hdg,5 );
            lightobj.push_back( Point3D::fromSGGeod(pt1) );
            normals.push_back( normal );
        }
    }
    else if (type == 2)
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Generating PAPI 4L = " << assoc_rw);

        // unit1
        pt1 = SGGeodesy::direct( ref, left_hdg, -12 );
        lightobj.push_back( Point3D::fromSGGeod(pt1) );
        normals.push_back( normal );

        // unit2-4
        for (int i = 0; i < 3; ++i)
        {
            pt1 = SGGeodesy::direct( pt1, left_hdg, 8 );
            lightobj.push_back( Point3D::fromSGGeod(pt1) );
            normals.push_back( normal );
        }
    }
    else if (type == 3)
    {
        SG_LOG(SG_GENERAL, SG_DEBUG, "Generating PAPI 4R = " << assoc_rw);

        // unit1
        pt1 = SGGeodesy::direct( ref, left_hdg, 12 );
        lightobj.push_back( Point3D::fromSGGeod(pt1) );
        normals.push_back( normal );

        // unit2-4
        for (int i = 0; i < 3; ++i)
        {
            pt1 = SGGeodesy::direct( pt1, left_hdg, -8 );
            lightobj.push_back( Point3D::fromSGGeod(pt1) );
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
        lightobj.push_back(  Point3D::fromSGGeod(ref) );
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

#include <simgear/math/SGMathFwd.hxx>
#include <simgear/debug/logstream.hxx>

#include "object.hxx"
#include "debug.hxx"

LightingObj::LightingObj( char* definition )
{
    std::istringstream ss(definition);
    ss  >> lat
        >> lon
        >> type
        >> heading
        >> glideslope
        >> assoc_rw;

    TG_LOG(SG_GENERAL, SG_DEBUG, "Read lighting object: (" << lon << "," << lat << ") heading: " << heading << " type: " << type  );
}

void LightingObj::BuildBtg( tglightcontour_list& lights )
{
    tgLightContour light_contour;
    light_contour.SetType( "RWY_VASI_LIGHTS" );

    double left_hdg = SGMiscd::normalizePeriodic(0, 360, heading - 90.0 );
    SGGeod ref      = SGGeod::fromDeg(lon, lat);

    // Calculate the light normal with the help of a second point
    // in the object heading direction.
    // SimGear takes care of the glideslope angle calculation from the normal
    SGVec3f cart1 = SGVec3f::fromGeod(SGGeodesy::direct( ref, heading, 10));
    SGVec3f cart2 = SGVec3f::fromGeod(ref);
    SGVec3f normal = normalize(cart2 - cart1);

    // We know our normal, now create the lights
    SGGeod pt1;

    if (type == 1)
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Generating VASI = " << assoc_rw);

        // VASI coordinates describe the center between the two bars.
        // Space between the bars is 200m

        // Go to downwind bar
        pt1 = SGGeodesy::direct( ref, heading, -100 );

        // unit1
        pt1 = SGGeodesy::direct( pt1, left_hdg, -5 );
        light_contour.AddLight( pt1, normal );

        // unit2+3
        for (int i = 0; i < 2; ++i)
        {
            pt1 = SGGeodesy::direct( pt1, left_hdg, 5 );
            light_contour.AddLight( pt1, normal );
        }

        // Go to upwind bar
        pt1 = SGGeodesy::direct( ref, heading, 100 );

        // unit4
        pt1 = SGGeodesy::direct( pt1, left_hdg, -5 );
        light_contour.AddLight( pt1, normal );

        // unit5+6
        for (int i = 0; i < 2; ++i)
        {
            pt1 = SGGeodesy::direct( pt1, left_hdg,5 );
            light_contour.AddLight( pt1, normal );
        }
    }
    else if (type == 2)
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Generating PAPI 4L = " << assoc_rw);

        // unit1
        pt1 = SGGeodesy::direct( ref, left_hdg, -12 );
        light_contour.AddLight( pt1, normal );

        // unit2-4
        for (int i = 0; i < 3; ++i)
        {
            pt1 = SGGeodesy::direct( pt1, left_hdg, 8 );
            light_contour.AddLight( pt1, normal );
        }
    }
    else if (type == 3)
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Generating PAPI 4R = " << assoc_rw);

        // unit1
        pt1 = SGGeodesy::direct( ref, left_hdg, 12 );
        light_contour.AddLight( pt1, normal );

        // unit2-4
        for (int i = 0; i < 3; ++i)
        {
            pt1 = SGGeodesy::direct( pt1, left_hdg, -8 );
            light_contour.AddLight( pt1, normal );
        }
    }
    else if (type == 4)
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Space Shuttle PAPI is deprecated. Use the normal PAPI and set the glideslope accordingly");
        return;
    }
    else if (type == 5)
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Generating tri-colour VASI = " << assoc_rw);

        // only one light here
        light_contour.AddLight( ref, normal );
    }
    else if (type == 6)
    {
        TG_LOG(SG_GENERAL, SG_DEBUG, "Generating runway guard light = " << assoc_rw);

        light_contour.SetType( "RWY_GUARD_LIGHTS" );

        // unit 1
        light_contour.AddLight( SGGeodesy::direct( ref, left_hdg, 0.2 ), normal );

        // unit 2
        light_contour.AddLight( SGGeodesy::direct( ref, left_hdg, -0.2 ), normal );
    }
    else
    {
        TG_LOG(SG_GENERAL, SG_ALERT, "Unknown lighting object (PAPI/VASI...) code: " << type);
        return;
    }

    lights.push_back( light_contour);
}

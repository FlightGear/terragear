#include <simgear/math/sg_geodesy.hxx>
#include <simgear/debug/logstream.hxx>
#include "linked_objects.hxx"

Windsock::Windsock( char* definition )
{
    sscanf(definition, "%lf %lf %d", &lat, &lon, &lit);

    SG_LOG(SG_GENERAL, SG_DEBUG, "Read Windsock: (" << lon << "," << lat << ") lit: " << lit  );
}

Beacon::Beacon( char* definition )
{
    sscanf(definition, "%lf %lf %d", &lat, &lon, &code);

    SG_LOG(SG_GENERAL, SG_DEBUG, "Read Beacon: (" << lon << "," << lat << ") code: " << code  );
}


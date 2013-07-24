#include <map>

#include "tg_polygon.hxx"

// for ogr-decode : generate a bunch of polygons, mapped by bucket id
typedef std::map<long int, tgpolygon_list> bucket_polys_map;
typedef bucket_polys_map::iterator bucket_polys_map_interator;

class tgChopper
{
public:
    tgChopper( const std::string& path ) {
        root_path = path;
    }

    void Add( const tgPolygon& poly, const std::string& type );
    void Save( bool DebugShapes );

private:
    long int GenerateIndex( std::string path );
    void ClipRow( const tgPolygon& subject, const double& center_lat, const std::string& type );
    tgPolygon Clip( const tgPolygon& subject, const std::string& type, SGBucket& b );
    void Chop( const tgPolygon& subject, const std::string& type );

    std::string      root_path;
    bucket_polys_map bp_map;
    SGMutex          lock;
};

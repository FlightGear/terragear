#include <map>

#include <simgear/bucket/newbucket.hxx>

#include "tg_polygon_set.hxx"

// for ogr-decode : generate a bunch of polygons, mapped by bucket id
typedef std::map<long int, tgPolygonSetList> bucket_polys_map;
typedef bucket_polys_map::iterator bucket_polys_map_interator;

class tgChopper
{
public:
    tgChopper( const std::string& path, long int bid = -1 ) {
        root_path = path;
        bucket_id = bid;
    }

    void Add( const tgPolygonSet& poly );
    void Save( bool DebugShapes );

private:
    void ClipRow( const tgPolygonSet& subject, const double& center_lat );
    void Clip( const tgPolygonSet& subject, SGBucket& b );
    void Chop( const tgPolygonSet& subject );

    long int         bucket_id;     // set if we only want to save a single bucket
    std::string      root_path;
    bucket_polys_map bp_map;
};

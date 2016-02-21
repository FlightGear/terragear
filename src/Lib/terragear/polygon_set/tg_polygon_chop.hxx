#include <map>

#include <simgear/threads/SGThread.hxx>
#include <simgear/timing/timestamp.hxx>
#include <simgear/bucket/newbucket.hxx>

#include <terragear/tg_dataset_protect.hxx>
#include "tg_polygon_set.hxx"

class tgChopperChunk
{
public:
    tgChopperChunk( const tgPolygonSet& subject ) {
        chunk = subject;
    }
    
    void setBuckets( const SGGeod& min, const SGGeod& max, bool checkBorders );
    
    void clip( long int bucket_id, std::string& rootPath, SGMutex* lock );
    
private:
    std::vector<SGBucket>   buckets;
    tgPolygonSet            chunk;
};

class tgChopper
{
public:
    tgChopper( const std::string& path, long int bid = -1 ) {
        root_path = path;
        bucket_id = bid;
    }

    void Add( const tgPolygonSet& poly );

private:
    void PreChop( const tgPolygonSet& subject, std::vector<tgChopperChunk>& chunks );

    long int         bucket_id;     // set if we only want to save a single bucket
    std::string      root_path;
    SGMutex          lock;
    tgDatasetAcess   dataset;
};
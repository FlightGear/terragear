#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

#include <simgear/debug/logstream.hxx>
#include <simgear/structure/exception.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/io/lowlevel.hxx>

#include "tg_polygon_chop.hxx"
#include "tg_shapefile.hxx"
#include "tg_rectangle.hxx"
#include "tg_misc.hxx"


// prechopping greatly increases performance.  really HUGE features can take 6 hours to 
// chop tile by tile.
// if I first chop the shape into 1x1 deg rectangles, we pay 15-20 minutes penaly up front, 
// but individual tile chops are really fast.  The same 6 hour task is reduced to about 20 minutes
// (1x1)  
// I also tried 2x2, the penalty is shorter, but the til chops are longer - resulting in 30 minutes
#define DO_PRECHOP      1

#define PRECHOP_CORRECTION      (0.0004)
#define CLIP_CORRECTION         (0.0002)

#define DEBUG_CHOPPER   0

void tgChopperChunk::setBuckets( const SGGeod& min, const SGGeod& max, bool checkBorders )
{
    if ( checkBorders ) {
        std::vector<SGBucket> bucks;
        sgGetBuckets( min, max, bucks );
    
        // simgear can give us too many buckets when requesting min/max on
        // tile boundaries due to floating point round off error
        // make sure each tile center is within our bounding box
        tgRectangle bb( min, max );

        for (unsigned int i=0; i<bucks.size(); i++) {
            if ( bb.isInside( bucks[i].get_center() ) ) {
                buckets.push_back( bucks[i] );
            }
        }
    } else {
        sgGetBuckets( min, max, buckets );
    }
}

void tgChopper::Add( const tgPolygonSet& subject )
{
    // bail out immediately if polygon is empty
    if ( subject.isEmpty() ) {
        SG_LOG( SG_GENERAL, SG_INFO, "tgChopper::Add - subject is empty" );
        return;
    } else {
        SG_LOG( SG_GENERAL, SG_DEBUG, "tgChopper Add - material is " << subject.getMeta().material );
    }
    
    // if the bounding box width or height > 1.0, pre chop into 1x1 pieces
    std::vector<tgChopperChunk> chunks;
    PreChop( subject, chunks);

    for ( unsigned int i=0; i < chunks.size(); i++ ) {
        chunks[i].clip(bucket_id, root_path, &lock);
    }
}

void tgChopper::PreChop( const tgPolygonSet& subject, std::vector<tgChopperChunk>& chunks )
{
    #define CHUNK_X     (1.0l)
    #define CHUNK_Y     (1.0l)
    
    CGAL::Bbox_2 bb   = subject.getBoundingBox();
    
    double startx, endx, starty, endy;
    double width, height;
    
    SG_LOG( SG_GENERAL, SG_DEBUG, "tgChopper PreCHop - BB is " << bb );
    
    if ( bb.xmin() < 0 && bb.xmax() > 0 ) {
        // the bounding box crosses either the Greenwich meridian, or the IDL
        // chop the whole thing
        startx = -180.0l;
        endx   =  180.0l;
        width  =  360.0l;
    } else {
        startx = std::floor( bb.xmin() );
        endx   = std::ceil( bb.xmax() );
        width  = endx-startx;
    }
    
    // chop shapes past the poles
    if ( bb.ymin() < -90.0l ) {
        starty = -90.0l;
    } else {
        starty = std::floor( bb.ymin() );
    }
    
    if ( bb.ymax() > 90.0l ) {
        endy = 90.0l;
    } else {
        endy = std::ceil( bb.ymax() );
    }
    height = endy-starty;
    
    chunks.clear();
    if ( width > CHUNK_X || height > CHUNK_Y ) {
        // break up the geometries, and add them to the queue
        // use exact match
        for ( double x=startx; x<endx; x+=CHUNK_X ) {
            for ( double y=starty; y<endy; y+=CHUNK_Y ) {
                char chunkname[256];
                
                sprintf(chunkname, "%0f_%0f", x, y );
                
                // we need two rectangles.  1 for the chunk, and one for calculating the buckets
                // for the chunk.  we want the chunk slightly larger than what we need for chopping,
                // but we don't want to include the partial buckets on the edges.
                double min_bucket_x = x, max_bucket_x = x + CHUNK_X;
                double min_bucket_y = y, max_bucket_y = y + CHUNK_Y;
             
                double min_cor_x = min_bucket_x - PRECHOP_CORRECTION, max_cor_x = max_bucket_x + PRECHOP_CORRECTION;
                double min_cor_y = min_bucket_y - PRECHOP_CORRECTION, max_cor_y = max_bucket_y + PRECHOP_CORRECTION;
                tgPolygonSet result;
                
                // create the clipping geometry for this piece
                cgalPoly_Point base_pts[4];
                
                base_pts[0] = cgalPoly_Point( min_cor_x, min_cor_y );
                base_pts[1] = cgalPoly_Point( max_cor_x, min_cor_y );
                base_pts[2] = cgalPoly_Point( max_cor_x, max_cor_y );
                base_pts[3] = cgalPoly_Point( min_cor_x, max_cor_y );
                
                cgalPoly_Polygon clip( base_pts, base_pts+4 );
                result.intersection2( subject, clip );
                
                if ( !result.isEmpty() ) {
                    result.getMeta().setDescription( chunkname );
                    
                    tgChopperChunk chunk( result );
                    
                    // add the correct buckets to chunk ( without correction )
                    SGGeod gMin = SGGeod::fromDeg( min_bucket_x, min_bucket_y );
                    SGGeod gMax = SGGeod::fromDeg( max_bucket_x, max_bucket_y );
                    
                    chunk.setBuckets ( gMin, gMax, true );
                    chunks.push_back( chunk );
                }
            }
        }
    } else {
        // process current geometry        
        tgPolygonSet result = subject;
        result.getMeta().setDescription( "no chunk" );
        
        tgChopperChunk chunk( result );

        SGGeod gMin = SGGeod::fromDeg( bb.xmin(), bb.ymin() );
        SGGeod gMax = SGGeod::fromDeg( bb.xmax(), bb.ymax() );
        
        chunk.setBuckets( gMin, gMax, false );
        chunks.push_back( chunk );
    }
}

void tgChopperChunk::clip( long int bucket_id, std::string& rootPath, std::mutex* lock )
{
    for ( unsigned int i=0; i<buckets.size(); i++ ) {
        cgalPoly_Point    base_pts[4];
        const std::string material = chunk.getMeta().material;
        SGGeod            pt;
        char              layer[256];
        tgPolygonSet      result;
    
        SGTimeStamp       chop_begin, chop_end, chop_time;
    
        static double     total_chop_time = 0;
        static unsigned long num_chops = 1;
    
        // set up clipping tile
        pt = buckets[i].get_corner( SG_BUCKET_SW );
        base_pts[0] = cgalPoly_Point( pt.getLongitudeDeg()-CLIP_CORRECTION, pt.getLatitudeDeg()-CLIP_CORRECTION );
        pt = buckets[i].get_corner( SG_BUCKET_SE );
        base_pts[1] = cgalPoly_Point( pt.getLongitudeDeg()+CLIP_CORRECTION, pt.getLatitudeDeg()-CLIP_CORRECTION );
        pt = buckets[i].get_corner( SG_BUCKET_NE );
        base_pts[2] = cgalPoly_Point( pt.getLongitudeDeg()+CLIP_CORRECTION, pt.getLatitudeDeg()+CLIP_CORRECTION );
        pt = buckets[i].get_corner( SG_BUCKET_NW );
        base_pts[3] = cgalPoly_Point( pt.getLongitudeDeg()-CLIP_CORRECTION, pt.getLatitudeDeg()+CLIP_CORRECTION );
        cgalPoly_Polygon base( base_pts, base_pts+4 );
    
#if DEBUG_CHOPPER
        static unsigned int curClip=1;
        char debugDatasetName[128];
    
        sprintf(debugDatasetName, "./Chopper/tile_%s_%s", b.gen_index_str().c_str(), material.c_str() );
    
        lock->lock();
        SGPath sgp( debugDatasetName );
        sgp.create_dir( 0755 );
        
        GDALDataset* poDS = tgPolygonSet::openDatasource(debugDatasetName);
    
        // open Point layer
        OGRLayer* poLayerSubject = tgPolygonSet::openLayer(poDS, wkbPolygon25D, tgPolygonSet::LF_DEBUG, "subject");
        OGRLayer* poLayerTile    = tgPolygonSet::openLayer(poDS, wkbPolygon25D, tgPolygonSet::LF_DEBUG, "tile");
        OGRLayer* poLayerResult  = tgPolygonSet::openLayer(poDS, wkbPolygon25D, tgPolygonSet::LF_DEBUG, "result");
    
        tgPolygonSet::toDebugShapefile( poLayerSubject, subject.getPs(), "subject" );
        tgPolygonSet::toDebugShapefile( poLayerTile, base, "tile" );        
#endif
    
        chop_begin.stamp();
        // new geometry is intersection of original geometry and tile
        result.intersection2( chunk, base );
        chop_end.stamp();
        chop_time = chop_end-chop_begin;
    
#if DEBUG_CHOPPER
        tgPolygonSet::toDebugShapefile( poLayerResult, result.getPs(), "result" );
    
        curClip++;
        GDALClose( poDS );
        lock->unlock();
#endif
    
        if ( !result.isEmpty() ) {
            //      if ( subject.GetPreserve3D() ) {
            //          result.InheritElevations( subject );
            //          result.SetPreserve3D( true );
            //      }
        
            long int cur_bucket = buckets[i].gen_index();
            if ( ( bucket_id < 0 ) || (cur_bucket == bucket_id ) ) {
                std::string path = rootPath + "/" + buckets[i].gen_base_path();
                std::string polyfile = path + "/" + buckets[i].gen_index_str();
            
                // lock mutex to simgear directory creation
                lock->lock();
                SGPath sgp( polyfile );
                sgp.create_dir( 0755 );
                //lock.unlock();
            
                // now get a per dataset lock
                //dataset.Request( cur_bucket );
            
                snprintf( layer, 256, "%s_%s", material.c_str(), result.getMeta().getMetaType().c_str() );
            
                // save chopped polygon to a Shapefile in layer named from material
                result.toShapefile( polyfile.c_str(), material.c_str() );
            
                // Release per dataset lock
                //dataset.Release( cur_bucket );
                lock->unlock();
            }
        }

        // dump debug...
        total_chop_time += chop_time.toMSecs();
        SG_LOG( SG_GENERAL, SG_DEBUG, "tgChopper Clip - avg chop time: " << total_chop_time/num_chops );
        num_chops++;        
    }
}

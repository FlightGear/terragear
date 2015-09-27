#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

#include <simgear/debug/logstream.hxx>
#include <simgear/structure/exception.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/io/lowlevel.hxx>

#include "tg_polygon_chop.hxx"
#include "tg_shapefile.hxx"
#include "tg_misc.hxx"

void tgChopper::Clip( const tgPolygonSet& subject, SGBucket& b )
{
    cgalPoly_Point base_pts[4];
    tgPolygonSet   result;
    SGGeod         pt;

    // set up clipping tile : and remember to add the nodes!
    pt = b.get_corner( SG_BUCKET_SW );
    base_pts[0] = cgalPoly_Point( pt.getLongitudeDeg(), pt.getLatitudeDeg() );

    pt = b.get_corner( SG_BUCKET_SE );    
    base_pts[1] = cgalPoly_Point( pt.getLongitudeDeg(), pt.getLatitudeDeg() );
    
    pt = b.get_corner( SG_BUCKET_NE );    
    base_pts[2] = cgalPoly_Point( pt.getLongitudeDeg(), pt.getLatitudeDeg() );
    
    pt = b.get_corner( SG_BUCKET_NW );
    base_pts[3] = cgalPoly_Point( pt.getLongitudeDeg(), pt.getLatitudeDeg() );

    cgalPoly_Polygon base( base_pts, base_pts+4 );

    result = subject.intersection( base );

    if ( !result.isEmpty() ) {
//      if ( subject.GetPreserve3D() ) {
//          result.InheritElevations( subject );
//          result.SetPreserve3D( true );
//      }
        
        long int cur_bucket = b.gen_index();
        SG_LOG( SG_GENERAL, SG_INFO, " - poly saved in bucket " << cur_bucket << " bucket_id is " << bucket_id );

        if ( ( bucket_id < 0 ) || (cur_bucket == bucket_id ) ) {
            bp_map[b.gen_index()].push_back( result );
        }
    }
}

// Pass in the center lat for clipping buckets from the row.  
// We can't rely on sgBucketOffset, as rounding error sometimes causes it to look like there are 2 rows 
// (the first being a sliver)
// This leads to using that poly as the subject - which leads to having no usable polygon for this row.
void tgChopper::ClipRow( const tgPolygonSet& subject, const double& center_lat )
{
    CGAL::Bbox_2 bb = subject.getBoundingBox();
    SGGeod       gMin = SGGeod::fromDeg( CGAL::to_double(bb.xmin()), CGAL::to_double(bb.ymin()) );
    SGGeod       gMax = SGGeod::fromDeg( CGAL::to_double(bb.xmax()), CGAL::to_double(bb.ymax()) );
    SGBucket     b_min( gMin );
    SGBucket     b_max( gMax );
    int          dx, dy;

    sgBucketDiff(b_min, b_max, &dx, &dy);
    SGBucket start = SGBucket(SGGeod::fromDeg( b_min.get_center_lon(), center_lat ));

    for ( int i = 0; i <= dx; ++i ) {
        SGBucket b_cur = start.sibling(i, 0);
        Clip( subject, b_cur );
    }
}

void tgChopper::Add( const tgPolygonSet& subject )
{
    // bail out immediately if polygon is empty
    if ( subject.isEmpty() ) {
        SG_LOG( SG_GENERAL, SG_INFO, "tgChopper::Add - subject is empty" );
        return;
    }

    CGAL::Bbox_2 bb = subject.getBoundingBox();

    // find buckets for min, and max points of convex hull.
    // note to self: self, you should think about checking for
    // polygons that span the date line
    SGGeod       gMin = SGGeod::fromDeg( CGAL::to_double(bb.xmin()), CGAL::to_double(bb.ymin()) );
    SGGeod       gMax = SGGeod::fromDeg( CGAL::to_double(bb.xmax()), CGAL::to_double(bb.ymax()) );
    SGBucket     b_min( gMin );
    SGBucket     b_max( gMax );
    SGBucket     b_cur;
    int          dx, dy;

    sgBucketDiff(b_min, b_max, &dx, &dy);
    SG_LOG( SG_GENERAL, SG_INFO, "  y_min = " << bb.ymin() << " y_max = " << bb.ymax() << " dx = " <<  dx << "  dy = " << dy );

    if ( (dx > 2880) || (dy > 1440) )
        throw sg_exception("something is really wrong in split_polygon()!!!!");

    if ( dy == 0 )
    {
        // We just have a single row - no need to intersect first
        SG_LOG( SG_GENERAL, SG_INFO, "   UN_CLIPPED row -  center lat is " << b_min.get_center_lat() );

        ClipRow( subject, b_min.get_center_lat() );
    }
    else
    {
        // Multiple rows - perform row intersection to reduce the number of bucket clips we need
        // since many shapes are narraw in some places, wide in others - bb will be at the widest part
        SG_LOG( SG_GENERAL, SG_INFO, "subject spans tile rows: bb is from lat " << bb.ymin() << " to " << bb.ymax() << " dy is " << dy );

        for ( int row = 0; row <= dy; row++ )
        {
            // Generate a clip rectangle for the whole row

            SGBucket     b_clip      = b_min.sibling(0, row);
            double       clip_bottom = b_clip.get_center_lat() - SG_HALF_BUCKET_SPAN;
            double       clip_top    = b_clip.get_center_lat() + SG_HALF_BUCKET_SPAN;
            cgalPoly_Point clip_pts[4];
            
            clip_pts[0] = cgalPoly_Point( -180.0, clip_bottom );
            clip_pts[1] = cgalPoly_Point(  180.0, clip_bottom );
            clip_pts[2] = cgalPoly_Point(  180.0, clip_top );
            clip_pts[3] = cgalPoly_Point( -180.0, clip_top );
            cgalPoly_Polygon clip_row( clip_pts, clip_pts+4 );

            SG_LOG( SG_GENERAL, SG_INFO, "   CLIPPED row " << row << " center lat is " << b_clip.get_center_lat() << " clip_botton is " << clip_bottom << " clip_top is " << clip_top );

            tgPolygonSet clipped = subject.intersection( clip_row );
            if ( !clipped.isEmpty() ) {
#if 0
                if ( subject.GetPreserve3D() ) {
                    clipped.InheritElevations( subject );
                    clipped.SetPreserve3D( true );
                }
                clipped.SetTexParams( subject.GetTexParams() );
                if ( subject.GetTexMethod() == TG_TEX_BY_GEODE ) {
                    // need to set center latitude for geodetic texturing
                    clipped.SetTexMethod( TG_TEX_BY_GEODE, b_clip.get_center_lat() );
                }
#endif

                ClipRow( clipped, b_clip.get_center_lat() );
            }
        }
    }
}

#if 0
long int tgChopper::GenerateIndex( std::string path )
{
    std::string index_file = path + "/chop.idx";
    long int index = 0;
    int  bRead = -1;

    //Open or create the named mutex
    boost::interprocess::named_mutex mutex(boost::interprocess::open_or_create, "tgChopper_index2");
    {
        boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(mutex);

        /* first try to read the file */
        FILE *fp = fopen( index_file.c_str(), "r+" );
        if ( fp == NULL ) {
            /* doesn't exist - create it */
            fp = fopen( index_file.c_str(), "w" );
            if ( fp == NULL ) {
                SG_LOG(SG_GENERAL, SG_ALERT, "Error cannot open Index file " << index_file << " for writing");
                boost::interprocess::named_mutex::remove("tgChopper_index2");
                exit( 0 );
            }
        } else {
            bRead = fread( (void*)&index, sizeof(long int), 1, fp );
            if (ferror(fp))
            {
                perror ("The following error occurred");
                SG_LOG(SG_GENERAL, SG_ALERT, "Error reading Index file " << index_file << " abort : bRead is " << bRead);
                boost::interprocess::named_mutex::remove("tgChopper_index2");
                exit(0);
            }
        }

        index++;

        rewind( fp );
        fwrite( (void*)&index, sizeof(long int), 1, fp );
        fclose( fp );
    }

    boost::interprocess::named_mutex::remove("tgChopper_index2");

    return index;
}
#endif

void tgChopper::Save( bool DebugShapefiles )
{
    // traverse the bucket list
    bucket_polys_map_interator it;
    char tile_name[16];

    for (it=bp_map.begin(); it != bp_map.end(); it++) {
        SGBucket b( (*it).first );
        tgPolygonSetList const& polys = (*it).second;

        std::string path = root_path + "/" + b.gen_base_path();
        sprintf( tile_name, "%ld", b.gen_index() );

        std::string polyfile = path + "/" + tile_name;

        SGPath sgp( polyfile );
        sgp.create_dir( 0755 );

        // save chopped polygons to a Shapefile in layer named from material
        for ( unsigned int i=0; i<polys.size(); i++ ) {
            polys[i].toShapefile( sgp.c_str(), polys[i].getMaterial().c_str() );
        }
    }
}
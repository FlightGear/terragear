#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

#include <simgear/debug/logstream.hxx>
#include <simgear/structure/exception.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/io/lowlevel.hxx>

#include "tg_chopper.hxx"
#include "tg_shapefile.hxx"
#include "tg_misc.hxx"

tgPolygon tgChopper::Clip( const tgPolygon& subject,
                      const std::string& type,
                      SGBucket& b )
{
    tgPolygon base, result;

    // set up clipping tile : and remember to add the nodes!
    base.AddNode( 0, b.get_corner( SG_BUCKET_SW ) );
    base.AddNode( 0, b.get_corner( SG_BUCKET_SE ) );
    base.AddNode( 0, b.get_corner( SG_BUCKET_NE ) );
    base.AddNode( 0, b.get_corner( SG_BUCKET_NW ) );

    result = tgPolygon::Intersect( subject, base );    
    if ( result.Contours() > 0 ) {
        if ( subject.GetPreserve3D() ) {
            result.InheritElevations( subject );
            result.SetPreserve3D( true );
        }
        result.SetTexParams( subject.GetTexParams() );
        if ( subject.GetTexMethod() == TG_TEX_BY_GEODE ) {
            // need to set center latitude for geodetic texturing
            result.SetTexMethod( TG_TEX_BY_GEODE, b.get_center_lat() );
        }
        result.SetFlag(type);

        lock.lock();
        bp_map[b.gen_index()].push_back( result );
        lock.unlock();
    }

    return result;
}

// Pass in the center lat for clipping buckets from the row.  
// We can't rely on sgBucketOffset, as rounding error sometimes causes it to look like there are 2 rows 
// (the first being a sliver)
// This leads to using that poly as the subject - which leads to having no usable polygon for this row.
void tgChopper::ClipRow( const tgPolygon& subject, const double& center_lat, const std::string& type )
{
    tgRectangle bb = subject.GetBoundingBox();
    SGBucket    b_min( bb.getMin() );
    SGBucket    b_max( bb.getMax() );
    int         dx, dy;

    sgBucketDiff(b_min, b_max, &dx, &dy);
    SGBucket start = SGBucket(SGGeod::fromDeg( b_min.get_center_lon(), center_lat ));

    for ( int i = 0; i <= dx; ++i ) {
        SGBucket b_cur = start.sibling(i, 0);
        Clip( subject, type, b_cur );
    }
}

void tgChopper::Add( const tgPolygon& subject, const std::string& type )
{
    // bail out immediately if polygon is empty
    if ( subject.Contours() == 0 )
        return;

    tgRectangle bb = subject.GetBoundingBox();

    // find buckets for min, and max points of convex hull.
    // note to self: self, you should think about checking for
    // polygons that span the date line
    SGBucket b_min( bb.getMin() );
    SGBucket b_max( bb.getMax() );
    SGBucket b_cur;
    int      dx, dy;

    sgBucketDiff(b_min, b_max, &dx, &dy);
    SG_LOG( SG_GENERAL, SG_DEBUG, "  y_min = " << bb.getMin().getLatitudeDeg() << " y_max = " << bb.getMax().getLatitudeDeg() << " dx = " <<  dx << "  dy = " << dy );

    if ( (dx > 2880) || (dy > 1440) )
        throw sg_exception("something is really wrong in split_polygon()!!!!");

    if ( dy == 0 )
    {
        // We just have a single row - no need to intersect first
        SG_LOG( SG_GENERAL, SG_DEBUG, "   UN_CLIPPED row -  center lat is " << b_min.get_center_lat() );

        ClipRow( subject, b_min.get_center_lat(), type );
    }
    else
    {
        // Multiple rows - perform row intersection to reduce the number of bucket clips we need
        // since many shapes are narraw in some places, wide in others - bb will be at the widest part
        SG_LOG( SG_GENERAL, SG_DEBUG, "subject spans tile rows: bb is from lat " << bb.getMin().getLatitudeDeg() << " to " << bb.getMax().getLatitudeDeg() << " dy is " << dy );

        for ( int row = 0; row <= dy; row++ )
        {
            // Generate a clip rectangle for the whole row

            SGBucket  b_clip      = b_min.sibling(0, row);
            double    clip_bottom = b_clip.get_center_lat() - SG_HALF_BUCKET_SPAN;
            double    clip_top    = b_clip.get_center_lat() + SG_HALF_BUCKET_SPAN;
            tgPolygon clip_row, clipped;

            SG_LOG( SG_GENERAL, SG_DEBUG, "   CLIPPED row " << row << " center lat is " << b_clip.get_center_lat() << " clip_botton is " << clip_bottom << " clip_top is " << clip_top );

            clip_row.AddNode( 0, SGGeod::fromDeg(-180.0, clip_bottom) );
            clip_row.AddNode( 0, SGGeod::fromDeg( 180.0, clip_bottom) );
            clip_row.AddNode( 0, SGGeod::fromDeg( 180.0, clip_top)    );
            clip_row.AddNode( 0, SGGeod::fromDeg(-180.0, clip_top)    );

            clipped = tgPolygon::Intersect( subject, clip_row );
            if ( clipped.TotalNodes() > 0 ) {

                if ( subject.GetPreserve3D() ) {
                    clipped.InheritElevations( subject );
                    clipped.SetPreserve3D( true );
                }
                clipped.SetTexParams( subject.GetTexParams() );
                if ( subject.GetTexMethod() == TG_TEX_BY_GEODE ) {
                    // need to set center latitude for geodetic texturing
                    clipped.SetTexMethod( TG_TEX_BY_GEODE, b_clip.get_center_lat() );
                }
                clipped.SetFlag(type);

                ClipRow( clipped, b_clip.get_center_lat(), type );
            }
        }
    }
}

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

void tgChopper::Save( bool DebugShapefiles )
{
    // traverse the bucket list
    bucket_polys_map_interator it;
    char tile_name[16];
    char poly_ext[16];

    char layer[32];
    char ds_name[64];

    for (it=bp_map.begin(); it != bp_map.end(); it++) {
        SGBucket b( (*it).first );
        tgpolygon_list const& polys = (*it).second;

        sprintf(ds_name, "./bucket_%s", b.gen_index_str().c_str() );

        std::string path = root_path + "/" + b.gen_base_path();
        sprintf( tile_name, "%ld", b.gen_index() );

        std::string polyfile = path + "/" + tile_name;

        SGPath sgp( polyfile );
        sgp.create_dir( 0755 );

        long int poly_index = GenerateIndex( path );

        sprintf( poly_ext, "%ld", poly_index );
        polyfile = polyfile + "." + poly_ext;

        gzFile fp;
        if ( (fp = gzopen( polyfile.c_str(), "wb9" )) == NULL ) {
            SG_LOG( SG_GENERAL, SG_INFO, "ERROR: opening " << polyfile.c_str() << " for writing!" );
            return;
        }

        /* Write polys to the file */
        sgWriteUInt( fp, polys.size() );
        for ( unsigned int i=0; i<polys.size(); i++ ) {
            polys[i].SaveToGzFile( fp );

            if ( DebugShapefiles )
            {
                sprintf(layer, "poly_%s-%d", b.gen_index_str().c_str(), i );
                tgShapefile::FromPolygon( polys[i], true, false, ds_name, layer, "poly" );
            }
        }

        gzclose( fp );
    }
}

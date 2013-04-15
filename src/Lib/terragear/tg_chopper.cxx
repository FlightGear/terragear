#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

#include <simgear/debug/logstream.hxx>
#include <simgear/structure/exception.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/io/lowlevel.hxx>


#include "tg_chopper.hxx"
#include "tg_shapefile.hxx"

unsigned int strip_clip = 0;

tgPolygon tgChopper::Clip( const tgPolygon& subject,
                      const std::string& type,
                      SGBucket& b )
{
    // p;

    SGGeod min, max;
    SGGeod c    = b.get_center();
    double span = b.get_width();
    tgPolygon base, result;

    // calculate bucket dimensions
    if ( (c.getLatitudeDeg() >= -89.0) && (c.getLatitudeDeg() < 89.0) ) {
        min = SGGeod::fromDeg( c.getLongitudeDeg() - span/2.0, c.getLatitudeDeg() - SG_HALF_BUCKET_SPAN );
        max = SGGeod::fromDeg( c.getLongitudeDeg() + span/2.0, c.getLatitudeDeg() + SG_HALF_BUCKET_SPAN );
    } else if ( c.getLatitudeDeg() < -89.0) {
        min = SGGeod::fromDeg( -90.0, -180.0 );
        max = SGGeod::fromDeg( -89.0,  180.0 );
    } else if ( c.getLatitudeDeg() >= 89.0) {
        min = SGGeod::fromDeg(  89.0, -180.0 );
        max = SGGeod::fromDeg(  90.0,  180.0 );
    } else {
        SG_LOG( SG_GENERAL, SG_ALERT,  "Out of range latitude in clip_and_write_poly() = " << c.getLatitudeDeg() );
    }

    SG_LOG( SG_GENERAL, SG_DEBUG, "  (" << min << ") (" << max << ")" );

    // set up clipping tile
    base.AddNode( 0, SGGeod::fromDeg( min.getLongitudeDeg(), min.getLatitudeDeg()) );
    base.AddNode( 0, SGGeod::fromDeg( max.getLongitudeDeg(), min.getLatitudeDeg()) );
    base.AddNode( 0, SGGeod::fromDeg( max.getLongitudeDeg(), max.getLatitudeDeg()) );
    base.AddNode( 0, SGGeod::fromDeg( min.getLongitudeDeg(), max.getLatitudeDeg()) );

    result = tgPolygon::Intersect( base, subject );
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
    double      min_center_lon = b_min.get_center_lon();
    int         dx, dy;

    sgBucketDiff(b_min, b_max, &dx, &dy);

    for ( int i = 0; i <= dx; ++i ) {
        SGBucket b_cur = sgBucketOffset(min_center_lon, center_lat, i, 0);
        Clip( subject, type, b_cur );
    }
}

void tgChopper::Add( const tgPolygon& subject, const std::string& type )
{
    // bail out immediately if polygon is empty
    if ( subject.Contours() == 0 )
        return;

    tgRectangle bb = subject.GetBoundingBox();
    SG_LOG( SG_GENERAL, SG_DEBUG, "  min = " << bb.getMin() << " max = " << bb.getMax() );

    // find buckets for min, and max points of convex hull.
    // note to self: self, you should think about checking for
    // polygons that span the date line
    SGBucket b_min( bb.getMin() );
    SGBucket b_max( bb.getMax() );
    SGBucket b_cur;
    int      dx, dy;

    sgBucketDiff(b_min, b_max, &dx, &dy);
    SG_LOG( SG_GENERAL, SG_DEBUG, "  dx = " << dx << "  dy = " << dy );

    if ( (dx > 2880) || (dy > 1440) )
        throw sg_exception("something is really wrong in split_polygon()!!!!");

    if ( dy == 0 )
    {
        // We just have a single row - no need to intersect first
        ClipRow( subject, b_min.get_center_lat(), type );
    }
    else
    {
        // Multiple rows - perform row intersection to reduce the number of bucket clips we need
        // since many shapes are narraw in some places, wide in others - bb will be at the widest part
        SG_LOG( SG_GENERAL, SG_DEBUG, "subject spans tile rows: bb is from lat " << bb.getMin().getLatitudeDeg() << " to " << bb.getMax().getLatitudeDeg() << " dy is " << dy );

        for ( int row = 0; row <= dy; row++ )
        {
            // Generate a clip rectangle - add some buffer on top and bottom, so we don't clip directly on an edge when we 
            // clip the individual buckets
            // TODO : May no longer be necessary
            SGBucket  b_clip      = sgBucketOffset( bb.getMin().getLongitudeDeg(), bb.getMin().getLatitudeDeg(), 0, row );
            double    clip_bottom = b_clip.get_center_lat() - SG_HALF_BUCKET_SPAN; // + 0.01);
            double    clip_top    = b_clip.get_center_lat() + SG_HALF_BUCKET_SPAN; // + 0.01);
            tgPolygon clip_row, clipped;

            SG_LOG( SG_GENERAL, SG_DEBUG, "   row " << row << " center lat is " << b_clip.get_center_lat() << " clip_botton is " << clip_bottom << " clip_top is " << clip_top );

            clip_row.AddNode( 0, SGGeod::fromDeg(-180.0, clip_bottom) );
            clip_row.AddNode( 0, SGGeod::fromDeg( 180.0, clip_bottom) );
            clip_row.AddNode( 0, SGGeod::fromDeg( 180.0, clip_top)    );
            clip_row.AddNode( 0, SGGeod::fromDeg(-180.0, clip_top)    );

            clipped = tgPolygon::Intersect( clip_row, subject );
            if ( clipped.TotalNodes() > 0 ) {
                ClipRow( clipped, b_clip.get_center_lat(), type );

#if 0
                {
                    char layer[32];
                    char ds_name[64];
                    sprintf(layer,   "clipped_%d", strip_clip++ );
                    sprintf(ds_name, "./stripped_%s", type.c_str() );

                    tgShapefile::FromPolygon( clipped, ds_name, layer, "poly" );
                }
#endif

            }
        }
    }
}

long int tgChopper::GenerateIndex( std::string path )
{
    std::string index_file = path + "/chop.idx";
    long int index = 0;

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
            fread( (void*)&index, sizeof(long int), 1, fp );
        }

        index++;

        rewind( fp );
        fwrite( (void*)&index, sizeof(long int), 1, fp );
        fclose( fp );
    }

    boost::interprocess::named_mutex::remove("tgChopper_index2");

    return index;
}

void tgChopper::Save( void )
{
    // traverse the bucket list
    bucket_polys_map_interator it;
    char tile_name[16];
    char poly_ext[16];

    for (it=bp_map.begin(); it != bp_map.end(); it++) {
        SGBucket b( (*it).first );
        tgpolygon_list const& polys = (*it).second;

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
        }

        gzclose( fp );
    }
}
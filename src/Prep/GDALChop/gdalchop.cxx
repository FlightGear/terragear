// gdalchop.cxx -- chop up a GDAL-readable raster-file into it's corresponding pieces
//                 and stuff them into the workspace directory
//
// Written by Ralf Gerlich, started February 20th, 2007
//
// Copyright (C) 2007 Ralf Gerlich - ralf.gerlich@custom-scenery.org
//
// Loosely based on hgtchop by Curtis L. Olson - http://www.flightgear.org/~curt
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//

#include <algorithm>
#include <chrono>
#include <iostream>

#include <simgear/compiler.h>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/io/lowlevel.hxx>
#include <simgear/misc/sg_path.hxx>

#include <Lib/terragear/tg_rectangle.hxx>

#include <gdal.h>
#include <gdal_priv.h>
#include <gdalwarper.h>
#include <ogr_spatialref.h>

#include <boost/scoped_array.hpp>

/*
 * A simple benchmark using a 5x5 degree package
 * has shown that gdalchop takes only 80% of the time
 * hgtchop requires for chopping that data.
 *
 * This includes unzipping of the files.
 *
 * The experiment was done with the srtm_38_03.zip tile of
 * CGIAR data and the equivalent tiles of SRTM v2 data,
 * using gdalchop for chopping the CGIAR data and hgtchop for
 * the SRTM v2 data.
 *
 * CPU: Intel(R) Core(TM)2 CPU T5500  @ 1.66GHz
 *           (only one core is used during chopping)
 *
 * Three measurements were taken for each tool and each
 * generated .arr.gz files for 800 buckets.
 *
 * hgtchop: 216s 211s 211s, mean: 212s, 0.266s/bucket
 * gdalchop: 172s 167s 169s, mean: 169s, 0.212s/bucket
 *
 * - Ralf Gerlich
 */

struct SimpleRasterTransformerInfo {
    GDALTransformerFunc pfnTransformer;
    void* pTransformerArg;
    double x0, y0;
    double col_step, row_step;
};

int SimpleRasterTransformer(void *pTransformerArg,
                            int bDstToSrc, int nPointCount,
                            double *x, double *y, double *z, int *panSuccess )
{
    SimpleRasterTransformerInfo* info = static_cast<SimpleRasterTransformerInfo*>(pTransformerArg);
    int success;

    if (bDstToSrc) {
        /* transform destination -> source */
        for (int i = 0; i < nPointCount; i++) {
            x[i] = info->x0 + info->col_step * x[i];
            y[i] = info->y0 + info->row_step * y[i];
        }

        success = info->pfnTransformer(info->pTransformerArg,
                                       bDstToSrc, nPointCount,
                                       x, y, z, panSuccess);
    } else {
        success = info->pfnTransformer(info->pTransformerArg,
                                       bDstToSrc, nPointCount,
                                       x, y, z, panSuccess);
        for (int i = 0; i < nPointCount; i++) {
            if (!panSuccess[i])
                continue;
            x[i] = (x[i] - info->x0) / info->col_step;
            y[i] = (y[i] - info->y0) / info->row_step;
        }
    }
    return success;
}

class ImageInfo {
public:
    explicit ImageInfo(GDALDataset *dataset);

    void GetBounds(double &n, double &s, double &e, double &w) const {
        n = north;
        s = south;
        e = east;
        w = west;
    }

    tgRectangle GetBoundingBox() const {
        tgRectangle box( SGGeod::fromDeg(east, south), SGGeod::fromDeg(west, north) );
        box.sanify();
        return box;
    }

    const char* GetDescription() const {
        return dataset->GetDescription();
    }

    int GetColStepArcsec() {
        return pxSizeX * 3600;
    }

    int GetRowStepArcsec() {
        return pxSizeY * 3600;
    }

    void GetDataChunk(int *buffer,
                      double x, double y,
                      double colstep, double rowstep,
                      int w, int h,
                      int srcband = 1, int nodata = -32768);

protected:
    /* The dataset */
    GDALDataset *dataset;

    /* Source spatial reference system */
    OGRSpatialReference srs;

    /* Coordinate transformation pixel -> geographic */
    double geoXfrm[6];

    /* Coordinate transformation to WGS84 */
    OGRCoordinateTransformation *wgs84xform;

    /* geographical edge coordinates in CCW order, WGS84 */
    double geoX[4], geoY[4];

    /* bounding box in WGS84 */
    double north, south, east, west;

    /* Pixel size in degs */
    double pxSizeX, pxSizeY;
};

ImageInfo::ImageInfo(GDALDataset *dataset) :
    dataset(dataset),
    srs(dataset->GetProjectionRef())
{
    OGRSpatialReference wgs84SRS;

    // NOTE : This is equivelent to WGS84 - just using th formal EPSG db.
    // GDAL knows this well known projection already
    wgs84SRS.SetWellKnownGeogCS( "EPSG:4326" );

    /* Determine the bounds of the input file in WGS84 */
    int w = dataset->GetRasterXSize();
    int h = dataset->GetRasterYSize();

    if (dataset->GetGeoTransform(geoXfrm) != CE_None) {
        SG_LOG(SG_GENERAL, SG_ALERT,
               "Could not determine transform matrix for dataset "
               "'" << dataset->GetDescription() << "'"
               ":" << CPLGetLastErrorMsg());
        exit(1);
    }

    /* calculate pixel size */
    pxSizeX = fabs(geoXfrm[1]);
    pxSizeY = fabs(geoXfrm[5]);

    /* create points in CCW order */
    geoX[0] = geoXfrm[0] + 0 * geoXfrm[1] + 0 * geoXfrm[2];
    geoX[1] = geoXfrm[0] + 0 * geoXfrm[1] + h * geoXfrm[2];
    geoX[2] = geoXfrm[0] + w * geoXfrm[1] + h * geoXfrm[2];
    geoX[3] = geoXfrm[0] + w * geoXfrm[1] + 0 * geoXfrm[2];
    geoY[0] = geoXfrm[3] + 0 * geoXfrm[4] + 0 * geoXfrm[5];
    geoY[1] = geoXfrm[3] + 0 * geoXfrm[4] + h * geoXfrm[5];
    geoY[2] = geoXfrm[3] + w * geoXfrm[4] + h * geoXfrm[5];
    geoY[3] = geoXfrm[3] + w * geoXfrm[4] + 0 * geoXfrm[5];

    const char* projRef = dataset->GetProjectionRef();

    srs = OGRSpatialReference(projRef);

    wgs84xform = OGRCreateCoordinateTransformation( &srs, &wgs84SRS );

    if (!wgs84xform->Transform(4, geoX, geoY)) {
        SG_LOG(SG_GENERAL, SG_ALERT,
               "Could not transform edge points of dataset "
               "'" << dataset->GetDescription() << "'"
               ":" << CPLGetLastErrorMsg());
        exit(1);
    }

    east = west = geoX[0];
    north = south = geoY[0];

    for (int j = 1; j < 4; j++) {
        north = std::max(north, geoY[j]);
        south = std::min(south, geoY[j]);
        east = std::max(east, geoX[j]);
        west = std::min(west, geoX[j]);
    }

    SG_LOG(SG_GENERAL, SG_INFO,
           "INFO: Bounds for '" << dataset->GetDescription() << "' are"
           " n=" << north << " s=" << south <<
           " e=" << east << " w=" << west);
}

void ImageInfo::GetDataChunk(int *buffer,
                             double x, double y,
                             double colstep, double rowstep,
                             int w, int h,
                             int srcband, int nodata)
{
    OGRSpatialReference wgs84SRS;

    wgs84SRS.SetWellKnownGeogCS( "EPSG:4326" );

    char* wgs84WKT;
    wgs84SRS.exportToWkt(&wgs84WKT);

    /* Setup a raster transformation from WGS84 to raster coordinates of the array files */
    SimpleRasterTransformerInfo xformData;
    xformData.pTransformerArg = GDALCreateGenImgProjTransformer(
        dataset, NULL,
        NULL, wgs84WKT,
        FALSE,
        0.0,
        1);

    xformData.pfnTransformer = GDALGenImgProjTransform;
    xformData.x0 = x - pxSizeX * 0.5;
    xformData.y0 = y - pxSizeY * 0.5;
    xformData.col_step = colstep;
    xformData.row_step = rowstep;

    // TODO: check if this image can actually cover part of the chunk

    /* establish the full source to target transformation */
    GDALWarpOptions *psWarpOptions = GDALCreateWarpOptions();

    int srcBandNumbers[] = { srcband };
    int dstBandNumbers[] = { 1 };

    double srcNodataReal;
    double srcNodataImag   =  0.0;
    int    srcHasNodataValue;

    srcNodataReal = dataset->GetRasterBand(srcband)->GetNoDataValue(&srcHasNodataValue);

    psWarpOptions->hSrcDS = dataset;
    psWarpOptions->hDstDS = NULL;
    psWarpOptions->nBandCount = 1;
    psWarpOptions->panSrcBands = srcBandNumbers;
    psWarpOptions->panDstBands = dstBandNumbers;
    psWarpOptions->nSrcAlphaBand = 0;
    psWarpOptions->nDstAlphaBand = 0;
    psWarpOptions->padfSrcNoDataReal = (srcHasNodataValue ? &srcNodataReal : NULL);
    psWarpOptions->padfSrcNoDataImag = (srcHasNodataValue ? &srcNodataImag : NULL);
    psWarpOptions->padfDstNoDataReal = NULL;
    psWarpOptions->eResampleAlg = GRA_NearestNeighbour;
    psWarpOptions->eWorkingDataType = GDT_Int32;

    psWarpOptions->pfnTransformer = SimpleRasterTransformer;
    psWarpOptions->pTransformerArg = &xformData;

    GDALWarpOperation oOperation;
    oOperation.Initialize( psWarpOptions );

    /* do the warp */
    if (oOperation.WarpRegionToBuffer(0, 0, w, h, buffer, GDT_Int32) != CE_None) {
        SG_LOG(SG_GENERAL, SG_ALERT,
               "Could not warp to buffer on dataset '" << GetDescription() << "'"
               ":" << CPLGetLastErrorMsg());
    }

    /* clean up */
    psWarpOptions->panSrcBands = NULL;
    psWarpOptions->panDstBands = NULL;
    psWarpOptions->padfSrcNoDataReal = NULL;
    psWarpOptions->padfSrcNoDataImag = NULL;
    psWarpOptions->padfDstNoDataReal = NULL;

    GDALDestroyGenImgProjTransformer( xformData.pTransformerArg );
    GDALDestroyWarpOptions( psWarpOptions );
}

void write_bucket(const std::string& work_dir, SGBucket bucket,
                  int* buffer,
                  int min_x, int min_y,
                  int span_x, int span_y,
                  int col_step, int row_step)
{
    // generate output file name
    std::string base = bucket.gen_base_path();
    std::string path = work_dir + "/" + base;
    SGPath sgp( path );
    sgp.append( "dummy" );
    sgp.create_dir( 0755 );

    std::string array_file = path + "/" + bucket.gen_index_str() + ".arr.gz";

    gzFile fp;
    if ( (fp = gzopen(array_file.c_str(), "wb9")) == NULL ) {
        SG_LOG(SG_GENERAL, SG_ALERT, "cannot open " << array_file << " for writing!");
        exit(-1);
    }

    int32_t header = 0x54474152; // 'TGAR'
    sgWriteLong(fp, header);
    sgWriteInt(fp, min_x); sgWriteInt(fp, min_y);
    sgWriteInt(fp, span_x); sgWriteInt(fp, col_step);
    sgWriteInt(fp, span_y); sgWriteInt(fp, row_step);

    for ( int x = 0; x < span_x; ++x ) {
        for ( int y = 0; y < span_y; ++y ) {
            sgWriteShort(fp, buffer[ y * span_x + x ]);
        }
    }

    gzclose(fp);
}

void process_bucket(const SGPath& work_dir, SGBucket bucket,
                    ImageInfo* images[], int imagecount,
                    bool forceWrite = false)
{
    double bnorth, bsouth, beast, bwest;

    tgRectangle BucketBounds(bucket.get_corner(0), bucket.get_corner(2));
    BucketBounds.sanify();

    bnorth = bucket.get_center_lat() + bucket.get_height() * 0.5;
    bsouth = bucket.get_center_lat() - bucket.get_height() * 0.5;
    beast = bucket.get_center_lon() + bucket.get_width() * 0.5;
    bwest = bucket.get_center_lon() - bucket.get_width() * 0.5;

    SG_LOG(SG_GENERAL, SG_INFO, "processing bucket " << bucket << "(" << bucket.gen_index() << ") n=" << bnorth << " s=" << bsouth << " e=" << beast << " w=" << bwest);

    /* Get the data from the input datasets... */
    int min_x, min_y, span_x, span_y;

    min_x = (int)(bwest * 3600.0);
    min_y = (int)(bsouth * 3600.0);

    // Determine minimum common arcsec steps across images
    int col_step = -1, row_step = -1;
    for (int i = 0; i < imagecount; i++) {
        if ( images[i]->GetColStepArcsec() > col_step ) {
            col_step = images[i]->GetColStepArcsec();
        }
        if ( images[i]->GetRowStepArcsec() > row_step ) {
            row_step = images[i]->GetRowStepArcsec();
        }
    }

    span_x = (bucket.get_width() * 3600 / col_step) + 1;
    span_y = (bucket.get_height() * 3600 / row_step) + 1;

    int cellcount = span_x * span_y;
    boost::scoped_array<int> buffer(new int[cellcount]);

    ::memset(buffer.get(), 0, cellcount * sizeof(int));

    for (int i = 0; i < imagecount; i++) {
        if ( images[i]->GetBoundingBox().intersects(BucketBounds) ) {
            images[i]->GetDataChunk(buffer.get(),
                                    bwest, bsouth,
                                    col_step / 3600.0, row_step / 3600.0,
                                    span_x, span_y );
        }
    }

    /* ...check the amount of undefined cells... */
    int nodataCellCount = 0;

    for (int i = 0; i < cellcount; i++)
        if (buffer[i] == -32768)
            nodataCellCount++;

    const double nodataPercLimit = 5.0;
    double nodataPerc = 100.0 * nodataCellCount / cellcount;

    if (nodataCellCount > 0) {
        SG_LOG(SG_GENERAL, SG_INFO, "    " << nodataCellCount << " cells are not covered with data (" << nodataPerc << "% of cells)");
    }

    if (nodataPerc > nodataPercLimit) {
        SG_LOG(SG_GENERAL, SG_INFO, "    there is not enough data available to cover this cell (limit for non-covered cells is " << nodataPercLimit << "%)");
        /* don't write out if not forced to */
        if (!forceWrite)
            return;
    }

    /* ...and write it out */
    write_bucket(work_dir.str(), bucket,
                 buffer.get(),
                 min_x, min_y,
                 span_x, span_y,
                 col_step, row_step);
}

int main(int argc, const char **argv)
{
    sglog().setLogLevels( SG_ALL, SG_INFO );

    if ( argc < 3 ) {
        SG_LOG(SG_GENERAL, SG_ALERT,
               "Usage " << argv[0] << " <work_dir> <datasetname...> [-- <bucket-idx> ...]");
        return EXIT_FAILURE;
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    SGPath work_dir(argv[1]);
    work_dir.create_dir( 0755 );

    GDALAllRegister();

    int datasetcount = 0, tilecount = 0;
    int dashpos;

    for (dashpos = 2; dashpos < argc; dashpos++)
        if (!strcmp(argv[dashpos], "--"))
            break;

    datasetcount = dashpos - 2;
    tilecount = (dashpos == argc ? 0 : argc - dashpos - 1);

    if (datasetcount == 0) {
        SG_LOG(SG_GENERAL, SG_ALERT,
               "No data sets supplied. Must provide at least one dataset.");
        exit(1);
    }

    const char** tilenames = argv + dashpos + 1;
    const char** datasetnames = argv + 2;

    boost::scoped_array<ImageInfo *> images( new ImageInfo *[datasetcount] );

    double north = -1000, south = 1000, east = -1000, west = 1000;

    // TODO: allow specification of bounds by user

    /*
     * Step 1: Open all provided datasets and determine their bounds in WGS84.
     */
    for (int i = 0; i < datasetcount; i++) {
        GDALDataset* dataset;

        dataset = (GDALDataset*)GDALOpenShared(datasetnames[i], GA_ReadOnly);

        if (dataset == NULL) {
            SG_LOG(SG_GENERAL, SG_ALERT,
                   "Could not open dataset '" << datasetnames[i] << "'"
                   ":" << CPLGetLastErrorMsg());
            return EXIT_FAILURE;
        }

        images[i] = new ImageInfo(dataset);

        double inorth, isouth, ieast, iwest;
        images[i]->GetBounds(inorth, isouth, ieast, iwest);

        north = std::max(north, inorth);
        south = std::min(south, isouth);
        east = std::max(east, ieast );
        west = std::min(west, iwest );
    }

    SG_LOG(SG_GENERAL, SG_INFO, "Bounds of all datasets: n=" << north << " s=" << south << " e=" << east << " w=" << west);

    /*
     * Step 2: If no tiles were specified, go through all tiles contained in
     *         the common bounds of all datasets and find those which have
     *         sufficient coverage (non-null pixels). Create output for these.
     *         It tiles were specified, go through them and create output for
     *         all of them. Warn if no sufficient coverage (non-null pixels) is
     *         available.
     */
    if (tilecount == 0) {
        /*
         * No tiles were specified, so we determine the common bounds of all
         * specified datasets and check all the tiles contained in them.
         */

        SGBucket start( SGGeod::fromDeg(west, south) ), end( SGGeod::fromDeg(east, north) );

        int dx, dy;

        sgBucketDiff(start, end, &dx, &dy);

        SG_LOG(SG_GENERAL, SG_INFO, "dx=" << dx << " dy=" << dy);

        for (int x = 0; x <= dx; x++) {
            for (int y = 0; y <= dy; y++) {
                SGBucket bucket = start.sibling(x, y);

                process_bucket(work_dir, bucket, images.get(), datasetcount);
            }
        }
    } else {
        /*
         * Tiles were specified, so process them and warn if not enough
         * data is available, but write them in any case.
         */
        for (int i = 0; i < tilecount; i++) {
            SGBucket bucket(atol(tilenames[i]));

            process_bucket(work_dir, bucket, images.get(), datasetcount, true);
        }
    }

    GDALDestroyDriverManager();

    auto finish_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish_time - start_time;
    std::cout << std::endl << "Elapsed time: " << elapsed.count() << " seconds" << std::endl << std::endl;

    return EXIT_SUCCESS;
}

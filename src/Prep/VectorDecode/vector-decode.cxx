// vector-decode.cxx -- process OGR vector data layers
//                      generate smoothing contours, and polyygons that can be
//                      textured
//
// Written by Peter Sadrozinski, started January 2015.
// Based on ogr_decode.cxx Copyright (C) 2007 Ralf Gerlich
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

#include <string>
#include <map>

#include <ogrsf_frmts.h>

#include <simgear/compiler.h>
#include <simgear/debug/logstream.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/misc/sg_path.hxx>

#include <Include/version.h>

#include <terragear/tg_polygon.hxx>
#include <terragear/tg_intersection_generator.hxx>
#include <terragear/tg_chopper.hxx>
#include <terragear/tg_shapefile.hxx>

using std::string;

int line_width=50;
string line_width_col;
string area_type="Default";
string area_type_col;
int continue_on_errors=0;
int max_segment_length=0; // ==0 => don't split
int start_record=0;
bool use_attribute_query=false;
string attribute_query;
bool use_spatial_query=false;
double spat_min_x, spat_min_y, spat_max_x, spat_max_y;

tgIntersectionGenerator smooth_contours("./ig_dbg/", NULL);

/* very GDAL specific here... */
inline static bool is_ocean_area( const std::string &area ) {
    return area == "Ocean" || area == "Bay  Estuary or Ocean";
}

inline static bool is_void_area( const std::string &area ) {
    return area == "Void Area";
}

inline static bool is_null_area( const std::string& area ) {
    return area == "Null";
}

void processLineString(OGRLineString* poGeometry, const string& area_type, int width )
{
    SGGeod p0, p1;
    int i, numPoints;

    numPoints = poGeometry->getNumPoints();
    if (numPoints < 2) {
        SG_LOG( SG_GENERAL, SG_WARN, "Skipping line with less than two points" );
        return;
    }

    // add the points
    for ( i=1;i<numPoints;i++) {
        p0 = SGGeod::fromDeg( poGeometry->getX(i-1), poGeometry->getY(i-1) );
        p1 = SGGeod::fromDeg( poGeometry->getX(i),   poGeometry->getY(i) );

        smooth_contours.Insert( p0, p1, width, 0 );
    }
}

void processLayer(OGRLayer* poLayer, tgChopper& results )
{
    int feature_count=poLayer->GetFeatureCount();

    if (feature_count!=-1 && start_record>0 && start_record>=feature_count) {
        SG_LOG( SG_GENERAL, SG_ALERT, "Layer has only " << feature_count << " records, but start record is set to " << start_record );
        exit( 1 );
    }

    /* determine the indices of the required columns */
    OGRFeatureDefn *poFDefn = poLayer->GetLayerDefn();
    string layername=poFDefn->GetName();
    int line_width_field=-1, area_type_field=-1;

    if (!line_width_col.empty()) {
        line_width_field=poFDefn->GetFieldIndex(line_width_col.c_str());
        if (line_width_field==-1) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Field " << line_width_col << " for line-width not found in layer" );
        if (!continue_on_errors)
            exit( 1 );
        }
    }

    if (!area_type_col.empty()) {
        area_type_field=poFDefn->GetFieldIndex(area_type_col.c_str());
        if (area_type_field==-1) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Field " << area_type_col << " for line-width not found in layer" );
        if (!continue_on_errors)
            exit( 1 );
        }
    }

    /* setup a transformation to WGS84 */
    OGRSpatialReference *oSourceSRS, oTargetSRS;
    oSourceSRS=poLayer->GetSpatialRef();

    if (oSourceSRS == NULL) {
        SG_LOG( SG_GENERAL, SG_ALERT, "Layer " << layername << " has no defined spatial reference system" );
        exit( 1 );
    }

    char* srsWkt;
    oSourceSRS->exportToWkt(&srsWkt);
    SG_LOG( SG_GENERAL, SG_DEBUG, "Source spatial reference system: " << srsWkt );
    OGRFree(srsWkt);

    oTargetSRS.SetWellKnownGeogCS( "WGS84" );

    OGRCoordinateTransformation *poCT;

    poCT = OGRCreateCoordinateTransformation(oSourceSRS, &oTargetSRS);

    /* setup attribute and spatial queries */
    if (use_spatial_query) {
        double trans_min_x,trans_min_y,trans_max_x,trans_max_y;
        /* do a simple reprojection of the source SRS */
        OGRCoordinateTransformation *poCTinverse;

        poCTinverse = OGRCreateCoordinateTransformation(&oTargetSRS, oSourceSRS);

        trans_min_x=spat_min_x;
        trans_min_y=spat_min_y;
        trans_max_x=spat_max_x;
        trans_max_y=spat_max_y;

        poCTinverse->Transform(1,&trans_min_x,&trans_min_y);
        poCTinverse->Transform(1,&trans_max_x,&trans_max_y);

        poLayer->SetSpatialFilterRect(trans_min_x, trans_min_y,
                                      trans_max_x, trans_max_y);
    }

    if (use_attribute_query) {
        if (poLayer->SetAttributeFilter(attribute_query.c_str()) != OGRERR_NONE) {
            SG_LOG( SG_GENERAL, SG_ALERT, "Error in query expression '" << attribute_query << "'" );
            exit( 1 );
        }
    }

    OGRFeature *poFeature;
    poLayer->SetNextByIndex(start_record);
    for ( ; (poFeature = poLayer->GetNextFeature()) != NULL; OGRFeature::DestroyFeature( poFeature ) )
    {
        OGRGeometry *poGeometry;

        poGeometry = poFeature->GetGeometryRef();

        if (poGeometry==NULL) {
            SG_LOG( SG_GENERAL, SG_INFO, "Found feature without geometry!" );
            if (!continue_on_errors) {
                SG_LOG( SG_GENERAL, SG_ALERT, "Aborting!" );
                exit( 1 );
            } else {
                continue;
            }
        }

        assert(poGeometry!=NULL);

        OGRwkbGeometryType geoType=wkbFlatten(poGeometry->getGeometryType());

        if (geoType!=wkbPoint && geoType!=wkbMultiPoint &&
            geoType!=wkbLineString && geoType!=wkbMultiLineString &&
            geoType!=wkbPolygon && geoType!=wkbMultiPolygon) {
                SG_LOG( SG_GENERAL, SG_INFO, "Unknown feature" );
                continue;
        }

        string area_type_name=area_type;
        if (area_type_field!=-1) {
            area_type_name=poFeature->GetFieldAsString(area_type_field);
        }

        if ( is_ocean_area(area_type_name) ) {
            // interior of polygon is ocean, holes are islands

            SG_LOG(  SG_GENERAL, SG_ALERT, "Ocean area ... SKIPPING!" );

            // Ocean data now comes from GSHHS so we want to ignore
            // all other ocean data
            continue;
        } else if ( is_void_area(area_type_name) ) {
            // interior is ????

            // skip for now
            SG_LOG(  SG_GENERAL, SG_ALERT, "Void area ... SKIPPING!" );

            continue;
        } else if ( is_null_area(area_type_name) ) {
            // interior is ????

            // skip for now
            SG_LOG(  SG_GENERAL, SG_ALERT, "Null area ... SKIPPING!" );

            continue;
        }

        poGeometry->transform( poCT );

        switch (geoType) {
        case wkbLineString: {
            SG_LOG( SG_GENERAL, SG_DEBUG, "LineString feature" );
            int width=line_width;
            if (line_width_field!=-1) {
                width=poFeature->GetFieldAsInteger(line_width_field);
                if (width == 0) {
                    width=line_width;
                }
            }

            processLineString((OGRLineString*)poGeometry, area_type_name, width);
            break;
        }
        case wkbMultiLineString: {
            SG_LOG( SG_GENERAL, SG_DEBUG, "MultiLineString feature" );
            int width=line_width;
            if (line_width_field!=-1) {
                width=poFeature->GetFieldAsInteger(line_width_field);
                if (width == 0) {
                    width=line_width;
                }
            }

            OGRMultiLineString* multils=(OGRMultiLineString*)poGeometry;
            for (int i=0;i<multils->getNumGeometries();i++) {
                processLineString((OGRLineString*)poGeometry, area_type_name, width);
            }
            break;
        }
        default:
            /* Ignore unhandled objects */
            break;
        }
    }

    OCTDestroyCoordinateTransformation ( poCT );
}

void usage(char* progname) {
    SG_LOG( SG_GENERAL, SG_ALERT, "Usage: " <<
              progname << " [options...] <work_dir> <datasource> [<layername>...]" );
    SG_LOG( SG_GENERAL, SG_ALERT, "Options:" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--line-width width" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Width in meters for the lines" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--line-width-column colname" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Use value from colname as width for lines" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Overrides --line-width if present" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--point-width width" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Size in meters of the squares generated from points" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--point-width-column colname" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Use value from colname as width for points" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Overrides --point-width if present" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--area-type type" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Area type for all objects from file" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--area-type-column colname" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Use string from colname as area type" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Overrides --area-type if present" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--continue-on-errors" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Continue even if the file seems fishy" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--max-segment max_segment_length" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Maximum segment length in meters" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--start-record record_no" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Start processing at the specified record number (first record num=0)" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--where attrib_query" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Use an attribute query (like SQL WHERE)" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--spat xmin ymin xmax ymax" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        spatial query extents" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--texture-lines" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Enable textured lines" );
    SG_LOG( SG_GENERAL, SG_ALERT, "" );
    SG_LOG( SG_GENERAL, SG_ALERT, "<work_dir>" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Directory to put the polygon files in" );
    SG_LOG( SG_GENERAL, SG_ALERT, "<datasource>" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        The datasource from which to fetch the data" );
    SG_LOG( SG_GENERAL, SG_ALERT, "<layername>..." );
    SG_LOG( SG_GENERAL, SG_ALERT, "        The layers to process." );
    SG_LOG( SG_GENERAL, SG_ALERT, "        If no layer is given, all layers in the datasource are used" );
    exit(-1);
}

int main( int argc, char **argv ) {
    char* progname=argv[0];
    string datasource,work_dir;

    sglog().setLogLevels( SG_ALL, SG_INFO );

    while (argc>1) {
        if (!strcmp(argv[1],"--line-width")) {
            if (argc<3) {
                usage(progname);
            }
            line_width=atoi(argv[2]);
            argv+=2;
            argc-=2;
        } else if (!strcmp(argv[1],"--line-width-column")) {
            if (argc<3) {
                usage(progname);
            }
            line_width_col=argv[2];
            argv+=2;
            argc-=2;
        } else if (!strcmp(argv[1],"--area-type")) {
            if (argc<3) {
                usage(progname);
            }
            area_type=argv[2];
            argv+=2;
            argc-=2;
        } else if (!strcmp(argv[1],"--area-type-column")) {
            if (argc<3) {
                usage(progname);
            }
            area_type_col=argv[2];
            argv+=2;
            argc-=2;
        } else if (!strcmp(argv[1],"--continue-on-errors")) {
            argv++;
            argc--;
            continue_on_errors=1;
        } else if (!strcmp(argv[1],"--max-segment")) {
            if (argc<3) {
                usage(progname);
            }
            max_segment_length=atoi(argv[2]);
            argv+=2;
            argc-=2;
        } else if (!strcmp(argv[1],"--start-record")) {
            if (argc<3) {
                usage(progname);
            }
            start_record=atoi(argv[2]);
            argv+=2;
            argc-=2;
        } else if (!strcmp(argv[1],"--where")) {
            if (argc<3) {
                usage(progname);
            }
            use_attribute_query=true;
            attribute_query=argv[2];
            argv+=2;
            argc-=2;
        } else if (!strcmp(argv[1],"--spat")) {
            if (argc<6) {
                usage(progname);
            }
            use_spatial_query=true;
            spat_min_x=atof(argv[2]);
            spat_min_y=atof(argv[3]);
            spat_max_x=atof(argv[4]);
            spat_max_y=atof(argv[5]);
            argv+=5;
            argc-=5;
        } else if (!strcmp(argv[1],"--help")) {
            usage(progname);
        } else {
            break;
        }
    }

    SG_LOG( SG_GENERAL, SG_ALERT, "ogr-decode version " << getTGVersion() << "\n" );

    if (argc<3) {
        usage(progname);
    }
    work_dir=argv[1];
    datasource=argv[2];

    SGPath sgp( work_dir );
    sgp.append( "dummy" );
    sgp.create_dir( 0755 );

    tgChopper results( work_dir );

    SG_LOG( SG_GENERAL, SG_DEBUG, "Opening datasource " << datasource << " for reading." );

    OGRRegisterAll();
    OGRDataSource       *poDS;

    poDS = OGRSFDriverRegistrar::Open( datasource.c_str(), FALSE );
    if( poDS == NULL )
    {
        SG_LOG( SG_GENERAL, SG_ALERT, "Failed opening datasource " << datasource );
        exit( 1 );
    }

    OGRLayer  *poLayer;

    if (argc>3) {
        for (int i=3;i<argc;i++) {
            poLayer = poDS->GetLayerByName( argv[i] );

            if (poLayer == NULL )
            {
                SG_LOG( SG_GENERAL, SG_ALERT, "Failed opening layer " << argv[i] << " from datasource " << datasource );
                exit( 1 );
            }
            processLayer(poLayer, results );
        }
    } else {
        for (int i=0;i<poDS->GetLayerCount();i++) {
            poLayer = poDS->GetLayer(i);

            assert(poLayer != NULL);

            processLayer(poLayer, results );
        }
    }

    // add some additional Variables to the intersection generator
    // cleaning parameters
    // texture mode
    // simplify parameters
    // and add some data access
    // get skeleton segments
    // get skin segments
    // delta height info may be needed....
    // maybe needs a new class entirely based on intersectiongenerator.
    
    smooth_contours.Execute(true);
    
    results.Save(false);
    
    OGRDataSource::DestroyDataSource( poDS );

    return 0;
}

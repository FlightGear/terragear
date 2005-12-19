// main.cxx -- process shapefiles and extract polygon outlines,
//             lines and points, clipping against and sorting
//             them into the relevant tiles.
//
// Written by Curtis Olson, started February 1999.
// Modified by Ralf Gerlich, August 2005.
// Modified by Thomas Foerster, November 2005.
//
// Copyright (C) 1999  Curtis L. Olson  - http://www.flightgear.org/~curt
// Copyright (C) 2005  Ralf Gerlich     - http://home.easylink.de/rgerlich
// Copyright (C) 2005  Thomas Foerster  
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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// $Id$
 

#include <simgear/compiler.h>

#include STL_STRING
#include <map>

#include <simgear/debug/logstream.hxx>
#include <simgear/misc/sg_path.hxx>

#include <Geometry/line.hxx>
#include <Geometry/util.hxx>
#include <Polygon/chop.hxx>
#include <Polygon/index.hxx>
#include <Polygon/names.hxx>
#include <Polygon/polygon.hxx>
#include <shapelib/shapefil.h>

#ifdef _MSC_VER
#  include <Win32/mkdir.hpp>
#endif

SG_USING_STD( cout );
SG_USING_STD( string );
SG_USING_STD( map );

map<string,string> area_code_map;
int use_area_code_map=0;
int continue_on_errors=0;
int area_column=4,code_column=3;
double max_segment = 0.0;  // zero = no splitting
int width_column = -1;

void load_noaa_area_codes() {
    area_code_map["Urban (1990 Enhanced)"]="Urban";
    area_code_map["Residential"]="Urban";
    area_code_map["Commercial and Services"]="Urban";
    area_code_map["Industrial"]="Urban";
    area_code_map["Transportation, Communications and Utilities"]="Urban";
    area_code_map["Transportation, Communications, and Utilities"]="Urban";
    area_code_map["Industrial and Commercial Complexes"]="Urban";
    area_code_map["Mixed Urban or Built-up Land"]="BuiltUpCover";
    area_code_map["Mixed Urban or Built up Land"]="BuiltUpCover";
    area_code_map["Other Urban or Built-up Land"]="BuiltUpCover";
    area_code_map["Other Urban or Built up Land"]="BuiltUpCover";
    
    area_code_map["Cropland and Pasture"]="MixedCropPastureCover";
    area_code_map["Orchards, Groves, Vineyards, Nurseries, and Ornamental Horticultural Areas"]="IrrCropPastureCover";
    area_code_map["Orchards, Groves, Vineyards, Nurseries and Ornament"]="IrrCropPastureCover";
    area_code_map["Orchards, Groves, Vineyards, Nurseries and Ornamental Hort"]="IrrCropPastureCover";
    area_code_map["Confined Feeding Operations"]="MixedCropPastureCover";
    area_code_map["Other Agricultural Land"]="IrrCropPastureCover";
    
    area_code_map["Herbaceous Rangeland"]="GrassCover";
    area_code_map["Shrub and Brush Rangeland"]="ShrubCover";
    area_code_map["Mixed Rangeland"]="ShrubGrassCover";
    
    area_code_map["Deciduous Forest Land"]="DeciduousBroadCover";
    area_code_map["Evergreen Forest Land"]="EvergreenNeedleCover";
    area_code_map["Mixed Forest Land"]="MixedForestCover";
    
    area_code_map["Streams and Canals"]="Stream";
    area_code_map["Lakes"]="Lake";
    area_code_map["Reservoirs"]="Reservoir";
    area_code_map["Bays and Estuaries"]="Ocean";
    
    area_code_map["Forested Wetland"]="WoodedWetlandCover";
    area_code_map["Nonforested Wetland"]="HerbWetlandCover";
    
    area_code_map["Dry Salt Flats"]="DryLake";
    area_code_map["Beaches"]="DryLake";
    area_code_map["Sandy Areas Other than Beaches"]="DryLake";
    area_code_map["Bare Exposed Rock"]="BarrenCover";
    area_code_map["Strip Mines, Quarries, and Gravel Pits"]="BarrenCover";
    area_code_map["Transitional Areas"]="BarrenCover";
    area_code_map["Mixed Barren Land"]="BarrenCover";
    
    area_code_map["Shrub and Brush Tundra"]="WoodedTundraCover";
    area_code_map["Herbaceous Tundra"]="HerbTundraCover";
    area_code_map["Bare Ground"]="BareTundraCover";
    area_code_map["Wet Tundra"]="HerbTundraCover";
    area_code_map["Mixed Tundra"]="MixedTundraCover";
    
    area_code_map["Perennial Snowfields"]="Glacier";
    area_code_map["Glaciers"]="Glacier";
    
    area_code_map["No Data"]="Null";
    use_area_code_map=1;
}

// return the type of the shapefile record
AreaType get_shapefile_type(DBFHandle& hDBF, int rec) {
#if 0
    int         *panWidth, i, iRecord;
    char        szFormat[32];
    int         nWidth, nDecimals;
    int         bMultiLine = 0;
    char        szTitle[12];

    // grab the meta-information for all the fields
    // this applies to all the records in the DBF file.
    for ( i = 0; i < DBFGetFieldCount(hDBF); i++ ) {
	DBFFieldType eType;
	string pszTypeName;
 
	eType = DBFGetFieldInfo( hDBF, i, szTitle, &nWidth, &nDecimals );
	if( eType == FTString )
	    pszTypeName = "String";
	else if( eType == FTInteger )
	    pszTypeName = "Integer";
	else if( eType == FTDouble )
	    pszTypeName = "Double";
	else if( eType == FTInvalid )
	    pszTypeName = "Invalid";
	
	// printf( "Field %d: Type=%s, Title=`%s', Width=%d, Decimals=%d\n",
	//         i, pszTypeName.c_str(), szTitle, nWidth, nDecimals );
    }

    // Compute offsets to use when printing each of the field
    // values. We make each field as wide as the field title+1, or the
    // field value + 1.

    panWidth = (int *) malloc( DBFGetFieldCount( hDBF ) * sizeof(int) );
    for ( i = 0; i < DBFGetFieldCount(hDBF) && !bMultiLine; i++ ) {
        DBFFieldType eType;
 
        eType = DBFGetFieldInfo( hDBF, i, szTitle, &nWidth, &nDecimals );
        if( (int)strlen(szTitle) > nWidth ) {
            panWidth[i] = strlen(szTitle);
        } else {
            panWidth[i] = nWidth;
	}
 
        if( eType == FTString ) {
            sprintf( szFormat, "%%-%ds ", panWidth[i] );
        } else {
            sprintf( szFormat, "%%%ds ", panWidth[i] );
	}
        printf( szFormat, szTitle );
    }
    printf( "\n" );

    for( iRecord = 0; iRecord < DBFGetRecordCount(hDBF); iRecord++ ) {
        for( i = 0; i < DBFGetFieldCount(hDBF); i++ ) {
            DBFFieldType eType;
            
            eType = DBFGetFieldInfo( hDBF, i, szTitle, &nWidth, &nDecimals );
 
	    switch( eType )
		{
		case FTString:
		    sprintf( szFormat, "%%-%ds", nWidth );
		    printf( szFormat, 
			    DBFReadStringAttribute( hDBF, iRecord, i ) );
		    break;
		
		case FTInteger:
                    sprintf( szFormat, "%%%dd", nWidth );
                    printf( szFormat, 
                            DBFReadIntegerAttribute( hDBF, iRecord, i ) );
                    break;
 
		case FTDouble:
                    sprintf( szFormat, "%%%d.%dlf", nWidth, nDecimals );
                    printf( szFormat, 
                            DBFReadDoubleAttribute( hDBF, iRecord, i ) );
                    break;
                }
	}
    }
    printf( "\n" );

#endif
 
    string area = DBFReadStringAttribute( hDBF, rec, area_column );
    string code = DBFReadStringAttribute( hDBF, rec, code_column );
    cout << "next record = " << code << endl;

    // strip leading spaces
    while ( area[0] == ' ' ) {
	area = area.substr(1, area.length() - 1);
    }
    // strip trailing spaces
    while ( area[area.length() - 1] == ' ' ) {
	area = area.substr(0, area.length() - 1);
    }
    // strip other junk encountered
    while ( (int)area[area.length() - 1] == 9 ) {
	area = area.substr(0, area.length() - 1);
    }

    SG_LOG( SG_GENERAL, SG_INFO, "   raw area = " << area );
    SG_LOG( SG_GENERAL, SG_INFO, "       code = " << code );

    if (use_area_code_map) {
	area=area_code_map[area];
	SG_LOG( SG_GENERAL, SG_INFO, "cooked area = " << area );
    }

    return get_area_type( area );
}

// get attribute value of 'width' from shapefile
int get_shapefile_width(DBFHandle& hDBF, int rec) {
    string wstring = DBFReadStringAttribute( hDBF, rec, width_column );
    SG_LOG( SG_GENERAL, SG_DEBUG, "wstring = " << wstring );
    return atoi(wstring.c_str());
}

void processPolygon(SHPObject* psShape,
		    const string& work_dir,
		    AreaType area,
		    bool preserve3D) {
    int iPart,j;
    TGPolygon shape;
#if 0
    const char *pszPlus;
    const char *pszPartType = SHPPartTypeName( psShape->panPartType[0] );
#endif

    shape.erase();
    for ( j = 0, iPart = 1; j < psShape->nVertices; j++ ) {

	if( iPart < psShape->nParts
	    && psShape->panPartStart[iPart] == j )
	{
	    iPart++;
#if 0
	    pszPartType = SHPPartTypeName( psShape->panPartType[iPart] );
	    pszPlus = "+";
	} else {
	    pszPlus = " ";
#endif
	}

	shape.add_node( iPart - 1, 
			Point3D(psShape->padfX[j],
				psShape->padfY[j],
				psShape->padfZ[j])
			);
#if 0
	printf("%d %d %s (%12.3f,%12.3f, %g, %g) %s \n",
	       iPart, j,
	       pszPlus,
	       psShape->padfX[j],
	       psShape->padfY[j],
	       psShape->padfZ[j],
	       psShape->padfM[j],
	       pszPartType );
#endif
    }
    
    // check/set hole status for each contour.  negative area
    // means counter clockwise winding indicating the ring/contour
    // is a hole.
    for ( int i = 0; i < shape.contours(); ++i ) {
	double area = shape.area_contour( i );
	if ( area > 0 ) {
	    cout << "contour " << i << " = area" << endl;
	    shape.set_hole_flag( i, false );
	} else {
	    cout << "contour " << i << " = hole" << endl;
	    shape.set_hole_flag( i, true );
	}
    }

    if ( max_segment > 1.0 ) {
        shape = tgPolygonSplitLongEdges( shape, max_segment );
    }
    tgChopNormalPolygon(work_dir, area, shape, preserve3D);
}

void processLine(SHPObject* psShape,
		 const string& work_dir,
		 AreaType area,
		 int linewidth) {
    int iPart,j=0,partEnd=psShape->nVertices;

    for (iPart=psShape->nParts-1;iPart>=0;iPart--) {
	tg::Line line;
	TGPolygon shape;
#if 0
	SG_LOG( SG_GENERAL, SG_DEBUG,
		   "Part " << iPart
		<< " type = " << SHPPartTypeName(psShape->panPartType[iPart])
		<< " start = " << psShape->panPartStart[iPart]
		<< " end = " << partEnd
		<< " # of points = " << partEnd-psShape->panPartStart[iPart]);
#endif

	if (partEnd-psShape->panPartStart[iPart]<=1) {
	    SG_LOG( SG_GENERAL, SG_WARN,
	    	    "Skipping line with less than two points" );
	    continue;
	}
	shape.erase();
	for (j=psShape->panPartStart[iPart];j<partEnd;j++) {
#if 0
	    SG_LOG( SG_GENERAL, SG_DEBUG,
	    	       "   Point " << j << " ("
		    << psShape->padfX[j] << ", "
		    << psShape->padfY[j] << ")");
#endif
	    line.addPoint(Point3D(psShape->padfX[j],psShape->padfY[j],0));
	}
	partEnd=psShape->panPartStart[iPart];
	tg::makePolygon(line,linewidth,shape);

        if ( max_segment > 1.0 ) {
            shape = tgPolygonSplitLongEdges( shape, max_segment );
        }
	tgChopNormalPolygon(work_dir, area, shape, false);
    }
}

void processPoints(SHPObject* psShape,
		   const string& work_dir,
		   AreaType area,
		   int pointwidth) {
    TGPolygon shape;
    int j;

    for (j=0;j<psShape->nVertices;j++) {
#if 0
	SG_LOG( SG_GENERAL, SG_DEBUG,
		   "   Point ("
		<< psShape->padfX[j] << ", "
		<< psShape->padfY[j] << ")");
#endif
	shape.erase();
	tg::makePolygon(Point3D(psShape->padfX[j],psShape->padfY[j],0),
			pointwidth,
			shape);

        if ( max_segment > 1.0 ) {
            shape = tgPolygonSplitLongEdges( shape, max_segment );
        }
	tgChopNormalPolygon(work_dir, area, shape, false);
    }
}

void usage(char* progname) {
    SG_LOG( SG_GENERAL, SG_ALERT, "Usage: " << progname 
	    << " [--line-width width] [--point-width width]"
	       " [--area-column col] [--code-col col]"
               " [--line-width-column col ] "
	       " [--continue-on-errors] [--max-segment max_segment_length]"
	       " [--start-record num]"
	       " <shape_file> <work_dir> [ area_string ]" );
    SG_LOG( SG_GENERAL, SG_ALERT, "Options:" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--line-width width" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Width in meters for the lines" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--point-width width" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Size in meters of the squares generated from points" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--max-segment max_segment_length" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Maximum segment length in meters" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--area-column col" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Get areatype for objects from column number col" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        in associated dbf-file (only if <area_string> is not given)" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--code-column col" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Get codetype for objects from column number col" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        in associated dbf-file" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        (currently code is only used in debug printouts)" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--continue-on-errors" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Continue even if the file seems fishy" );
    SG_LOG( SG_GENERAL, SG_ALERT, "" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--start-record" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Start processing at the specified record number (first record num=0)" );
    SG_LOG( SG_GENERAL, SG_ALERT, "" );
    SG_LOG( SG_GENERAL, SG_ALERT, "<shape_file>" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Name of the shape-file to process, without .shp extension" );
    SG_LOG( SG_GENERAL, SG_ALERT, "<work_dir>" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Directory to put the polygon files in" );
    SG_LOG( SG_GENERAL, SG_ALERT, "<area_string>" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        (Optional) Area type for all objects in file" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Overrides --area-column option if present" );
    exit(-1);
}

int main( int argc, char **argv ) {
    int i, j;
    int pointwidth = 500, linewidth = 50, force_linewidth = -1;
    int start_record = 0;
    char* progname=argv[0];
    SGPath programPath(progname);
    string force_area_type = "";

    sglog().setLogLevels( SG_ALL, SG_INFO );

    if (programPath.file()=="noaa-decode") {
	SG_LOG( SG_GENERAL, SG_INFO, "Entering noaa-decode mode" );
	area_column=2;
	code_column=1;
	load_noaa_area_codes();
    }

    while (argc>1) {
	if (!strcmp(argv[1],"--line-width")) {
	    if (argc<3)
		usage(progname);
	    force_linewidth=atoi(argv[2]);
	    argv+=2;
	    argc-=2;
	} else if (!strcmp(argv[1],"--point-width")) {
	    if (argc<3)
		usage(progname);
	    pointwidth=atoi(argv[2]);
	    argv+=2;
	    argc-=2;
	} else if (!strcmp(argv[1],"--max-segment")) {
	    if (argc<3)
		usage(progname);
	    max_segment=atoi(argv[2]);
	    argv+=2;
	    argc-=2;
	} else if (!strcmp(argv[1],"--area-column")) {
	    if (argc<3)
		usage(progname);
	    area_column=atoi(argv[2]);
	    argv+=2;
	    argc-=2;
	} else if (!strcmp(argv[1],"--code-column")) {
	    if (argc<3)
		usage(progname);
	    code_column=atoi(argv[2]);
	    argv+=2;
	    argc-=2;
	} else if (!strcmp(argv[1],"--continue-on-errors")) {
            argv++;
            argc--;
	    continue_on_errors=1;
	} else if (!strcmp(argv[1],"--line-width-column")) {
	    if (argc<3)
		usage(progname);
	    width_column=atoi(argv[2]);
	    argv+=2;
	    argc-=2;
	} else if (!strcmp(argv[1],"--start-record")) {
	    if (argc<3)
		usage(progname);
	    start_record=atoi(argv[2]);
	    argv+=2;
	    argc-=2;
	} else if (!strcmp(argv[1],"--help")) {
	    usage(progname);
	} else
		break;
    }
    
    if (argc<3)
	usage(progname);
    
    SG_LOG( SG_GENERAL, SG_DEBUG, "Opening " << argv[1] << " for reading." );

    // make work directory
    string work_dir = argv[2];

#ifdef _MSC_VER
    fg_mkdir( work_dir.c_str() );
#else
    string command = "mkdir -p " + work_dir;
    system( command.c_str() );
#endif

    // allow us to override the area type from the command line.  All
    // polygons in the processed shape file will be assigned this area
    // type
    if ( argc == 4 ) {
	force_area_type = argv[3];
    }
	
    // initialize persistant polygon counter
    string counter_file = work_dir + "/../poly_counter";
    poly_index_init( counter_file );

    string dbffile = argv[1];
    dbffile += ".dbf";
    DBFHandle hDBF = DBFOpen( dbffile.c_str(), "rb" );
    if( hDBF == NULL ) {
	SG_LOG( SG_GENERAL, SG_ALERT, "DBFOpen(" << dbffile
		<< ",\"rb\") failed." );
        exit( -1 );
    }

    string shpfile = argv[1];
    shpfile += ".shp";
    SHPHandle hSHP = SHPOpen( shpfile.c_str(), "rb" );
    if( hSHP == NULL ) {
	SG_LOG( SG_GENERAL, SG_ALERT, "SHPOpen(" << shpfile
		<< ",\"rb\") failed." );
        exit( -1 );
    }

    int nShapeType, nEntities;
    double adfMinBound[4], adfMaxBound[4];
    SHPGetInfo( hSHP, &nEntities, &nShapeType, adfMinBound, adfMaxBound );

    SG_LOG( SG_GENERAL, SG_INFO, "shape file records = " << nEntities << endl );

    if ( nShapeType != SHPT_POINT &&
         nShapeType != SHPT_POINTM &&
         nShapeType != SHPT_MULTIPOINT &&
         nShapeType != SHPT_MULTIPOINTM &&
         nShapeType != SHPT_ARC &&
         nShapeType != SHPT_ARCM &&
         nShapeType != SHPT_POLYGON &&
         nShapeType != SHPT_POLYGONM &&
         nShapeType != SHPT_POLYGONZ) {
	SG_LOG( SG_GENERAL, SG_ALERT,
		"Can only handle 2D-Points, 2D-Multipoints, "
		"2D-Arcs and Polygons (2D and 3D)" );
	exit(-1);
    }

    for ( i = start_record; i < nEntities; i++ ) {
	// fetch i-th record (shape)
        SHPObject *psShape;

        psShape = SHPReadObject( hSHP, i );

	if (psShape->nSHPType==0) {
	    SG_LOG( SG_GENERAL, SG_DEBUG, "Skipping NULL record " << i);
	    continue;
	}
	
	SG_LOG( SG_GENERAL, SG_INFO, "Processing record = " << i 
		<< "  rings = " << psShape->nParts
		<< "  total vertices = " << psShape->nVertices );

	if (psShape->nParts<0) {
	    SG_LOG( SG_GENERAL, SG_ALERT,
	    	    "Record with negative part count, "
		    "file may be corrupted!");
	    if (continue_on_errors)
		continue;
	    else
		exit(-1);
	}
	
	if (psShape->nParts==0 &&
	    psShape->nSHPType!=SHPT_POINT &&
	    psShape->nSHPType!=SHPT_POINTM &&
	    psShape->nSHPType!=SHPT_MULTIPOINT &&
	    psShape->nSHPType!=SHPT_MULTIPOINTM) {
	    SG_LOG( SG_GENERAL, SG_ALERT,
	    	    "Non-point record with zero part count, "
		    "file may be corrupted!");
	    if (continue_on_errors)
		continue;
	    else
		exit(-1);
	}
	
	AreaType area = DefaultArea;
	if ( force_area_type.length() == 0 ) {
	    area = get_shapefile_type(hDBF, i);
	    SG_LOG( SG_GENERAL, SG_DEBUG, "  area type = " 
		    << get_area_name(area) << " (" << (int)area << ")" );
	} else {
	    area = get_area_type( force_area_type );
	}
      
    if ( force_linewidth == -1) {
      if (width_column != -1) // line width from shape file
        linewidth = get_shapefile_width(hDBF, i);
    } else
      linewidth = force_linewidth;

	SG_LOG( SG_GENERAL, SG_DEBUG, "  record type = " 
		<< SHPTypeName(psShape->nSHPType) );
	SG_LOG( SG_GENERAL, SG_DEBUG, "  bounds = (" 
		<< psShape->dfXMin << "," << psShape->dfYMin << ")  "
		<< psShape->dfZMin << "," <<  psShape->dfMMin
		<< " to (" << psShape->dfXMax << "," << psShape->dfYMax << ")  "
		<< psShape->dfZMax << "," << psShape->dfMMax );

#if 0
        printf( "\nShape:%d (%s)  nVertices=%d, nParts=%d\n"
                "  Bounds:(%12.3f,%12.3f, %g, %g)\n"
                "      to (%12.3f,%12.3f, %g, %g)\n",
                i, SHPTypeName(psShape->nSHPType),
                psShape->nVertices, psShape->nParts,
                psShape->dfXMin, psShape->dfYMin,
                psShape->dfZMin, psShape->dfMMin,
                psShape->dfXMax, psShape->dfYMax,
                psShape->dfZMax, psShape->dfMMax );
#endif

	if ( area == OceanArea ) {
	    // interior of polygon is ocean, holes are islands

	    SG_LOG(  SG_GENERAL, SG_ALERT, "Ocean area ... SKIPPING!" );

	    // Ocean data now comes from GSHHS so we want to ignore
	    // all other ocean data
	    continue;
	} else if ( area == VoidArea ) {
	    // interior is ????

	    // skip for now
	    SG_LOG(  SG_GENERAL, SG_ALERT, "Void area ... SKIPPING!" );

	    if ( psShape->nParts > 1 ) {
		SG_LOG(  SG_GENERAL, SG_ALERT, "  Void area with holes!" );
		// exit(-1);
	    }
	    
	    continue;
	} else if ( area == NullArea ) {
	    // interior is ????

	    // skip for now
	    SG_LOG(  SG_GENERAL, SG_ALERT, "Null area ... SKIPPING!" );

	    if ( psShape->nParts > 1 ) {
		SG_LOG(  SG_GENERAL, SG_ALERT, "  Null area with holes!" );
		// exit(-1);
	    }

	    continue;
	}

	if ( force_area_type.length() > 0 ) {
	    // interior of polygon is assigned to force_area_type,
	    // holes are preserved

	    area = get_area_type( force_area_type );
	}
	
	switch (psShape->nSHPType) {
	case SHPT_POLYGON:
	case SHPT_POLYGONM:
		/* 2D Polygons */
		processPolygon(psShape,work_dir,area,false);
		break;
	case SHPT_POLYGONZ:
		/* 3D Polygons */
		processPolygon(psShape,work_dir,area,true);
		break;
	case SHPT_ARC:
	case SHPT_ARCM:
		/* 2D Lines */
		processLine(psShape,work_dir,area,linewidth);
		break;
	case SHPT_POINT:
	case SHPT_POINTM:
	case SHPT_MULTIPOINT:
	case SHPT_MULTIPOINTM:
		/* 2D Points */
		processPoints(psShape,work_dir,area,pointwidth);
		break;
	default:
		SG_LOG( SG_GENERAL, SG_WARN,
			"Can't handle "
			<< SHPTypeName(psShape->nSHPType)
			<< " yet - skipping" );
		break;
	}
	
        SHPDestroyObject( psShape );

    }

    DBFClose( hDBF );
    SHPClose( hSHP );

    return 0;
}



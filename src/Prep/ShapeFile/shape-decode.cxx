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
// $Id: shape-decode.cxx,v 1.17 2007-11-05 21:58:59 curt Exp $
 

#include <simgear/compiler.h>

#include STL_STRING
#include <map>

#include <simgear/debug/logstream.hxx>
#include <simgear/math/sg_geodesy.hxx>
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
bool use_area_code_map = false;
int area_column = 4, code_column = 3;
double max_segment = 0.0;  // zero = no splitting
int width_column = -1;
bool ivmap = false;
double line_width_pad = 0.0;

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
    use_area_code_map = true;
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

    if ( use_area_code_map ) {
	area = area_code_map[area];
	SG_LOG( SG_GENERAL, SG_INFO, "cooked area = " << area );
    }

    if ( ivmap ) {
        if ( area == "40" ) {
            area = "WhiteLine";
        } else if ( area == "156" ) {
            area = "YellowLine";
        } else {
            SG_LOG( SG_GENERAL, SG_ALERT,
                    "Unknown ivmap area type = " << area );
        }
    }

    return get_area_type( area );
}

// get attribute value of 'width' from shapefile
float get_shapefile_width(DBFHandle& hDBF, int rec) {
    string wstring = DBFReadStringAttribute( hDBF, rec, width_column );
    SG_LOG( SG_GENERAL, SG_DEBUG, "width string = " << wstring );
    double width = atof(wstring.c_str());
    return width + line_width_pad;
}

// return an arbitrary shape file attribute as a string
string get_attribute(DBFHandle& hDBF, int rec, int column) {
 
    string attrib = DBFReadStringAttribute( hDBF, rec, column );

    // strip leading spaces
    while ( attrib[0] == ' ' ) {
	attrib = attrib.substr(1, attrib.length() - 1);
    }
    // strip trailing spaces
    while ( attrib[attrib.length() - 1] == ' ' ) {
	attrib = attrib.substr(0, attrib.length() - 1);
    }
    // strip other junk encountered
    while ( (int)attrib[attrib.length() - 1] == 9 ) {
	attrib = attrib.substr(0, attrib.length() - 1);
    }

    return attrib;
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


// Calculate theta of angle (a, b, c)
double calc_angle(Point3D a, Point3D b, Point3D c) {
    Point3D u, v;
    double udist, vdist, uv_dot, tmp;

    // u . v = ||u|| * ||v|| * cos(theta)

    u.setx( b.x() - a.x() );
    u.sety( b.y() - a.y() );
    udist = sqrt( u.x() * u.x() + u.y() * u.y() );
    // printf("udist = %.6f\n", udist);

    v.setx( b.x() - c.x() );
    v.sety( b.y() - c.y() );
    vdist = sqrt( v.x() * v.x() + v.y() * v.y() );
    // printf("vdist = %.6f\n", vdist);

    uv_dot = u.x() * v.x() + u.y() * v.y();
    // printf("uv_dot = %.6f\n", uv_dot);

    tmp = uv_dot / (udist * vdist);
    // printf("tmp = %.6f\n", tmp);

    return acos(tmp);
}


// given an input line, try to identify locations in the line where
// the data doubles back on itself and remove those (IVLab data glitch
// work around)
tg::Line fixDegenerateLine1( tg::Line line ) {
    tg::Line result;

    int size = line.getPointCount();
    if ( size < 3 ) {
        return line;
    }

    // prime the pump, and copy over the starting point
    Point3D a = line.getPoint(0);
    Point3D b = line.getPoint(1);
    result.addPoint( a );

    for ( int i = 1; i < size - 1; i++ ) {
        Point3D c = line.getPoint(i+1);
        double angle = calc_angle( a, b, c );
        if ( angle < 0 ) {
            cout << "OOPS, negative angle!!!!  PLEASE CHECK ME!!!" << endl;
        }
        if ( angle > SGD_PI_2 ) {
            // good angle, march forward (small angles indicate the
            // line is doubling back on itself)
            result.addPoint(b);
            a = b;
            b = c;
        } else {
            // degenerate angle, skip point c and try next point
            printf("IVLab data glitch at = %.8f %.8f\n", b.lon(), b.lat());
        }
    }

    // copy the final point.
    result.addPoint( line.getPoint(size-1) );

    return result;
}


// given an input line, try to identify locations in the line where
// the points are packed really tightly and remove those (IVLab data glitch
// work around)
tg::Line fixDegenerateLine2( tg::Line line ) {
    const double min_segment_length_m = 10.0;
    tg::Line result;

    int size = line.getPointCount();
    if ( size <= 2 ) {
        return line;
    }

    // prime the pump, and copy over the starting point
    Point3D a = line.getPoint(0);
    result.addPoint( a );

    for ( int i = 1; i < size - 1; i++ ) {
        Point3D b = line.getPoint(i);
        double az1, az2, dist;
        geo_inverse_wgs_84( 0.0, a.y(), a.x(), b.y(), b.x(),
                            &az1, &az2, &dist );
        if ( dist > min_segment_length_m ) {
            // distance ok, march forward (small distances could
            // indicate clusters of points that might cause problems.
            result.addPoint(b);
            a = b;
        } else {
            // distance too small, skip point b and try next point
            printf("IVLab data glitch at = %.8f %.8f\n", b.lon(), b.lat());
        }
    }

    // add the last point (we always want the first and last points of
    // the line to minimize visual glitches
    result.addPoint( line.getPoint(size-1) );

    return result;
}


// given an input line, avoid a situation where there is a completely
// loop, i.e. after a long route, the road loops around and connects
// up with itself.  Back off from the end of the line until there is a
// small gap of more than 1 meter.
tg::Line fixDegenerateLine3( tg::Line line ) {
    tg::Line result;

    int size = line.getPointCount();
    if ( size <= 2 ) {
        return line;
    }

    // prime the pump
    Point3D a = line.getPoint(0);
    int end = size - 1;

    for ( int i = size - 1; i > 1; i-- ) {
        end = i;
        Point3D b = line.getPoint(end);
        double az1, az2, dist;
        geo_inverse_wgs_84( 0.0, a.y(), a.x(), b.y(), b.x(),
                            &az1, &az2, &dist );
        if ( dist > 2.0 ) {
            // distance ok, go ahead and exit
            break;
        } else {
            // distance too small, march backwards one step and try
            // the previous point
            printf("IVLab data loop at = %.8f %.8f\n", b.lon(), b.lat());
        }
    }

    for ( int i = 0; i <= end; i++ ) {
        result.addPoint( line.getPoint(i) );
    }

    return result;
}


void processLine(SHPObject* psShape,
		 const string& work_dir,
		 AreaType area,
		 float linewidth) {
    int iPart,j=0,partEnd=psShape->nVertices;
    double minx = 200, miny = 200, maxx = -200, maxy = -200;
    for (iPart=psShape->nParts-1;iPart>=0;iPart--) {
	tg::Line line;
	TGPolygon shape;
        // #if 0
	SG_LOG( SG_GENERAL, SG_DEBUG,
		   "In processLine() Part " << iPart
		<< " type = " << SHPPartTypeName(psShape->panPartType[iPart])
		<< " start = " << psShape->panPartStart[iPart]
		<< " end = " << partEnd
		<< " # of points = " << partEnd-psShape->panPartStart[iPart]);
        // #endif

	if (partEnd-psShape->panPartStart[iPart]<=1) {
	    SG_LOG( SG_GENERAL, SG_WARN,
	    	    "Skipping line with less than two points" );
	    continue;
	}
	shape.erase();
	for (j=psShape->panPartStart[iPart];j<partEnd;j++) {
            //#if 0
	    SG_LOG( SG_GENERAL, SG_DEBUG,
	    	       "   Point " << j << " ("
		    << psShape->padfX[j] << ", "
		    << psShape->padfY[j] << ")");
            if ( psShape->padfX[j] < minx ) { minx = psShape->padfX[j]; }
            if ( psShape->padfX[j] > maxx ) { maxx = psShape->padfX[j]; }
            if ( psShape->padfY[j] < miny ) { miny = psShape->padfY[j]; }
            if ( psShape->padfY[j] > maxy ) { maxy = psShape->padfY[j]; }
            //#endif
	    line.addPoint(Point3D(psShape->padfX[j],psShape->padfY[j],0));
	}
        SG_LOG( SG_GENERAL, SG_DEBUG, "(" << minx << "," << miny << ") (" << maxx << "," << maxy << ")" );

        if ( ivmap ) {
            cout << "original line size = " << line.getPointCount() << endl;
            line = fixDegenerateLine1( line );
            cout << "angle line size = " << line.getPointCount() << endl;
            line = fixDegenerateLine2( line );
            cout << "dist line size = " << line.getPointCount() << endl;
            line = fixDegenerateLine3( line );
            cout << "cycle line size = " << line.getPointCount() << endl;
        }

	partEnd=psShape->panPartStart[iPart];
	tg::makePolygon(line,linewidth,shape);

        if ( max_segment > 1.0 ) {
            shape = tgPolygonSplitLongEdges( shape, max_segment );
        }
        cout << "hole flag = " << shape.get_hole_flag(0) << endl;
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
    SG_LOG( SG_GENERAL, SG_ALERT, "--start-record record-number" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        Start processing at the specified record number (first record num=0)" );
    SG_LOG( SG_GENERAL, SG_ALERT, "--end-record record-number" );
    SG_LOG( SG_GENERAL, SG_ALERT, "        End processing at (immediately after) the specified record number (first record num=0)" );
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
    int i;
    int pointwidth = 500;
    float force_linewidth = -1, linewidth = 50.0;
    int start_record = 0, end_record = -1;
    char* progname=argv[0];
    SGPath programPath(progname);
    string force_area_type = "";
    bool continue_on_errors = false;

    sglog().setLogLevels( SG_ALL, SG_DEBUG );

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
	    force_linewidth=atof(argv[2]);
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
	    continue_on_errors=true;
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
	} else if (!strcmp(argv[1],"--end-record")) {
	    if (argc<3)
		usage(progname);
	    end_record=atoi(argv[2]);
	    argv+=2;
	    argc-=2;
        } else if (!strcmp(argv[1],"--ivlanemarkings")) {
            argv++;
            argc--;
            ivmap = true;
            width_column = 6;
        } else if (!strcmp(argv[1],"--ivlane")) {
            argv++;
            argc--;
            ivmap = true;
            width_column = 3;
        } else if (!strcmp(argv[1],"--pad-width")) {
            line_width_pad = atof(argv[2]);
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
         nShapeType != SHPT_POLYGONZ)
    {
	SG_LOG( SG_GENERAL, SG_ALERT,
		"Can only handle 2D-Points, 2D-Multipoints, "
		"2D-Arcs and Polygons (2D and 3D)" );
	exit(-1);
    }

    if (end_record<0 || end_record>=nEntities)
            end_record=nEntities-1;
    if (start_record<0)
            start_record=0;
    if (start_record>nEntities)
            start_record=nEntities;
    
    for ( i = start_record; i <= end_record; i++ ) {
	// fetch i-th record (shape)
        SHPObject *psShape;

        psShape = SHPReadObject( hSHP, i );

	if (psShape->nSHPType==0) {
	    SG_LOG( SG_GENERAL, SG_DEBUG, "Skipping NULL record " << i);
            SHPDestroyObject( psShape );
	    continue;
	}
	
	SG_LOG( SG_GENERAL, SG_INFO, "Processing record = " << i 
		<< "  rings = " << psShape->nParts
		<< "  total vertices = " << psShape->nVertices );

	if (psShape->nParts<0) {
	    SG_LOG( SG_GENERAL, SG_ALERT,
	    	    "Record with negative part count, "
		    "file may be corrupted!");
            if (continue_on_errors) {
                SHPDestroyObject( psShape );
		continue;
            } else
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
            if (continue_on_errors) {
                SHPDestroyObject( psShape );
		continue;
            } else
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
      
        if ( force_linewidth < 0 ) {
            if (width_column != -1) // line width from shape file
                linewidth = get_shapefile_width(hDBF, i);
        } else
            linewidth = force_linewidth;

        if ( ivmap ) {
            // get skip type
            string skip = get_attribute(hDBF, i, 7);

            // FIXME: implement skip logic
        }

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
            SHPDestroyObject( psShape );
	    continue;
	} else if ( area == VoidArea ) {
	    // interior is ????

	    // skip for now
	    SG_LOG(  SG_GENERAL, SG_ALERT, "Void area ... SKIPPING!" );

	    if ( psShape->nParts > 1 ) {
		SG_LOG(  SG_GENERAL, SG_ALERT, "  Void area with holes!" );
		// exit(-1);
	    }
	    SHPDestroyObject( psShape );
	    continue;
	} else if ( area == NullArea ) {
	    // interior is ????

	    // skip for now
	    SG_LOG(  SG_GENERAL, SG_ALERT, "Null area ... SKIPPING!" );

	    if ( psShape->nParts > 1 ) {
		SG_LOG(  SG_GENERAL, SG_ALERT, "  Null area with holes!" );
		// exit(-1);
	    }
            SHPDestroyObject( psShape );
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



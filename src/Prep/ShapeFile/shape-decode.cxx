// main.cxx -- process shapefiles and extract polygon outlines,
//             clipping against and sorting them into the revelant
//             tiles.
//
// Written by Curtis Olson, started February 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - curt@flightgear.org
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

#include <simgear/debug/logstream.hxx>

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
 
    string area = DBFReadStringAttribute( hDBF, rec, 4 );
    string test = DBFReadStringAttribute( hDBF, rec, 3 );
    cout << "next record = " << test << endl;

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

    SG_LOG( SG_GENERAL, SG_INFO, "  raw area = " << area );

    return get_area_type( area );
}


int main( int argc, char **argv ) {
    TGPolygon shape;
    int i, j;

    sglog().setLogLevels( SG_ALL, SG_DEBUG );

    if ( argc < 3 ) {
	SG_LOG( SG_GENERAL, SG_ALERT, "Usage: " << argv[0] 
		<< " <shape_file> <work_dir> [ area_string ]" );
	exit(-1);
    }

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
    string force_area_type = "";
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

    string shapetype = SHPTypeName( nShapeType );

    if ( shapetype != string("Polygon") ) {
	SG_LOG( SG_GENERAL, SG_ALERT, "Can't handle non-polygon shape files" );
	exit(-1);
    }

    int iPart;
    const char *pszPlus;
    for ( i = 0; i < nEntities; i++ ) {
	// fetch i-th record (shape)
        SHPObject *psShape;
 
	shape.erase();

        psShape = SHPReadObject( hSHP, i );

	SG_LOG( SG_GENERAL, SG_DEBUG, "Processing record = " << i 
		<< "  rings = " << psShape->nParts
		<< "  total vertices = " << psShape->nVertices );

	AreaType area = DefaultArea;
	if ( force_area_type.length() == 0 ) {
	    area = get_shapefile_type(hDBF, i);
	    SG_LOG( SG_GENERAL, SG_DEBUG, "  area type = " 
		    << get_area_name(area) << " (" << (int)area << ")" );
	} else {
	    area = get_area_type( force_area_type );
	}

	SG_LOG( SG_GENERAL, SG_INFO, "  record type = " 
		<< SHPTypeName(psShape->nSHPType) );
	SG_LOG( SG_GENERAL, SG_INFO, "  bounds = (" 
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

        for ( j = 0, iPart = 1; j < psShape->nVertices; j++ ) {
            const char *pszPartType = "";
 
            if ( j == 0 && psShape->nParts > 0 ) {
                pszPartType = SHPPartTypeName( psShape->panPartType[0] );
	    }
            
            if( iPart < psShape->nParts
                && psShape->panPartStart[iPart] == j )
            {
                pszPartType = SHPPartTypeName( psShape->panPartType[iPart] );
                iPart++;
                pszPlus = "+";
            } else {
                pszPlus = " ";
	    }
 
	    shape.add_node( iPart - 1, 
			    Point3D(psShape->padfX[j], psShape->padfY[j], 0)
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
	
        SHPDestroyObject( psShape );

	if ( force_area_type.length() > 0 ) {
	    // interior of polygon is assigned to force_area_type,
	    // holes are preserved

	    area = get_area_type( force_area_type );
	    tgChopPolygon(work_dir, area, shape, false);
	} else if ( area == OceanArea ) {
	    // interior of polygon is ocean, holes are islands

	    SG_LOG(  SG_GENERAL, SG_ALERT, "Ocean area ... SKIPPING!" );

	    // Ocean data now comes from GSHHS so we want to ignore
	    // all other ocean data
	    // tgChopPolygon(work_dir, area, shape, false);
	} else if ( area == VoidArea ) {
	    // interior is ????

	    // skip for now
	    SG_LOG(  SG_GENERAL, SG_ALERT, "Void area ... SKIPPING!" );

	    if ( shape.contours() > 1 ) {
		SG_LOG(  SG_GENERAL, SG_ALERT, "  Void area with holes!" );
		// exit(-1);
	    }

	    // tgChopPolygon(work_dir, area, shape, false);
	} else if ( area == NullArea ) {
	    // interior is ????

	    // skip for now
	    SG_LOG(  SG_GENERAL, SG_ALERT, "Null area ... SKIPPING!" );

	    if ( shape.contours() > 1 ) {
		SG_LOG(  SG_GENERAL, SG_ALERT, "  Null area with holes!" );
		// exit(-1);
	    }

	    // tgChopPolygon(work_dir, area, shape, false);
	} else {
	    tgChopPolygon(work_dir, area, shape, false);
	}
    }

    DBFClose( hDBF );
    SHPClose( hSHP );

    return 0;
}



//////////////////////////////////////////////////////////////////////////////////////////////
/////                           filename:  airspace.cxx                                 //////
//////////////////////////////////////////////////////////////////////////////////////////////
//////                                                                                  //////
//////               airspace: generates files for 3d depiction of airspace             //////
//////               for flighgear                                                      //////
//////               version 0.2 released 2006-04-12                                    //////
//////               by Philip Cobbin                                                   //////
//////                                                                                  //////
//////               running:  ./airspace US                                            //////
//////                         report files are generated for the various types of      //////
//////                         data processed:                                          //////
//////                                                                                  //////
//////                         summary-airport.txt                                      //////
//////                         summary-airspace.txt                                     //////
//////                         summary-ils.txt                                          //////
//////                         summary-navaid.txt                                       //////
//////                         summary-suas.txt                                         //////
//////                         summary-waypoint.txt                                     //////
//////                         tile_pathlist.txt                                        //////
//////                                                                                  //////
//////               see the readme file in the support files folder for where to put   //////
//////               the extra files for adding menu support and default configuration  //////
//////               values.                                                            //////
//////                                                                                  //////
//////               compiling:                                                         //////
//   g++ airspace.cxx  /usr/local/lib/libsgmagvar.a /usr/local/lib/libsgmath.a /usr/local/lib/libsgmisc.a /usr/local/lib/libsgdebug.a -o airspace      //
//////                                                                                  //////
//////                                                                                  //////
//////                            ---- NOTICE ----                                      //////
//////                                                                                  //////
//////     This program is free software; you can redistribute it and/or modify         //////
//////    it under the terms of the GNU General Public License as published by          //////
//////    the Free Software Foundation; either version 2 of the License, or             //////
//////    (at your option) any later version.                                           //////
//////                                                                                  //////
//////    This program is distributed in the hope that it will be useful,               //////
//////    but WITHOUT ANY WARRANTY; without even the implied warranty of                //////
//////    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                 //////
//////    GNU General Public License for more details.                                  //////
//////                                                                                  //////
//////    You should have received a copy of the GNU General Public License             //////
//////    along with this program; if not, write to the Free Software                   //////
//////    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA      //////
//////                                                                                  //////
//////    Philip Cobbin; AKA Sierra Simulations & Software can be reached at:           //////
//////    499 Grafton Turnpike, Canaan, New Hampshire, U.S.A 03741                      //////
//////    Website:  http://www.cobbin.com             e-mail:  cobbin@cobbin.com        //////
//////                                                                                  //////
//////                            ---- DAFIF NOTICE ----                                //////
//////                                                                                  //////
//////    This product was developed using DAFIF, a product of the                      //////  
//////    National  Geospatial-Intelligence Agency.                                     //////
//////                                                                                  ////// 
//////    This product has not been endorsed or otherwise approved by the National      //////
//////    Geospatial-Intelligence Agency or the former National Imagery and Mapping     //////
//////    Agency, or the United States Department of Defense (10 U.S.C. 425)            //////
//////                                                                                  //////
//////    a.  Under 10 U.S.C. 456, no civil action may be brought against the United    //////
//////    States on the basis of the content of a navigational aid prepared or          //////
//////    disseminated by either the former Defense Mapping Agency (DMA),               //////
//////    National Imagery and Mapping Agency (NIMA)or the National                     //////
//////    Geospatial-Intelligence Agency (NGA).                                         //////
//////                                                                                  //////
//////    b.  The DAFIF product is provided "as is," and no warranty, express or        //////
//////    implied, including, but not limited to the implied warranties of              //////   
//////    merchantability and fitness for particular purpose or arising by statute or   //////
//////    otherwise in law or from a course of dealing or usage in trade, is made by    //////
//////    NGA as to the accuracy and functioning of the product.                        //////
//////	                                                                                //////
//////    c.  Neither NGA nor its personnel will be liable for any claims, losses,      //////
//////    or damages arising from or connected with the use of this product.  The user  //////
//////    agrees to hold harmless the United States National Geospatial-Intelligence    //////
//////    Agency.  The user's sole and exclusive remedy is to stop using the DAFIF      //////
//////    product.                                                                      //////
//////                                                                                  //////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
                                                                      
#include <iostream.h>
#include <fstream.h>
#include <string>
#include <stdio.h>

#include <math.h>

#include <netinet/in.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

using std::string;

/******************* simgear ********************************/
#include <simgear/constants.h>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/misc/sg_path.hxx>
/************************************************************/


// Configure the following for your particular directory structure ...
const string dafift_base = "/usr/local/share/DAFIFT";        
const string output_base = "/usr/local/share/FlightGear/data/Scenery-Airspace";
const string short_output_base = "Scenery-Airspace";

//const string output_base = "/usr/local/share/FlightGear/data/Scenery-9.10";
const string airspace_path = "/usr/local/share/FlightGear/data/Models/Airspace";

const string airspace_texture_path = "/usr/local/share/FlightGear/data/Textures/Airspace";


/**************** simgear hybred of newbucket ***************/


/*  routines modified from simgear to get tile numbers right */

    int lon;        // longitude index (-180 to 179)
    int lat;        // latitude index (-90 to 89)
    int x;          // x subdivision (0 to 7)
    int y;          // y subdivision (0 to 7)


// return the horizontal tile span factor based on latitude
double sg_bucket_span( double l ) {
    if ( l >= 89.0 ) {
	return 360.0;
    } else if ( l >= 88.0 ) {
	return 8.0;
    } else if ( l >= 86.0 ) {
	return 4.0;
    } else if ( l >= 83.0 ) {
	return 2.0;
    } else if ( l >= 76.0 ) {
	return 1.0;
    } else if ( l >= 62.0 ) {
	return 0.5;
    } else if ( l >= 22.0 ) {
	return 0.25;
    } else if ( l >= -22.0 ) {
	return 0.125;
    } else if ( l >= -62.0 ) {
	return 0.25;
    } else if ( l >= -76.0 ) {
	return 0.5;
    } else if ( l >= -83.0 ) {
	return 1.0;
    } else if ( l >= -86.0 ) {
	return 2.0;
    } else if ( l >= -88.0 ) {
	return 4.0;
    } else if ( l >= -89.0 ) {
	return 8.0;
    } else {
	return 360.0;
    }
}


// Set the bucket params for the specified lat and lon
void set_bucket( double dlon, double dlat ) {
    //
    // latitude first
    //
    double span = sg_bucket_span( dlat );
    double diff = dlon - (double)(int)dlon;

    // cout << "diff = " << diff << "  span = " << span << endl;

    if ( (dlon >= 0) || (fabs(diff) < SG_EPSILON) ) {
	lon = (int)dlon;
    } else {
	lon = (int)dlon - 1;
    }

    // find subdivision or super lon if needed
    if ( span < SG_EPSILON ) {
	// polar cap
	lon = 0;
	x = 0;
    } else if ( span <= 1.0 ) {
	x = (int)((dlon - lon) / span);
    } else {
	if ( (dlon >= 0) || (fabs(diff) < SG_EPSILON) ) {
	    lon = (int)( (int)(lon / span) * span);
	} else {
	    // cout << " lon = " << lon 
	    //  << "  tmp = " << (int)((lon-1) / span) << endl;
	    lon = (int)( (int)((lon + 1) / span) * span - span);
	    if ( lon < -180 ) {
		lon = -180;
	    }
	}
	x = 0;
    }

    //
    // then latitude
    //
    diff = dlat - (double)(int)dlat;

    if ( (dlat >= 0) || (fabs(diff) < SG_EPSILON) ) {
	lat = (int)dlat;
    } else {
	lat = (int)dlat - 1;
    }
    y = (int)((dlat - lat) * 8);
}

    /**
     * Generate the unique scenery tile index for this bucket
     *
     * The index is constructed as follows:
     * 
     * 9 bits - to represent 360 degrees of longitude (-180 to 179)
     * 8 bits - to represent 180 degrees of latitude (-90 to 89)
     *
     * Each 1 degree by 1 degree tile is further broken down into an 8x8
     * grid.  So we also need:
     *
     * 3 bits - to represent x (0 to 7)
     * 3 bits - to represent y (0 to 7)
     * @return tile index
     */

    long int gen_index() {
	return ((lon + 180) << 14) + ((lat + 90) << 6) + (y << 3) + x;
    }
// ptc addition 2006-03-01
// gen_tilenum is derived from set_bucket and gen_index...
// ptc 2006-03-01
long int gen_tilenum( double dlon, double dlat ) {
    //
    // latitude first
    //
    double span = sg_bucket_span( dlat );
    double diff = dlon - (double)(int)dlon;
    
    int lon;
    int lat;
    int x; 
    int y;

    // cout << "diff = " << diff << "  span = " << span << endl;

    if ( (dlon >= 0) || (fabs(diff) < SG_EPSILON) ) {
	lon = (int)dlon;
    } else {
	lon = (int)dlon - 1;
    }

    // find subdivision or super lon if needed
    if ( span < SG_EPSILON ) {
	// polar cap
	lon = 0;
	x = 0;
    } else if ( span <= 1.0 ) {
	x = (int)((dlon - lon) / span);
    } else {
	if ( (dlon >= 0) || (fabs(diff) < SG_EPSILON) ) {
	    lon = (int)( (int)(lon / span) * span);
	} else {
	    // cout << " lon = " << lon 
	    //  << "  tmp = " << (int)((lon-1) / span) << endl;
	    lon = (int)( (int)((lon + 1) / span) * span - span);
	    if ( lon < -180 ) {
		lon = -180;
	    }
	}
	x = 0;
    }

    //
    // then latitude
    //
    diff = dlat - (double)(int)dlat;

    if ( (dlat >= 0) || (fabs(diff) < SG_EPSILON) ) {
	lat = (int)dlat;
    } else {
	lat = (int)dlat - 1;
    }
    y = (int)((dlat - lat) * 8);
//    printf("x[%d]y[%d]",x,y);
    return ((lon + 180) << 14) + ((lat + 90) << 6) + (y << 3) + x;

}


/********************************************************************* globals ********************************************************/

// ------------------------------------------------DAFIF fields-----------------------------------------------------------

  char tc = '\t';
  char tc2= '\n';
  char header[1000];  // to read header into to skip over ....

/** boundary (country) record **/
  char bdry_ident[13];
  char bdry_ident_safe[13]; //safe as a filename with '-' replacing ' ' characters
  char bdry_ident_last[13];
  char seg_nbr[6];
  char ctry_1[5];
  char ctry_2[5];
  char ctry_3[5];
  char ctry_4[5];
  char ctry_5[5];
  char cycle_date[8];

// procedure to make a "safe" version of the bdry identifier for use as a string in a filename...
void set_safe_bdry()
{
  int i=0;
  for (i=0; i<sizeof(bdry_ident); i++) {
    bdry_ident_safe[i] = bdry_ident[i];
    if (bdry_ident[i]== ' ') bdry_ident_safe[i] = '-'; 
    if (bdry_ident[i]== '(') bdry_ident_safe[i] = '-'; 
    if (bdry_ident[i]== ')') bdry_ident_safe[i] = '-'; 
  }
//  if (strncmp(bdry_ident,bdry_ident_safe,sizeof(bdry_ident))!=0) printf("Boundary Identifier changed from [%s] to [%s]\n",bdry_ident, bdry_ident_safe);
}

  char airport_icao[5]; // for generating signs over airports...

/** special use SUAS fields that are different than boundary (country) ref. appendix IV**/
  char suas_sector[3];
  char suas_icao[5];

  char ptype[3];
  char pname[39];  
  char picao[5];
  char pcon_auth[39];
  char ploc_hdatum[4];
  char pwgs_datum[4];
  char pcomm_name[21];
  char pcomm_freq1[10];
  char pcomm_freq2[10];
  char pclass[2];
  char pclass_exc[2];
  char pclass_ex_rmk[81];
  char plevel[2];
  char pupper_alt[11];
  char plower_alt[10];
  char prnp[4];
  char pcycle_date[8];
/** special use SUAS fields that are different than boundary (parent) record **/

// suas_sector
  char suas_con_agcy[39];
  char suas_eff_times[39];
  char suas_wx[9];
  char suas_eff_date[10];

  char sseg_nbr[6];
  char sname[39];
  char stype[3];
  char sicao[5];
  char sshap[2];
  char sderivation[2];
  char swgs_lat1[10];
  char swgs_dlat1[11];
  char swgs_long1[11];
  char swgs_dlong1[12];
  char swgs_lat2[10];
  char swgs_dlat2[11];
  char swgs_long2[11];
  char swgs_dlong2[12];
  char swgs_lat0[10];
  char swgs_dlat0[11];
  char swgs_long0[11];
  char swgs_dlong0[12];
  char sradius1[7];
  char sradius2[7];
  char sbearing1[6];
  char sbearing2[6];
  char snav_ident[5];
  char snav_type[2];
  char snav_ctry[3];
  char snav_key_cd[3];
  char scycle_date[8];

//------------------------------------ airport information for setting glide slopes in tiles
  char arpt_ident[8]; 	//ARPT_IDENT	
  char name[39];	//NAME	
  char state_prov[3];	//STATE_PROV	
  char icao[5];		//ICAO	
  char faa_host_id[5];	//FAA_HOST_ID	
  char loc_hdatum[4]; 	//LOC_HDATUM	
  char wgs_datum[4];	//WGS_DATUM	
  char wgs_lat[10];	//WGS_LAT
  char wgs_dlat[11];	//WGS_DLAT	
  char wgs_long[11];	//WGS_LONG	
  char wgs_dlong[12];	//WGS_DLONG	
  char elev[6];		//ELEV	
  char type[2];		//TYPE	
  char mag_var[13];	//MAG_VAR  more like 7+1=8
  char wac[5];		//WAC	
  char beacon[2];	//BEACON	
  char second_arpg[2];	//SECOND_ARPT	
  char opr_agy[3];	//OPR_AGY	
  char sec_name[39];	//SEC_NAME	
  char sec_icao[5];	//SEC_ICAO	
  char sec_faa[5];	//SEC_FAA	
  char sec_opr_agy[3];	//SEC_OPR_AGY	
  char acycle_date[8];	//CYCLE_DATE	
  char terrain[2];	//TERRAIN  looks like y/n unknown what it is	
  char hydro[2];	//HYDRO  looks like y/n unknown content

///-------------------------------------- runway information
//	ARPT_IDENT
  char high_ident[4];		//	HIGH_IDENT
  char low_ident[4];		//	LOW_IDENT
  char high_hdg[6];		//	HIGH_HDG
  char low_hdg[6];		//	LOW_HDG
  char rwy_length[6];		//	LENGTH
  char rwy_width[6];		//	RWY_WIDTH
  char surface[4];		//	SURFACE
  char pcn[8];			//	PCN
  char he_wgs_lat[10];		//	HE_WGS_LAT
  char he_wgs_dlat[11];		//	HE_WGS_DLAT
  char he_wgs_long[11];		//	HE_WGS_LONG
  char he_wgs_dlong[12];	//	HE_WGS_DLONG
  char he_elev[8];		//	HE_ELEV
  char he_slope[5];		//	HE_SLOPE
  char he_tdze[8];		//	HE_TDZE
  char he_dt[5];		//	HE_DT
  char he_dt_elev[8];		//	HE_DT_ELEV
  char hlgt_sys_1[3];		//	HLGT_SYS_1
  char hlgt_sys_2[3];		//	HLGT_SYS_2
  char hlgt_sys_3[3];		//	HLGT_SYS_3
  char hlgt_sys_4[3];		//	HLGT_SYS_4
  char hlgt_sys_5[3];		//	HLGT_SYS_5
  char hlgt_sys_6[3];		//	HLGT_SYS_6
  char hlgt_sys_7[3];		//	HLGT_SYS_7
  char hlgt_sys_8[3];		//	HLGT_SYS_8
  char le_wgs_lat[10];		//	LE_WGS_LAT
  char le_wgs_dlat[11];		//	LE_WGS_DLAT
  char le_wgs_long[11];		//	LE_WGS_LONG
  char le_wgs_dlong[12];	//	LE_WGS_DLONG
  char le_elev[8];		//	LE_ELEV
  char le_slope[5];		//	LE_SLOPE
  char le_tdze[8];		//	LE_TDZE
  char le_dt[5];		//	LE_DT
  char le_dt_elev[8];		//	LE_DT_ELEV
  char llgt_sys_1[3];		//	LLGT_SYS_1
  char llgt_sys_2[3];		//	LLGT_SYS_2
  char llgt_sys_3[3];		//	LLGT_SYS_3
  char llgt_sys_4[3];		//	LLGT_SYS_4
  char llgt_sys_5[3];		//	LLGT_SYS_5
  char llgt_sys_6[3];		//	LLGT_SYS_6
  char llgt_sys_7[3];		//	LLGT_SYS_7
  char llgt_sys_8[3];		//	LLGT_SYS_8
  char he_true_hdg[6];		//	HE_TRUE_HDG
  char le_true_hdg[6];		//	LE_TRUE_HDG
  char cld_rwy[2];		//	CLD_RWY        (closed= C)
  char heland_dis[6];		//	HELAND_DIS
  char he_takeoff[6];		//	HE_TAKEOFF
  char leland_dis[6];		//	LELAND_DIS
  char le_takeoff[6];		//	LE_TAKEOFF
  char rcycle_date[7];		//	CYCLE_DATE


#define MAX_RWY 20

 int num_rwy;  //how many at airport
 int i_rwy;

 typedef char ident_str[4];
 typedef char elev_str[8];
 typedef char lat_str[12];
 typedef char long_str[12];


 ident_str _rwy_ident[MAX_RWY];
 elev_str _rwy_elev[MAX_RWY][8];
 elev_str _rwy_width[MAX_RWY][8];
 lat_str _rwy_lat[MAX_RWY][12];
 long_str _rwy_long[MAX_RWY][12];
 long_str _rwy_t_heading[MAX_RWY][12];

//------------------------------------------------------------------- NAVAID record


  char nav_ident[6];
//  char type[2];
  char ctry[3];
  char nav_key_cd[3];
//  char state_prov[3];
//  char name[39];
//  char icao[5];
//  char wac[5];
  char freq[9];
  char usage_cd[2];
  char chan[6];
  char rcc[5];
  char freq_prot[10];
  char power[4];
  char nav_range[3];
//  char loc_hdatum[10];
//  char wgs_datum[11];
//  char wgs_lat[4];
//  char wgs_dlat[10];
//  char wgs_long[11];
//  char wgs_dlong[4];
  char slaved_var[10];
//  char mag_var[13];	
//  char elev[6];
  char dme_wgs_lat[10];
  char dme_wgs_dlat[11];
  char dme_wgs_long[10];
  char dme_wgs_dlong[11];
  char dme_elev[6];
  char arpt_icao[5];
  char os[2];
//  char cycle_date[4];

//--------------------------------------------------------------------- WAYPOINT record

  char wpt_ident[9];
//  char ctry[3];
//  char state_prov[3];
  char wpt_nav_flag[2];
  char wpt_type[4];
  char desc[26];
//  char icao[5];
//  char usage_cd[2];
  char bearing[9];
  char wpt_distance[9];
//  char wac[5];
//  char loc_hdatum[10];
//  char wgs_datum[11];
//  char wgs_lat[4];
//  char wgs_dlat[10];
//  char wgs_long[11];
//  char wgs_dlong[4];
//  char mag_var[13];
//  char nav_ident[5];
  char nav_type[3];
  char nav_ctry[3];
//  char nav_key_cd[3];
//  char cycle_date[5];

//--------------------------------------------------------------------ILS record

//arpt_ident
  char rwy_ident[4];			//  RUNWAY IDENTIFIER	3	A/N	4-2

  char rwy_ident_ref[4];
  char rwy_ident_last[4];
  bool rwy_ident_changed=false;

  bool do_vfr_glideslope=true;


  char comp_type[2];			//  COMPONENT TYPE	1	A	4-3
  char colctn[3];			//  COLLOCATION	2	A	4-4
//  char name[39];			//  NAME	38	A/N	4-5
//  char freq[9];				//  FREQUENCY	8	A/N	4-6
//  char chan[5];				//  CHANNEL	4	A/N	4-7
  char gs_angle[4];			//  GLIDE SLOPE ANGLE	3	A/N	4-8
  char lczr_gslctn[7];			//  LOCALIZER OR GLIDE SLOPE  LOCATION	6	a/n	4-9
  char loc_mkrlctn[6];			//  LOCATOR OR MARKER LOCATION	5	A/N	4-11
//  char elev[6];				//  ELEVATION	5	a/n	4-12
//  char loc_hdatum[4];			//  LOCAL HORIZONTAL DATUM	3	A	4-13
//  char wgs_datum[10];			//  LATITUDE	9	a/n	4-14
  char ils_cat[11];			//  longitude	10	a/n	4-15
//  char wgs_lat[4];			//  world geodetic datum	3	a/n	4-16
//  char wgs_dlat[2];			//  ILS/MLS CATEGORY	1	A/N	4-17
//  char wgs_long[10];			//  geodetic latitude	9	a/n	4-18
//  char wgs_dlong[11];			//  geodetic longitude	10	a/n	4-19
//  char nav_ident[6];			//  IDENTIFIER	5	a	4-20
//  char nav_type[2];			//  NAVAID TYPE	1	n	4-21
//  char nav_ctry[3];			//  NAVAID country code	2	a	4-22
//  char nav_key_cd[3];			//  NAVAID key code	2	n	4-23
//  char mag_var[13];			//  magnetic variation	12	a/n	4-24
  char slave_var[10];			//  SLAVED VARIATION	9	A/N	4-25
  char ils_brg[5];			//  ILS BEARING COURSE	4	A/N	4-26
  char loc_width[5];			//  LOCALIZER WIDTH	4	N	4-27
  char thd_crossing_hgt[3];		//  THRESHOLD CROSSING HEIGHT	2	N	4-28
  char dme_bias[3];			//  ILS DME BIAS	2	N	4-29
//  char cycle_date[5];			//  cycle date	4	n	4-30


//******************************** ils elevation capture
#define BAD_ELEVATION -99999.99
#define BAD_LAT_LONG  -99999.99
#define BAD_GS_ANGLE  -99999.99

  double ils_elev;
  double ils_gs_angle;
  double ils_inner_long;
  double ils_middle_long;
  double ils_outer_long;
  double ils_inner_lat;
  double ils_middle_lat;
  double ils_outer_lat;
  double ils_thd_crossing_hgt;
  char ils_inner_name[39];
  char ils_middle_name[39];
  char ils_outer_name[39];

//--------------------------------------------------------------------- now for the fun stuff -----------------------------

  char country[3];

//----------------------------------------------------------------------US state codes -------------------------------------
#define NSTATES 51
const string us_state[NSTATES][2] = {
		"01","AL",// 					ALABAMA
		"02","AK",//					ALASKA
		"04","AZ",//					ARIZONA
		"05","AR",//					ARKANSAS
		"06","CA",//						CALIFORNIA
		"08","CO",//						COLORADO
		"09","CT",//						CONNECTICUT
		"10","DE",//						DELAWARE
		"11","DC",//						DISTRICT OF COLUMBIA*
		"12","FL",//						FLORIDA
		"13","GA",//						GEORGIA
		"15","HA",//						HAWAII
		"16","ID",//						IDAHO
		"17","IL",//						ILLINOIS
		"18","IN",//						INDIANA
		"19","IA",//						IOWA
		"20","KA",//						KANSAS
		"21","KY",//						KENTUCKY
		"22","LA",//						LOUISIANA
		"23","ME",//						MAINE
		"24","MD",//						MARYLAND
		"25","MA",//						MASSACHUSETTS
		"26","MI",//						MICHIGAN
		"27","MN",//						MINNESOTA
		"28","MS",//						MISSISSIPPI
		"29","MO",//						MISSOURI
		"30","MT",//						MONTANA
		"31","NE",//						NEBRASKA
		"32","NV",//						NEVADA
		"33","NH",//						NEW HAMPSHIRE
		"34","NJ",//						NEW JERSEY
		"35","NM",//						NEW MEXICO
		"36","NY",//						NEW YORK
		"37","NC",//						NORTH CAROLINA
		"38","ND",//						NORTH DAKOTA
		"39","OH",//						OHIO
		"40","OK",//						OKLAHOMA
		"41","OR",//						OREGON
		"42","PA",//						PENNSYLVANIA
		"44","RI",//						RHODE ISLAND
		"45","SC",//						SOUTH CAROLINA
		"46","SD",//						SOUTH DAKOTA
		"47","TN",//						TENNESSEE
		"48","TX",//						TEXAS
		"49","UT",//						UTAH
		"50","VT",//						VERMONT
		"51","VA",//						VIRGINIA
		"53","WA",//						WASHINGTON
		"54","WV",//						WEST VIRGINIA
		"55","WI",//						WISCONSIN
		"56","WY"//						WYOMING
};
//--------------------------------------------------------------------------------------------------------------------------

char * lookup_state(char* state_prov) 
{
  int i;
  for (i=0; i < NSTATES; i++) if (strncmp(us_state[i][0].c_str(),state_prov,2)==0)  return (char *) us_state[i][1].c_str();
  return "";
}


//------------------------files---------------------

  FILE * fp_country;    //DAFIF country records
  FILE * fp_parent;     //DAFIF parent    "
  FILE * fp_segment;    //DAFIF segment   "
  FILE * fp_airport;    //DAFIF airport   "
  FILE * fp_runway;     //DAFIF runway    "
  FILE * fp_navaid;     //DAFIF navaid    "
  FILE * fp_waypoint;   //DAFIF waypoint  "
  FILE * fp_ils;        //DAFIF ils       "

//summary files for auditing
  FILE * fp_tiles;      //list of tiles processed (for audit and debugging).
//  FILE * fp_icao;       //list of icao processed for airspace boundary plus extra info
  FILE * fp_rep;   //summary reports
//-----------------------various constants------------------------------

#define MAX_CLASSES 12

#define CLASS_A 0    // Class A airspace
#define CLASS_B 1    // Class B   "
#define CLASS_C 2    // Class C   "
#define CLASS_D 3    // Class D   "
#define CLASS_E 4    // Class E   "

#define CLASS_SA 5   // Special use "S" A-Alert
#define CLASS_SD 6   // Special use "S" D-Danger
#define CLASS_SM 7   // Special use "S" M-Military Operations Area
#define CLASS_SP 8   // Special use "S" P-Prohibited
#define CLASS_SR 9   // Special use "S" R-Restricted
#define CLASS_ST 10  // Special use "S" T-Temporary Reserved Airspace
#define CLASS_SW 11  // Special use "S" W-Warning


bool generate_class[MAX_CLASSES] = {
  false, //true,
  true,
  true,
  true,
  false, //true,
  true,
  true,
  true,
  true,
  true,
  true,
  true
};

bool high_agl_flag;  

//flag lower altitude limits at or about UPPER_ALTITUDE_LIMIT and skip output for them as you need an ATC clearance in the zone anyway...

#define UPPER_ALTITUDE_LIMIT 100000.00
bool high_altitude_flag;

bool process_class(int i) 
{
  if ((i<0)||(i>=MAX_CLASSES)) return false;
  return generate_class[i];
}

double class_height[MAX_CLASSES] = {
                       0.0,
                   10000.0,
                    4000.0,
                    2500.0,
                   14500.0,
      UPPER_ALTITUDE_LIMIT,
      UPPER_ALTITUDE_LIMIT,
      UPPER_ALTITUDE_LIMIT,
      UPPER_ALTITUDE_LIMIT,
      UPPER_ALTITUDE_LIMIT,
      UPPER_ALTITUDE_LIMIT,
      UPPER_ALTITUDE_LIMIT
};

//idiot proof lower boundary so the depiction goes to the surface...  150 meters
#define CLASS_FLOOR_ADD 150.0

//********************************* color/texture parameters ***************************************
//************* Note can use either transparency color method or a transparent texturing method for airspace boundairies
bool use_texture[MAX_CLASSES] = {
  true,
  true,
  true,
  true,
  true,
  true,
  true,
  true,
  true,
  true,
  true,
  true
};

double rgb_colors[MAX_CLASSES][3]= { 
                         {1.0, 1.0, 1.0},    // Class A                                  (white)
                         {0.0, 0.0, 1.0},    // class B                                  (blue)
                         {1.0, 0.0, 1.0},    // class C                                  (magenta)
                         {0.5, 0.5, 1.0},    // class D                                  (blue...dashing used)
                         {1.0, 0.5, 1.0},    // class E                                  (magenta)
                         {0.5, 0.5, 0.0},    // class S A Alert                          (1/2 yellow)
                         {1.0, 0.0, 0.0},    // class S D Danger                         (red)
                         {1.0, 1.0, 0.0},    // class S M MOA                            (yellow)
                         {0.0, 0.0, 0.0},    // class S P Prohibited                     (black)
                         {0.25, 0.25, 0.25}, // class S R Restricted                     (3/4   black)
                         {0.0, 0.0, 0.0},    // class S T Temporary                      (black)
                         {1.0, 0.5, 0.0}     // class S W Warning                        (red/yellow)
                         };

double rgb_transparency[MAX_CLASSES]= {  // transparency used as a que as to whether you should enter lower number is f16 bait...
    0.8,
    0.8,  //B
    0.8,  //C
    0.8,  //D
    0.8,  //E
    0.8,  //S A
    0.8,  //S D
    0.8,  //S M
    0.8,  //S P
    0.8,  //S R
    0.8,  //S T
    0.8,  //S W
}; 
/**   Note: rgb's are generated for each individual airspace boundary to sign the wall
const string airspace_texture[MAX_CLASSES] = {
             "class_a.rgb",                       
             "class_b.rgb",                       
             "class_c.rgb",
             "class_d.rgb",
             "class_e.rgb",
             "class_s_alert.rgb",
             "class_s_danger.rgb",
             "class_s_moa.rgb",
             "class_s_prohibited.rgb",
             "class_s_restricted.rgb",
             "class_s_temporary.rgb",
             "class_s_warning.rgb"
             };
**/
const string airspace_enabled_flag[MAX_CLASSES] = {
             "airspace_a",                       
             "airspace_b",                       
             "airspace_c",                       
             "airspace_d",                       
             "airspace_e",                       
             "airspace_alert",
             "airspace_danger",
             "airspace_moa",
             "airspace_prohibited",
             "airspace_restricted",
             "airspace_temporary",
             "airspace_warning"
             };


//-----------------statistics....although iclass has heavy use otherwise ----------------------

  int cum_class[MAX_CLASSES] = {0,0,0,0,0,0,0,0,0,0,0,0};
  int cum_class_surface[MAX_CLASSES] = {0,0,0,0,0,0,0,0,0,0,0,0};
  int iclass;           // iclass=0..4 Class A-E airspace.... 5-11 are special use 
  int npoints=0;
  int ncountry=0;
  int nairports=0;

  double r_diff         =0.0;  // used for picking arc radui
  double max_r_diff     =-1.0;
  double ave_r		=0.0;


  double sum_long=0.0;      // compute an average to place sign over special use airspace
  double sum_lat=0.0;
  int num_pts=0;


void init_boundary_ave()
{
   sum_long=0.0;
   sum_lat=0.0;
   num_pts=0;
}

void sum_boundary_pt(double xlat, double xlong)
{

   sum_long = sum_long + xlong;
   sum_lat = sum_lat + xlat;
   num_pts++;
 //  printf("sum_boundary pts: added [%f] [%f] sums are [%f] [%f] n is [%d] ave is [%f] [%f] [%f] \n",xlat, xlong, sum_lat, sum_long, num_pts,sum_lat/(double)num_pts, sum_long/(double)num_pts);

}
double ave_boundary_long()
{
  if (num_pts > 0 ) return sum_long/((double)(num_pts)); else {
    fprintf(fp_rep,"WARNING: bad average calculations (ave_boundary_long)\n");
    return -1.0;
  }
}
double ave_boundary_lat()
{
  if (num_pts > 0 ) return sum_lat/((double)(num_pts)); else {
    fprintf(fp_rep,"WARNING: bad average calculations (ave_boundary_lat)\n");
    return -1.0;
  }
}

//------------------- DAFIF conversions i.e. string latitude to the actual number...---------------------------

  double r;                      // my wonderful collection of kludges...
  double rr;                     // first you make em work then you make em elegant...
  int ir;//
  int irr;//
  int ilatxx;//
  int ilongxx;//
  int ilatxxx;//
  int ilongxxx;//

  double dlat;			//  longitude
  double dlong;			//  latitude
  double dlat2;			//  longitude 2nd pt 
  double dlong2;		//  latitude  2nd pt
  double dlatc;			//  longitude center
  double dlongc;		//  latitude center
  double dlatp1;		//  longitude p1
  double dlongp1;		//  latitude p1
  double dlatp2;		//  longitude p2
  double dlongp2;		//  latitude p2

  double fremlatxxx;//
  double fremlongxxx;//
  char ns;			// n or s
  char ew;			// e or w

  double floor_alt,ceiling_alt;

  double altitude_high;
  double altitude_low;
  char alt_low_type[10];
  char alt_high_type[10];


// --------------------------------------misc--------------------------------------------


  double altitude;
  char alt_digits[12];
  char type_digits[11];
  char tile_filename[1000];
  char tile_filename_bak[1000];
  char tile_addline[1000];

  char airspace_filename[1000];
  char airspace_filename_xml[1000];
  char airspace_filename_xml_bak[1000];
  char airspace_texture_filename[1000];
  char airspace_filename_bak[1000];

  int counter=0;

  double az1		=0.0;
  double az2		=0.0;
  double distance	=0.0;
  double start_az	=0.0;
  double end_az		=0.0;
  double start_r	=0.0;
  double end_r		=0.0;
  double arc_r		=0.0;                    

  int tilenum;
  long int ltilenum;

  char subpath[1000]; 		// i.e. w080n40/w073n43 


//----------------------------------------- icao list for setting terminal waypoint heights...

struct icao_list {
  struct icao_list * next;
  struct icao_list * last;
  char _icao[9];
  double elevation;
};

  struct icao_list * head_icao    = NULL;
  struct icao_list * current_icao = NULL;
  struct icao_list * tail_icao    = NULL;
  struct icao_list * t_icao       = NULL; 
  

void erase_icao_list() {
  current_icao=head_icao;
  while (current_icao!=NULL) {
    current_icao = current_icao->next;
    delete head_icao;
    head_icao = current_icao;
  }
}  

void init_icao_list()
{
  if (head_icao!=NULL) erase_icao_list();
}

struct icao_list * get_icao(char * icao) 
{
  current_icao = head_icao;
  bool done = false;
  if (current_icao==NULL) return NULL;
  while (!done) {
  //  printf("get_icao(%s) is ? [%s] strncmp is %d\n",icao,current_icao->_icao,strncmp(current_icao->_icao,icao,9));
    if (strncmp(current_icao->_icao,icao,9) == 0 ) return current_icao;
    if (strncmp(current_icao->_icao,icao,9) < 0 ) return NULL;
    current_icao = current_icao->next;
    if (current_icao==NULL) {
      return NULL;
    }
  }
}

void add_icao(char * icao, double elev )
{
  bool done=false;

  struct icao_list * new_icao = new struct icao_list;

  new_icao->last=NULL;
  new_icao->next=NULL;
  strncpy((char *) new_icao->_icao, icao,5);
  new_icao->elevation=elev;

  if (head_icao!=NULL) {
    current_icao = head_icao;
    while (!done) {
      if (strncmp(current_icao->_icao,icao,9)>0) {
        if (current_icao->next!=NULL) {
          current_icao = current_icao->next;
        }
        else {//new tail
          current_icao->next=new_icao;
          new_icao->last=current_icao;
          done=true;
        }       
      }
      else {  // < assuming no double insert attempts...i.e. call lookup first...
        if (current_icao->last!=NULL) {
          new_icao->last = current_icao->last;
          current_icao->last->next=new_icao;
          new_icao->next = current_icao;
          current_icao->last = new_icao;
          done = true;
        }
        else {//new head
          current_icao->last=new_icao;
          new_icao->next=current_icao;
          head_icao=new_icao;
          done=true;
        }      
      } 
    } // !done
  }
  else head_icao=new_icao;
}


//----------------------------------------- procedures for paths etc
void set_subpath_for(double dlat, double dlong)
{
  if (dlong>=0.0) {
    ew='e';
    ilongxxx    = (int) dlong;  //truncate    
    fremlongxxx = dlong - trunc(dlong); //(double) ilongxxx;         
    ilongxxx    = (int) (dlong+0.999999);  
    ilongxx     = (int) (ilongxxx+9.999999)/10;
  }
  else {
    ew ='w';
    ilongxxx = (int) (0.9999999-dlong); 
    fremlongxxx = fabs((double) dlong) - trunc(fabs(dlong));         
    ilongxx = (int)(ilongxxx+9.999999)/10;
    ilongxxx = -ilongxxx;
    ilongxx  = -ilongxx;
  }
  if (dlat>=0) {
    ns='n';
    ilatxxx = (int)dlat;  //truncate    
    fremlatxxx = dlat - trunc(dlat);         
    ilatxxx = (int)dlat;  //round down
    ilatxx = (ilatxxx)/10;
  }
  else {
    ns='s';
    ilatxxx = (int)(-dlat);  //truncate    
    fremlatxxx = fabs(dlat) - trunc(fabs(dlat));// - (double) ilatxxx;         
    ilatxxx = (int)(-dlat);  //round down
    ilatxx = (ilatxxx)/10;
    ilatxxx = -ilatxxx;
    ilatxx  = -ilatxx;
  }
  fremlatxxx = fremlatxxx*8.0;
  fremlongxxx = fremlongxxx*8.0;
  x = (int)fremlongxxx; //+0.999999;
  y = (int)fremlatxxx; //+0.999999;
  sprintf(subpath,"%c",ew);      // [e]
  if (abs(ilongxx)<10) sprintf(subpath,"%s0%d",subpath,abs(ilongxx));  // [e0x]
  else sprintf(subpath,"%s%d",subpath,abs(ilongxx));      // [exx]
  sprintf(subpath,"%s0%c",subpath,ns);  //  [e0x0n] or [exx0xn]
  sprintf(subpath,"%s%d",subpath,abs(ilatxx));     // [e0xxnxx]
  sprintf(subpath,"%s0/%c",subpath,ew);        // [e0xxnxx0
  if (abs(ilongxxx)<100) sprintf(subpath,"%s0%d",subpath,abs(ilongxxx));
  else sprintf(subpath,"%s%d",subpath,abs(ilongxxx));
  sprintf(subpath,"%s%c",subpath,ns);
  if (abs(ilatxxx)<10) sprintf(subpath,"%s0%d/",subpath,abs(ilatxxx));
  else sprintf(subpath,"%s%d/",subpath,abs(ilatxxx));
}

//----------------------------------------- line management and generation to keep lines within tile boundaries--------


//**************Line division work...
//NOTE! bucket span changes as a function of latitude.
//
double divisor;

struct pt {
  double xp;
  double yp;
  char seg_nbr[6];
  struct pt * next;
  struct pt * last;
};

  struct pt * head_pt=NULL;
  struct pt * current_pt=NULL;
  struct pt * end_pt=NULL;
  struct pt * t_pt=NULL;

  struct pt * head_pt_m=NULL;
  struct pt * current_pt_m=NULL;
  struct pt * tail_pt_m=NULL;
  struct pt * t_pt_m=NULL; 


 int good_lists=0;
 int bad_lists=0;


// tile node for processing line lists for an airspace identifier....

struct tile_list {
  struct tile_list * next;
  struct tile_list * last;
  struct pt * start;
  struct pt * end;
  long int tile_nbr;
};

  struct tile_list * head_tile    = NULL;
  struct tile_list * current_tile = NULL;
  struct tile_list * tail_tile    = NULL;
  struct tile_list * t_tile       = NULL; 
  

void erase_tile_list() {
  current_tile=head_tile;
  while (current_tile!=NULL) {
    current_tile = current_tile->next;
    delete head_tile;
    head_tile = current_tile;
  }
}  

void init_tile_list()
{
  if (head_tile!=NULL) erase_tile_list();
}

void add_tile(long int tn, pt * sp, pt * ep)
{
  bool done=false;

  struct tile_list * new_tile = new struct tile_list;

  new_tile->last=NULL;
  new_tile->next=NULL;
  new_tile->start=sp;
  new_tile->end=ep;
  new_tile->tile_nbr = tn;
 
  if (head_tile!=NULL) {
    current_tile = head_tile;
    while (!done) {
      if (tn <= current_tile->tile_nbr) {  //insert before
        done=true;
        if (current_tile->last!=NULL) { // in list insert
          new_tile->last = current_tile->last;  
          new_tile->next = current_tile;
          current_tile->last=new_tile;
          new_tile->last->next=new_tile;
        }
        else { //new head
          current_tile->last=new_tile;
          new_tile->next=current_tile;
          head_tile = new_tile;
        }
      }
      else { // insert after ?
        if (current_tile->next != NULL) current_tile = current_tile->next;
        else {
          done=true;
          current_tile->next = new_tile;
          new_tile->last = current_tile;
        }
      }
    } // !done
  }
  else head_tile=new_tile;
}


  FILE * ac_fp=NULL;


  int nkids;
  int ikid;


void write_xml_file()
{
// airspace_filename_xml
  FILE * x_fp;
  int ik;
  string command = "mkdir -p "; command += SGPath(airspace_filename_xml).dir();
   // cout << command << endl;
  system(command.c_str());
  x_fp = fopen(airspace_filename_xml,"w+");
  if (x_fp!=NULL) {
    fprintf(x_fp,"<?xml version=\"1.0\"?>\n<PropertyList>\n");
    fprintf(x_fp,"  <path>%s-%d.ac</path>\n",bdry_ident_safe,current_tile->tile_nbr);
    fprintf(x_fp,"  <animation>\n    <type>select</type>\n");
    for (ik=0; ik<nkids; ik++) {
      fprintf(x_fp,"    <object-name>airspace-%d</object-name>\n",ik+1);
    }
    fprintf(x_fp,"    <condition>\n      <property>/sim/airspace/enabled</property>\n");
    fprintf(x_fp,"      <property>/sim/%s/enabled</property>\n    </condition>\n  </animation>\n</PropertyList>\n",airspace_enabled_flag[iclass].c_str());
    fclose(x_fp);
  } 
  else { 
    fprintf(fp_rep,"WARNING! COULD WRITE TO FILE [%s]\n",airspace_filename_xml);
  }
}
// who's the leader of the pack that's ........  M  I  C  K  E  Y      M  O  U  S  E....but I'll clean them up later, but individual texture generators allow
// for tweaking.  For example, Restricted airpsace is usually a tower as to an MOA's more fence like configuration, so the texturing can be tuned....
void write_airspace_texture()
{
  char command_str[1000];

  string command = "";
  sprintf(command_str,"");
  switch (iclass) {
    case CLASS_A : {
      command = "convert -size 256x128 xc:lightblue  -encoding None -font Helvetica-Bold -fill white \\\n"; 
      sprintf(command_str,"-pointsize 15 -gravity Center -draw \"text  33,-33 'Class A\\n%s\\n%s %s'\" \\\n",picao,pcomm_freq1,pcomm_freq2);  
      command += command_str;
      sprintf(command_str,"-pointsize 15 -gravity Center -draw \"text  -99,-33 'Class A\\n%s\\n%s %s'\" \\\n",picao,pcomm_freq1,pcomm_freq2);  
      command += command_str;
      command += "-rotate 90 \\\n";
      sprintf(command_str,"-compress RLE SGI:%s\n",airspace_texture_filename);
      command += command_str;
    break;
    } 
    case CLASS_B : {
      command = "convert -size 256x128 xc:blue  -encoding None -font Helvetica-Bold -fill white \\\n"; 
      sprintf(command_str,"-pointsize 15 -gravity Center -draw \"text  33,-33 'Class B\\n%s\\n%s %s'\" \\\n",picao,pcomm_freq1,pcomm_freq2);  
      command += command_str;
      sprintf(command_str,"-pointsize 15 -gravity Center -draw \"text  -99,-33 'Class B\\n%s\\n%s %s'\" \\\n",picao,pcomm_freq1,pcomm_freq2);  
      command += command_str;
      command += "-rotate 90 \\\n";
      sprintf(command_str,"-compress RLE SGI:%s\n",airspace_texture_filename);
      command += command_str;
      break;
    } 
    case CLASS_C : {
      command = "convert -size 256x128 xc:magenta  -encoding None -font Helvetica-Bold -fill white \\\n"; 
      sprintf(command_str,"-pointsize 15 -gravity Center -draw \"text  33,-33 'Class C\\n%s\\n%s %s'\" \\\n",picao,pcomm_freq1,pcomm_freq2);  
      command += command_str;
      sprintf(command_str,"-pointsize 15 -gravity Center -draw \"text  -99,-33 'Class C\\n%s\\n%s %s'\" \\\n",picao,pcomm_freq1,pcomm_freq2);  
      command += command_str;
      command += "-rotate 90 \\\n";
      sprintf(command_str,"-compress RLE SGI:%s\n",airspace_texture_filename);
      command += command_str;
      break;
    } 
    case CLASS_D : {
      command = "convert -size 256x128 xc:blue  -encoding None -font Helvetica-Bold -fill white \\\n"; 

      command+="-draw \"rectangle 0,0,64,128\" \\\n";
      command+="-draw \"rectangle 128,0,192,128\" \\\n";
      command+="xc:white -fill black \\\n";

      sprintf(command_str,"-pointsize 15 -gravity Center -draw \"text  33,-33 'Class D\\n%s\\n%s %s'\" \\\n",picao,pcomm_freq1,pcomm_freq2);  
      command += command_str;
      sprintf(command_str,"-pointsize 15 -gravity Center -draw \"text  -99,-33 'Class D\\n%s\\n%s %s'\" \\\n",picao,pcomm_freq1,pcomm_freq2);  
      command += command_str;
      command += "-rotate 90 \\\n";
      sprintf(command_str,"-compress RLE SGI:%s\n",airspace_texture_filename);
      command += command_str;
      break;
    } 
    case CLASS_E : {
      command = "convert -size 256x128 xc:magenta  -encoding None -font Helvetica-Bold -fill white \\\n"; 

      command+="-draw \"rectangle 0,0,64,128\" \\\n";
      command+="-draw \"rectangle 128,0,192,128\" \\\n";
      command+="xc:white -fill black \\\n";

      sprintf(command_str,"-pointsize 15 -gravity Center -draw \"text  33,-33 'Class E\\n%s\\n%s %s'\" \\\n",picao,pcomm_freq1,pcomm_freq2);  
      command += command_str;
      sprintf(command_str,"-pointsize 15 -gravity Center -draw \"text  -99,-33 'Class E\\n%s\\n%s %s'\" \\\n",picao,pcomm_freq1,pcomm_freq2);  
      command += command_str;
      command += "-rotate 90 \\\n";
      sprintf(command_str,"-compress RLE SGI:%s\n",airspace_texture_filename);
      command += command_str;
      break;
    } 

//************************************** special use airspace ***********************************************8

    case CLASS_SA : {
      command ="convert -size 256x128 xc:orange  -encoding None -font Helvetica-Bold -fill black \\\n";
      command+="-gravity west \\\n";//                                      'Class B\\n%s\               v                       \n%s %s'\" \\\n"
      sprintf(command_str,"-pointsize 12 -gravity Center -draw \"text -45,-7 '%s\\n \\\n%s\\n \\\n%s %s\\n%s\\n \\\n%s\\n \\\n%s %s'\" \\\n",
      pname,pcomm_name,pcomm_freq1,pcomm_freq2,
      pname,pcomm_name,pcomm_freq1,pcomm_freq2);
      command += command_str;
      command+="-gravity west \\\n";
      sprintf(command_str,"-pointsize 12 -gravity Center -draw \"text  65,-7 '%s\\n \\\n%s\\n \\\n%s %s\\n%s\\n \\\n%s\\n \\\n%s %s'\" \\\n",
      pname,pcomm_name,pcomm_freq1,pcomm_freq2,
      pname,pcomm_name,pcomm_freq1,pcomm_freq2);
      command += command_str;
      command += "-rotate 90 \\\n";
      sprintf(command_str,"-compress RLE SGI:%s\n",airspace_texture_filename);
      command += command_str;
      break;
    } 
    case CLASS_SD : {
      command ="convert -size 256x128 xc:red  -encoding None -font Helvetica-Bold -fill black \\\n";
      command+="-gravity west \\\n";
      sprintf(command_str,"-pointsize 12 -gravity Center -draw \"text -65,-7 '%s\\n \\\n%s\\n \\\n%s %s\\n%s\\n \\\n%s\\n \\\n%s %s'\" \\\n",
      pname,pcomm_name,pcomm_freq1,pcomm_freq2,
      pname,pcomm_name,pcomm_freq1,pcomm_freq2);
      command += command_str;
      command+="-gravity west \\\n";
      sprintf(command_str,"-pointsize 12 -gravity Center -draw \"text  65,-7 '%s\\n \\\n%s\\n \\\n%s %s\\n%s\\n \\\n%s\\n \\\n%s %s'\" \\\n",
      pname,pcomm_name,pcomm_freq1,pcomm_freq2,
      pname,pcomm_name,pcomm_freq1,pcomm_freq2);
      command += command_str;
      command += "-rotate 90 \\\n";
      sprintf(command_str,"-compress RLE SGI:%s\n",airspace_texture_filename);
      command += command_str;
      break;
    } 
    case CLASS_SM : {
      command ="convert -size 256x128 xc:yellow  -encoding None -font Helvetica-Bold -fill black \\\n";
      command+="-gravity west \\\n";
      sprintf(command_str,"-pointsize 12 -gravity Center -draw \"text -65,-7 '%s\\n \\\n%s\\n \\\n%s %s\\n%s\\n \\\n%s\\n \\\n%s %s'\" \\\n",
      pname,pcomm_name,pcomm_freq1,pcomm_freq2,
      pname,pcomm_name,pcomm_freq1,pcomm_freq2);
      command += command_str;
      command+="-gravity west \\\n";
      sprintf(command_str,"-pointsize 12 -gravity Center -draw \"text  65,-7 '%s\\n \\\n%s\\n \\\n%s %s\\n%s\\n \\\n%s\\n \\\n%s %s'\" \\\n",
      pname,pcomm_name,pcomm_freq1,pcomm_freq2,
      pname,pcomm_name,pcomm_freq1,pcomm_freq2);
      command += command_str;
      command += "-rotate 90 \\\n";
      sprintf(command_str,"-compress RLE SGI:%s\n",airspace_texture_filename);
      command += command_str;
/*

convert -size 256x128 xc:yellow  -encoding None -font Helvetica-Bold -fill black \
-gravity west \
-pointsize 12 -gravity Center -draw "text -65,8 'YANKEE 1 MOA, NH\n \
BOSTON CENTER\n \
135.7   M 282.2   M\n \
YANKEE 1 MOA, NH\n \
BOSTON CENTER\n \
135.7   M 282.2   M\n" \
-gravity west \
-pointsize 12 -gravity Center -draw "text  65,8 'YANKEE 1 MOA, NH\n \
BOSTON CENTER\n \
135.7   M 282.2   M\n \
YANKEE 1 MOA, NH\n \
BOSTON CENTER\n \
135.7   M 282.2   M\n" \
-rotate 90  \
-compress RLE SGI:/usr/local/share/FlightGear/data/Scenery-Airspace/Objects/w080n40/w073n43/MOA-US-02350-1761659.rgb
*/


      break;
    } 
    case CLASS_SP : {
      command ="convert -size 256x128 xc:grey  -encoding None -font Helvetica-Bold -fill black \\\n";
      command+="-gravity west \\\n";
      sprintf(command_str,"-pointsize 12 -gravity Center -draw \"text -65,-7 '%s\\n \\\n%s\\n \\\n%s %s\\n%s\\n \\\n%s\\n \\\n%s %s'\" \\\n",
      pname,pcomm_name,pcomm_freq1,pcomm_freq2,
      pname,pcomm_name,pcomm_freq1,pcomm_freq2);
      command += command_str;
      command+="-gravity west \\\n";
      sprintf(command_str,"-pointsize 12 -gravity Center -draw \"text  65,-7 '%s\\n \\\n%s\\n \\\n%s %s\\n%s\\n \\\n%s\\n \\\n%s %s'\" \\\n",
      pname,pcomm_name,pcomm_freq1,pcomm_freq2,
      pname,pcomm_name,pcomm_freq1,pcomm_freq2);
      command += command_str;
      command += "-rotate 90 \\\n";
      sprintf(command_str,"-compress RLE SGI:%s\n",airspace_texture_filename);
      command += command_str;
      break;
    } 
    case CLASS_SR : {
      command ="convert -size 256x128 xc:grey  -encoding None -font Helvetica-Bold -fill black \\\n";
      command+="-gravity west \\\n";
      sprintf(command_str,"-pointsize 12 -gravity Center -draw \"text -65,-7 '%s\\n \\\n%s\\n \\\n%s %s\\n%s\\n \\\n%s\\n \\\n%s %s'\" \\\n",
      pname,pcomm_name,pcomm_freq1,pcomm_freq2,
      pname,pcomm_name,pcomm_freq1,pcomm_freq2);
      command += command_str;
      command+="-gravity west \\\n";
      sprintf(command_str,"-pointsize 12 -gravity Center -draw \"text  65,-7 '%s\\n \\\n%s\\n \\\n%s %s\\n%s\\n \\\n%s\\n \\\n%s %s'\" \\\n",
      pname,pcomm_name,pcomm_freq1,pcomm_freq2,
      pname,pcomm_name,pcomm_freq1,pcomm_freq2);
      command += command_str;
      command += "-rotate 90 \\\n";
      sprintf(command_str,"-compress RLE SGI:%s\n",airspace_texture_filename);
      command += command_str;
      break;
    } 
    case CLASS_ST : {
      command ="convert -size 256x128 xc:grey  -encoding None -font Helvetica-Bold -fill black \\\n";
      command+="-gravity west \\\n";
      sprintf(command_str,"-pointsize 12 -gravity Center -draw \"text -65,-7 '%s\\n \\\n%s\\n \\\n%s %s\\n%s\\n \\\n%s\\n \\\n%s %s'\" \\\n",
      pname,pcomm_name,pcomm_freq1,pcomm_freq2,
      pname,pcomm_name,pcomm_freq1,pcomm_freq2);
      command += command_str;
      command+="-gravity west \\\n";
      sprintf(command_str,"-pointsize 12 -gravity Center -draw \"text  65,-7 '%s\\n \\\n%s\\n \\\n%s %s\\n%s\\n \\\n%s\\n \\\n%s %s'\" \\\n",
      pname,pcomm_name,pcomm_freq1,pcomm_freq2,
      pname,pcomm_name,pcomm_freq1,pcomm_freq2);
      command += command_str;
      command += "-rotate 90 \\\n";
      sprintf(command_str,"-compress RLE SGI:%s\n",airspace_texture_filename);
      command += command_str;
      break;
    } 
    case CLASS_SW : {
      command ="convert -size 256x128 xc:red  -encoding None -font Helvetica-Bold -fill black \\\n";
      command+="-gravity west \\\n";
      sprintf(command_str,"-pointsize 12 -gravity Center -draw \"text  65,-7 '%s\\n \\\n%s\\n \\\n%s %s\\nWarning:%s\\n \\\n%s\\n \\\n%s %s'\" \\\n",
      pname,pcomm_name,pcomm_freq1,pcomm_freq2,
      pname,pcomm_name,pcomm_freq1,pcomm_freq2);
      command += command_str;
      command+="-gravity west \\\n";
      sprintf(command_str,"-pointsize 12 -gravity Center -draw \"text -65,-7 'Warning:%s\\n \\\n%s\\n \\\n%s %s\\nWarning:%s\\n \\\n%s\\n \\\n%s %s'\" \\\n",
      pname,pcomm_name,pcomm_freq1,pcomm_freq2,
      pname,pcomm_name,pcomm_freq1,pcomm_freq2);
      command += command_str;
      command += "-rotate 90 \\\n";
      sprintf(command_str,"-compress RLE SGI:%s\n",airspace_texture_filename);
      command += command_str;
      break;
    }   
  }
  //string 
 // command = command_str; 
// I put the string in the report file for debugging and tweaking experiments...
  fprintf(fp_rep,"Create rgb file with command: [%s]\n",command.c_str());
//  printf("Create rgb file with command: [%s]\n",command.c_str());
  system(command.c_str());
}


void write_ac_header()
{
  nkids=0;
  tile_list * t_tile=current_tile;
  long int ttn = current_tile->tile_nbr;
  bool done=false;
  while (!done) {
    if (t_tile->tile_nbr==ttn) {  
      nkids++;
      t_tile=t_tile->next;
    }
    else done=true;
    if (t_tile==NULL) done=true;
  }
  if (use_texture[iclass]) {
    fprintf(ac_fp,"AC3Db\nMATERIAL \"Material.001\" rgb 1 1 1 amb 0.5 0.5 0.5 emis 0 0 0 spec 1 1 1 shi  80 trans 0.7\nOBJECT world\nkids %d\n",nkids);
    write_airspace_texture();
  }
  else {
    fprintf(ac_fp,"AC3Db\nMATERIAL \"Material.001\" rgb %f %f %f  amb 0.5 0.5 0.5 emis 0 0 0 spec 1 1 1 shi  80 trans %f\nOBJECT world\nkids %d\n",
    rgb_colors[iclass][0], rgb_colors[iclass][1], rgb_colors[iclass][2],rgb_transparency[iclass], nkids);
  }
  if (nkids==0) printf(" WARNING, no kids for ac file! ");
  ikid=1;
}
void ac_from_wgs_84(double dlatc, double dlongc, double dlatp, double dlongp,double *ac_x, double *ac_y)
{
  double start_az; 
  double az2;
  double r;

  geo_inverse_wgs_84(0.0, dlatc, dlongc, dlatp,dlongp, &start_az, &az2,&r);

  start_az+=180.0; //get ac to flightgear bearings aligned

  *ac_x = r * cos(start_az*SGD_DEGREES_TO_RADIANS);
  
  *ac_y = r * sin(start_az*SGD_DEGREES_TO_RADIANS);

}


void write_kid(double dlatc, double dlongc, double zlow, double zhigh)
{
  double ac_x;
  double ac_y;
  struct pt * sp;
  struct pt * cp;
  struct pt * ep;

  int npts=0;
  int numvert=0;
  int numsurf=0;
  int isurf;
  int ix;
  bool done;

  fprintf(ac_fp,"OBJECT poly\nname\"airspace-%d\"\n",ikid);


  if (use_texture[iclass]) {
//    sprintf(airspace_texture_filename,"%s/%s",airspace_texture_path.c_str(),airspace_texture[iclass].c_str());
    fprintf(ac_fp,"texture \"%s\"\n",airspace_texture_filename);  
/*  symbolic link design abandoned in favor of signed textures 
    string command = "ln -s  "; 
    command += airspace_texture_filename; // SGPath(airspace_texture_filename).dir();  
    command += " ";
    command += SGPath(airspace_filename).dir();
    command += "/";
    command += airspace_texture[iclass].c_str();
//  cout << command << endl;
    system(command.c_str());
*/
  }
//**/
  ikid++;

// numvert matrix
  sp = current_tile->start;
  ep = current_tile->end;
  cp = sp;
  npts=1;
  while (cp!=ep) {
    npts++;
    cp = cp->next;
  }
  if (npts<2) {
    printf("Warning npts is only %d\n",npts);
    return;
  }
  numvert = 2*npts;  // this design has redundant pts...used now for gator management while draining and cleaning the swimming pool...
  numsurf = npts-1;
  fprintf(ac_fp,"numvert %d    #origin at [%11.6f] - [%11.6f]  npts %d low alt [%f] high alt [%f]\n",numvert,dlatc, dlongc,npts,zlow,zhigh);
  if (numvert!=0) {
  cp = sp;    

// need to translate the x, y pts into their metric units with the origin at the center of the tile...
  ix=0;
  done=false;
  while (!done) {
    ac_from_wgs_84(dlatc, dlongc, cp->yp, cp->xp,&ac_x, &ac_y);
    fprintf(ac_fp,"  %12.2f  %12.2f  %12.2f   # pt(%4d) %11.6f  %11.6f  %11.6f\n", ac_x, zlow, ac_y, ix, cp->xp, zlow, cp->yp);
    ix++;
    if (cp!=ep) cp = cp->next;
    else done=true;
  }
  cp = sp; 
  done=false;    
  while (!done) {
    ac_from_wgs_84(dlatc, dlongc, cp->yp, cp->xp,&ac_x, &ac_y);
    fprintf(ac_fp,"  %12.2f  %12.2f  %12.2f   # pt(%4d) %11.6f  %11.6f  %11.6f\n", ac_x, zhigh, ac_y, ix, cp->xp, zhigh, cp->yp);
    ix++;
    if (cp!=ep) cp = cp->next;
    else done=true;
  }
  fprintf(ac_fp,"numsurf %d\n",numsurf);
  for (isurf=0; isurf<numsurf; isurf++) {
     fprintf(ac_fp,"SURF 0x20\nmat 0\nrefs 4\n");
     fprintf(ac_fp,"%5d 0.003906 0.640625\n",isurf);
     fprintf(ac_fp,"%5d 0.972656 0.644531\n",numvert/2+isurf);
     fprintf(ac_fp,"%5d 0.972656 0.992188\n",numvert/2+isurf+1);
     fprintf(ac_fp,"%5d 0.003906 0.988281\n",isurf+1);
  }
  fprintf(ac_fp,"kids 0\n");
  }
  else printf("Numvert is 0 in write_kid!\n");
}

bool open_airspace_file()
{
  string command = "mkdir -p "; command += SGPath(airspace_filename).dir();
 // cout << command << endl;
  system(command.c_str());
  ac_fp = fopen(airspace_filename,"w+");
  if (ac_fp!=NULL) { 
     write_ac_header();
     write_xml_file(); //now that we know nkids...
     return true;
  }
  else { 
    fprintf(fp_rep,"WARNING! COULD NOT OPEN FILE [%s]\n",airspace_filename);
    return false;
  }
}

void process_tile_file()
{
  FILE * t_fp;
  fprintf(fp_tiles,"(%4s) %85s <-> %s",picao,tile_filename, tile_addline);
  string command = "mkdir -p "; command += SGPath(tile_filename).dir();
 // cout << command << endl;
  system(command.c_str());
  t_fp = fopen(tile_filename,"a");
  if (t_fp!=NULL) {
    fprintf(t_fp,"%s\n",tile_addline);
    fclose(t_fp);
  } 
  else { 
    fprintf(fp_rep,"WARNING! COULD NOT APPEND TO FILE [%s]\n",tile_filename);
  }
}

bool close_airspace_file()
{
  if (ac_fp!=NULL) {
    if (fclose(ac_fp) != EOF) return true; else return false;
  }
}

void set_tile_info(double * dlatc, double * dlongc)
{
  double x, y, xn, yn;
  double ax, ay;
  double divisor;
  divisor = 1.0/sg_bucket_span(current_tile->start->next->yp);
  x = (current_tile->start->xp + 1000.0)*divisor;  // arbitrary transform to keep math positive and "indexed" 
  y = (current_tile->start->yp  + 1000.0)*8.0;  
  xn= (current_tile->start->next->xp + 1000.0)*divisor;  // arbitrary transform to keep math positive and "indexed" 
  yn= (current_tile->start->next->yp  + 1000.0)*8.0;  

  ax= (x+xn)/2.0;
  ay= (y+yn)/2.0;
  ax =  (trunc(ax)/divisor-1000.0+trunc(ax+1.0)/divisor-1000.0)/2.0;
  ay =  (trunc(ay)/8.0-1000.0+trunc(ay+1.0)/8.0-1000.0)/2.0;

  *dlatc  =ay;
  *dlongc =ax;

  set_subpath_for(ay,ax); //current_tile->start->next->yp, current_tile->start->next->xp);   
  sprintf(airspace_filename, "%s/Objects/%s%s-%d.ac", output_base.c_str(), subpath, bdry_ident_safe,current_tile->tile_nbr);
  if (use_texture[iclass]) sprintf(airspace_texture_filename, "%s/Objects/%s%s-%d.rgb", output_base.c_str(), subpath, bdry_ident_safe,current_tile->tile_nbr);
  sprintf(airspace_filename_xml, "%s/Objects/%s%s-%d.xml", output_base.c_str(), subpath, bdry_ident_safe,current_tile->tile_nbr);
  sprintf(tile_addline,"OBJECT_STATIC %s-%d.xml %f %f %f 0.00\n",bdry_ident_safe,current_tile->tile_nbr, ax, ay, altitude_high*SG_FEET_TO_METER);
  sprintf( tile_filename, "%s/Objects/%s%d.stg", output_base.c_str(), subpath, current_tile->tile_nbr);
  process_tile_file();
  open_airspace_file();
} 

void process_tile_list()
{
  double dlongc;
  double dlatc;
  struct pt * sp;
  struct pt * cp;
  struct pt * ep;

  int ix;

  long int ctn; 
 
  int nk;

  current_tile = head_tile;
  ctn = current_tile->tile_nbr;
  set_tile_info(&dlatc, &dlongc);
  ctn = current_tile->tile_nbr;
  while (current_tile !=NULL) {
    if (current_tile->tile_nbr !=ctn) {
       close_airspace_file();
       set_tile_info(&dlatc, &dlongc);
       ctn = current_tile->tile_nbr;
    }
    write_kid(dlatc, dlongc, floor_alt, 0.0); //altitude_low,altitude_high);
    sp = current_tile->start;
    ep = current_tile->end;
    cp = sp;
    ix=1;
    while (cp!=ep) {
      ix++;
      cp = cp->next;
    }
    if (cp->next== NULL) {
      cp = cp->last;  //cross over line between tile boundaries for lines  1 - (n-1) with the last tile ending with ep->next being NULL
    }
    current_tile=current_tile->next;
  }
  close_airspace_file();
}

void create_tile_list()
{ 
  int ix;
  struct pt * hp = head_pt_m;
  struct pt * cp = hp;
  struct pt * sp = hp;
  ix=1;
  double txp;
  double typ;
  double txp2;
  double typ2;

  init_tile_list();
  cp=hp;
  ix=1;

  long int last_tile;
  long int tile_n;
  int tile_changes=0;

  struct pt * startpt=head_pt_m;
  struct pt * endpt;
  struct pt * lastpt = NULL;
  if (hp==NULL) {
    printf("create_tile_list call with NULL ptr!\n");
    return;
  }
  txp  =  cp->xp;
  typ  =  cp->yp;
  txp2 =  cp->next->xp;
  typ2 =  cp->next->yp;

  last_tile = gen_tilenum( (txp+txp2)/2.0, (typ+typ2)/2.0);

  while ( cp->next!=NULL) {

    txp  =  cp->xp;
    typ  =  cp->yp;
    txp2 =  cp->next->xp;
    typ2 =  cp->next->yp;

    tile_n = gen_tilenum( (txp+txp2)/2.0, (typ+typ2)/2.0);
 
    if (tile_n != last_tile) {
      tile_changes++;
      endpt=cp;
      add_tile(last_tile,startpt,endpt);
      startpt=cp;
      last_tile = tile_n;
    }  

    cp = cp->next;
  }
  endpt=cp;
  //printf("end tile...add_tile ( %d ... ...)\n",tile_n);
  add_tile(tile_n,startpt,endpt); //cp->last);

  hp=sp;
  cp=sp;
  if (head_tile!=NULL) process_tile_list();
  else printf("NO TILES?\n");
}



void list_line(  struct pt * hp)
{ 
  int ix;
  struct pt * cp = hp;
  struct pt * sp = hp;
  ix=1;
  double txp;
  double typ;
  double txp2;
  double typ2;
  cp=hp;
  ix=1;

  long int last_tile=-1;
  long int tile_n;
  int tile_changes=-1;

  struct pt * startpt;
  struct pt * endpt;

  if (hp==NULL) {
    printf("list_line call with NULL ptr!\n");
    return;
  }
  while ( cp->next!=NULL) {
    txp  =  cp->xp;
    typ  =  cp->yp;
    txp2 =  cp->next->xp;
    typ2 =  cp->next->yp;

    tile_n = gen_tilenum( (txp+txp2)/2.0, (typ+typ2)/2.0);
    if (last_tile<0) startpt=cp; 
    if (tile_n != last_tile) {
      tile_changes++;
      if (tile_changes>0) {
        endpt=cp;
      }
    }  

    last_tile = tile_n;

    fprintf(fp_rep,"Line [%4d] seg [%s-%s]: %11.6f , %11.6f  |  %11.6f , %11.6f  Tile: %d",
         ix, cp->seg_nbr, cp->next->seg_nbr,typ, txp, typ2, txp2,
         gen_tilenum( (txp+txp2)/2.0, (typ+typ2)/2.0)
    );
    // double check for zero length line segments. if the following line shows up in the output it's an oops somewhere...
    if ((txp==txp2) && (typ==typ2)) fprintf(fp_rep," zero length line segment?");
    fprintf(fp_rep,"\n");
    ix++;
    cp = cp->next;
  }
  endpt=cp;
  fprintf(fp_rep,"add_tile ( %d ... ...)\n",tile_n);
  txp  =  cp->xp;
  typ  =  cp->yp;
  txp2 =  hp->xp;
  typ2 =  hp->yp;
#define TOLERANCE 0.0001
    if ( (fabs(txp-txp2) < TOLERANCE) && fabs(typ-typ2)<TOLERANCE) {
      fprintf(fp_rep,"Confirm coordinates at tail of list are same head of list...i.e. deed meets and bounds description.\n");
      good_lists++;
    }
    else {
      fprintf(fp_rep,"list does not satisfy meets and bounds test...? %f | %f     %f | %f\n",txp, txp2, typ, typ2);
      bad_lists++;
    }
    fprintf(fp_rep," rough estimate of tile changes %d\n",tile_changes);
  hp=sp;
  cp=sp;
}

void add_pts_to_master()
{
  double txp;
  double typ;
  double txp2;
  double typ2;
  if (head_pt_m!=NULL) {
    current_pt_m = tail_pt_m;
    if (head_pt!=NULL) {
//first point in head should be same as last pt in tail on master...
      txp  =  tail_pt_m->xp;
      typ  =  tail_pt_m->yp;
      txp2 =  head_pt->xp;
      typ2 =  head_pt->yp;
      if ((txp==txp2) && (typ==typ2)) {
        t_pt = head_pt;
        delete t_pt;
        head_pt = head_pt->next;
      }
      current_pt_m->next = head_pt;
      head_pt->last=current_pt_m;
      while (head_pt->next !=NULL) head_pt=head_pt->next;
      tail_pt_m=head_pt;
    }
  }
  else {
    head_pt_m=head_pt;
    current_pt_m=head_pt_m;
    tail_pt_m=head_pt_m;  
  }
  current_pt_m=head_pt_m;
  head_pt=NULL;
}

void erase_pt_list() {
  current_pt=head_pt;
  while (current_pt!=NULL) {
    current_pt = current_pt->next;
    delete head_pt;
    head_pt = current_pt;
  }
}  

void erase_master_list() {
  current_pt_m=head_pt_m;
  while (current_pt_m!=NULL) {
    sum_boundary_pt(current_pt_m->yp, current_pt_m->xp); //track average for sign posting...
    current_pt_m = current_pt_m->next;
    delete head_pt_m;
    head_pt_m = current_pt_m;
  }
}  

void init_pt_list()
{
  if (head_pt!=NULL) add_pts_to_master();
}

void add_pt(double x, double y)
{
  bool done=false;
  struct pt * new_pt = new struct pt;
  new_pt->xp=x;
  new_pt->yp=y;
  strcpy(new_pt->seg_nbr,sseg_nbr);
  new_pt->last=new_pt->next=NULL;
  if (head_pt!=NULL) {
    current_pt = head_pt;
    while (!done) {
      if (x < current_pt->xp) {  //insert before
        done=true;
        if (current_pt->last!=NULL) { // in list insert
          new_pt->last = current_pt->last;
          new_pt->next = current_pt;
          current_pt->last=new_pt;
          new_pt->last->next=new_pt;
        }
        else { //new head
          current_pt->last=new_pt;
          new_pt->next=current_pt;
          head_pt = new_pt;
        }
      }
      else {
        if (x!=current_pt->xp) {
          if (current_pt->next!=NULL) current_pt = current_pt->next;
          else {  //insert after
            done=true;
            if (x!=current_pt->xp) { //possible fence post error where coordinate is on boundary...not a trivial chance...
              current_pt->next = new_pt;
              new_pt->last = current_pt;
            }
            else {
              if (y!=current_pt->yp) { //fence post error if x and y already in list...
                current_pt->next = new_pt;
                new_pt->last = current_pt;
              }
  //            else printf("point already exists. not added to list\n");
              delete new_pt;
            }
          }
        }
        else {// n/s running lines x is fixed...
          if (y<current_pt->yp) { //insert before
            done=true;
            if (current_pt->last!=NULL) { // in list insert
              new_pt->last = current_pt->last;
              new_pt->next = current_pt;
              current_pt->last=new_pt;
              new_pt->last->next=new_pt;
            }
            else { //new head
              current_pt->last=new_pt;
              new_pt->next=current_pt;
              head_pt = new_pt;
            }
          }
          else { //insert after ?
            if (y!=current_pt->yp) {          
              if (current_pt->next!=NULL) current_pt = current_pt->next;
              else {
                done=true;
                current_pt->next = new_pt;
                new_pt->last = current_pt;
              }
            }
            else {
              done=true;
  //            printf("n/s point already exists. not added to list\n");
              delete new_pt;
            }
          }
        } 
      }
    }
  }
  else head_pt=new_pt;
}

int maxtilespan=-1;

void create_line(double dlong, double dlat, double dlong2, double dlat2)
{
  double delta_long;
  double delta_lat;
  double delta_max;
  int ntilespan;

  double x;  // x and y are in "indexed" coordinates as tiles are 0-7 by 0-7...
  double y;  

  double xn;
  double yn;

  double sx;  // x and y are in "indexed" coordinates as tiles are 0-7 by 0-7...
  double sy;  

  double sxn;
  double syn;


  int ix;
  int iy;

  double lastx;
  double lasty;

  double m;
  double b;
     
  bool swap_flag=false;
 
//******* note: line needs further splitting if begining and ending lattitudes have different bucket spans
  set_bucket(dlong,dlat);
  double check_divisor = 1.0/sg_bucket_span(dlat2);
  divisor = 1.0/sg_bucket_span(dlat);
  if (     fabs(divisor - check_divisor) > SG_EPSILON ) {
    fprintf(fp_rep,"WARNING: Bucket span divisor ranges from %f to %f, using larger divisor to divide lines\n",divisor, check_divisor);
  //use larger divisor or first vertically chop line up...
  //use larger divisor and you get say two panes in the tile instead one when the line drawing goes to one pane per line segment...
    if (divisor < check_divisor) divisor = check_divisor; 
  }
  if (gen_tilenum(dlong,dlat)!=gen_tilenum(dlong2,dlat2)) {
 //   printf("Line crosses tile boundaries\n");

    delta_long = dlong-dlong2; //strtod((const char *)&swgs_dlong2,NULL);
    delta_lat  = dlat -dlat2; //strtod((const char *)&swgs_dlat2,NULL);
    if (fabs(delta_long)>fabs(delta_lat)) delta_max= delta_long; else delta_max=delta_lat;

    ntilespan = (int) ((fabs(delta_max)/0.125)+1.0);
  
    if (ntilespan>maxtilespan) maxtilespan=ntilespan;
//********* a few words on why the arbitrary tranform:
//********* you then have numbers on the scalar 0, 1, 2, .... which are then the tile boundaries
//********* so.... between the end points are a sequence of intercepts for the x axis and the y axis
//********* so.... compute the various x/y intercepts and sort em....
//********* then.. generate the line segments from p  ....p.....p
//*********                                         0      i     n
//********* there will be n-1 of them...
    x = (dlong + 1000.0)*divisor;  // arbitrary transform to keep math positive and "indexed" 
    y = (dlat  + 1000.0)*8.0;  
    xn= (dlong2 + 1000.0)*divisor;  // arbitrary transform to keep math positive and "indexed" 
    yn= (dlat2  + 1000.0)*8.0;  

    sx=x;
    sy=y;
    sxn=xn;
    syn=yn;

//    printf("original coordinates are      %f , %f  transformed coordinates %f , %f\n",dlong,dlat,x,y);
//    printf("                              %f , %f                          %f , %f\n",dlong2,dlat2,xn,yn);
//    printf("inverted transforms           %f , %f\n",x/divisor-1000.0, y/8.0-1000.0);  
//    printf("                              %f , %f\n",xn/divisor-1000.0, yn/8.0-1000.0);  

    init_pt_list();

    // start list with end points...

    add_pt( x,  y);  
    add_pt( xn, yn);

    if (x>xn) { //swap pt order...
      lastx=x;
      lasty=y;
      x=xn;
      y=yn;
      xn=lastx;
      yn=lasty;
      lastx=xn;
      lasty=yn;
    } 
    if (fabs(delta_long) < SG_EPSILON) {  //n-s line
       // i.e. x is fixed
      if (y>yn) { //swap y order
        lasty=y;
        y=yn;
        yn=lasty;
        lasty=y;
      }   
      if ((int) y==(int) yn) {
         ; // printf("ns1) y range is singular for %f to %f\n",y,yn);
      }
      else {
        if ((int)y+1 < (int)yn) {
          for (iy=(int)y+1; iy<(int)yn; iy++){
            add_pt( x, (double) iy);
          }
        }
        else {
          iy=(int) yn;
          add_pt( x, (double) iy);     
        }
      }
    }
    else {
      if (fabs(delta_lat) < SG_EPSILON) {  //e-w line
      //  printf("east west running line...\n");
        // i.e. y is fixed
        if ((int) x==(int) xn) {
          ; //  printf("ew1) x range is singular for %f to %f\n",x,xn);
        }
        else {
          if ((int)x+1 < (int)xn) {
            for (ix=(int)x+1; ix<(int)xn; ix++){
              add_pt( (double) ix, y);
            }
          }
          else {
            ix=(int) xn;
            add_pt( (double) ix,  y);     
          }
        }
      }
      else {  // y=mx+b geometry...
    
        m = (yn-y)/(xn-x);
        b = y-m*x;

        // check order 
        if (x>xn) { //swap pt order...
          lastx=x;
          lasty=y;
          x=xn;
          y=yn;
          xn=lastx;
          yn=lasty;
          lastx=xn;
          lasty=yn;
        } 
        lasty = y;
        lastx = x;
        //y=mx+b
        //m=(yn-y)/(xn-x)
        //b= y-mx
        //x=(y-b)/m
        if ((int) x==(int) xn) {
          // y range then...
          if (y<yn) { // y increases with x
            if ((int) y==(int) yn) {
             ; // printf("1) y range is singular for %f to %f\n",y,yn);
              fprintf(fp_rep,"Warning, this case should not occur if line crosses tile boundaries!\n");
            }
            else {
              if ((int)y+1 < (int)yn) {
                for (iy=(int)y+1; iy<(int)yn; iy++){
                  add_pt( ((double)iy-b)/m , (double)iy);
                }
              }
              else {
                iy=(int) yn;
                add_pt( ((double)iy-b)/m , (double)iy);
              } 
            } 
          }
          else { //y decreases with x
            if ((int) y==(int) yn) {
             // printf("4) y range is singular for %f to %f\n",y,yn);
              fprintf(fp_rep,"Warning, this case should not occure if line crosses tile boundaries!\n");
              add_pt((y-b)/m,y);
              add_pt((yn-b)/m,yn);
            }
            else {
              if ((int)yn+1 < (int) y) {
             //   printf("5) y ranges from %f to %d through %d up to %f\n", yn, (int) yn+1, (int) y, y);
                for (iy=(int)yn+1; iy<(int)y; iy++){
                  add_pt( ((double)iy-b)/m , (double)iy);
                }
              }
              else {
             //   printf("6) y ranges from %f through %d up to %f\n", yn, (int) y, y);   
                iy=(int) y;
                add_pt( ((double)iy-b)/m , (double)iy);
              }  
            } 
          }
        }  
        else {
          if ((int)x+1 < (int)xn) {
          //  printf("2) x ranges from %f to %d through %d up to %f\n", x, (int) x+1, (int) xn, xn);
            for (ix=(int)x+1; ix<(int)xn; ix++){
              add_pt( (double) ix,  m*((double)ix)+b);
            }
          }
          else {
          //  printf("3) x ranges from %f through %d up to %f\n", x, (int) xn, xn);
            ix=(int) xn;
            add_pt( (double) ix,  m*((double)ix)+b);     
          }
          // y range then...
          if (y<yn) { // y increases with x
            if ((int) y==(int) yn) {
            ; //  printf("7) y range is singular for %f to %f (no points added)\n",y,yn);
            }
            else {
              if ((int)y+1 < (int)yn) {
            //    printf("8) y ranges from %f to %d through %d up to %f\n", y, (int) y+1, (int) yn, yn);
                for (iy=(int)y+1; iy<(int)yn; iy++){
                  add_pt( ((double)iy-b)/m , (double)iy);
                }
              }
              else {
            //    printf("9) y ranges from %f through %d up to %f\n", y, (int) yn, yn); 
                iy=(int) yn;
                add_pt( ((double)iy-b)/m , (double)iy);
              }   
            } 
          }
          else { //y decreases with x
            if ((int) y==(int) yn) {
            ; //  printf("10) y range is singular for %f to %f\n",y,yn);
            }
            else {
              if ((int)yn+1 < (int) y) {
            //    printf("11) y ranges from %f to %d through %d up to %f\n", yn, (int) yn+1, (int) y, y);
                for (iy=(int)yn+1; iy<(int)y; iy++){
                  add_pt( ((double)iy-b)/m , (double)iy);
                }
              }
              else {
            //    printf("12) y ranges from %f through %d up to %f\n", yn, (int) y, y);   
                iy=(int) y;
                add_pt( ((double)iy-b)/m , (double)iy); 
              } 
            } 
          }
        }// y=mx+b 
      }// e/w?
    }// n/s?
    //assuming ya lived to reach here what are the pts....
  } //line crossing tile boundaries
  else {//line lies within tile boundaries
  //  printf("line is within tile boundaries\n");
    x = (dlong + 1000.0)*divisor;  // arbitrary transform to keep math positive and "indexed" 
    y = (dlat  + 1000.0)*8.0;  
    xn= (dlong2 + 1000.0)*divisor;  // arbitrary transform to keep math positive and "indexed" 
    yn= (dlat2  + 1000.0)*8.0;  
 //   printf("original coordinates are      %f , %f  transformed coordinates %f , %f\n",dlong,dlat,x,y);
 //   printf("                              %f , %f                          %f , %f\n",dlong2,dlat2,xn,yn);
 //   printf("inverted transforms           %f , %f\n",x/divisor-1000.0, y/8.0-1000.0);  
 //   printf("                              %f , %f\n",xn/divisor-1000.0, yn/8.0-1000.0);  
    init_pt_list();

    // start list with end points...

    add_pt( x,  y);  
    add_pt( xn, yn);
    if (x>xn) { //swap pt order...
    //        printf("swap pt order...\n");
  //    if (swap_flag) swap_flag=false else swap_flag=true;
    }   
  }
  current_pt=head_pt;
  ix=1;
  double txp;
  double typ;
  double txp2;
  double typ2;

  current_pt=head_pt;
  end_pt=head_pt;
  ix=1;
  if (current_pt!=NULL) {
    while (current_pt!=NULL) {
      current_pt->xp =  (current_pt->xp/divisor)-1000.0;
      current_pt->yp =  (current_pt->yp/8.0)-1000.0;
      ix++;
      current_pt = current_pt->next;  
    }
  }
  else {
    fprintf(fp_rep,"Warning: create_line:current_pt is null\n");
  }
  current_pt=head_pt;
  if (fabs(dlong-current_pt->xp) < SG_EPSILON ) { 
    if (fabs(dlat-current_pt->yp) < SG_EPSILON ) { 
      ; //printf("and y checks\n");
    }
    else {
      if (fabs(dlat2-current_pt->yp) < SG_EPSILON ) { 
      //  printf("but y looks like yn, swap\n");
        swap_flag=true;
      }
      else {
        fprintf(fp_rep,"(1) WARNING first point is neither x,y or xn, yn\n");
      }
    }  
  }
  else {
    if (fabs(dlong2-current_pt->xp) < SG_EPSILON ) { 
      //printf("first point looks like xn swap the order\n");
      swap_flag=true;
    }
    else {
      fprintf(fp_rep,"(2) WARNING first point is neither x,y or xn, yn\n");
    }
  }
  if (swap_flag) {
    current_pt=head_pt;
    while (current_pt!=NULL) {
      t_pt=current_pt;
      end_pt=current_pt; 
      current_pt = current_pt->next;
      t_pt->next = t_pt->last;
      t_pt->last = current_pt;      
    }
    t_pt=head_pt;
    head_pt=end_pt;
    end_pt=t_pt;
  }
}


void create_cw_arc(double starta, double enda, double radnm, double clong, double clat, double dlongs, double dlats, double dlonge, double dlate)
{
//72
//setup for clockwise arc
#define NUM_ARC_SEGMENTS 72
  double sa; 
  double ea;
  double ca;
  double ca_norm;

  double az1;
  double az2;
  double s;


  sa=starta;
  ea=enda;
  if (sa>ea) sa-=360.0;
//  sa+=180.0;//get ac to flighgear bearings aligned
//  ea+=180.0;
  double span = ea-sa;

  int i;
  double coordinates[NUM_ARC_SEGMENTS+1][2];
  double delta_d = span / ((double)(NUM_ARC_SEGMENTS));

  double xx;
  double yy;
 // double az2;
//unit circle
  ca=sa+delta_d;
  coordinates[0][0]=dlongs;
  coordinates[0][1]=dlats;
  coordinates[NUM_ARC_SEGMENTS][0]=dlonge;
  coordinates[NUM_ARC_SEGMENTS][1]=dlate;
 
  for (i=1; i<NUM_ARC_SEGMENTS; i++) {
    geo_direct_wgs_84( 0.0 , clat, clong, ca, radnm*SG_NM_TO_METER, &yy, &xx, &az2);
    if (fabs(xx) < SG_EPSILON) xx=0.0;
    if (fabs(yy) < SG_EPSILON) yy=0.0;
    coordinates[i][0]=xx;
    coordinates[i][1]=yy;
    geo_inverse_wgs_84(0.0, clat, clong, yy, xx, &az1, &az2, &s);
    ca_norm=ca; //+180;
    if (ca_norm>360.0) ca_norm-=360.0;
    ca+=delta_d;
  }

  ca=sa;
  for (i=0; i<NUM_ARC_SEGMENTS+1; i++) {
    xx = coordinates[i][0];
    yy = coordinates[i][1];
    geo_inverse_wgs_84(0.0, clat, clong, yy, xx, &az1, &az2, &s);
    ca_norm=ca; //+180;
    if (ca_norm>360.0) ca_norm-=360.0;
    ca+=delta_d;
  }
  for (i=0; i<NUM_ARC_SEGMENTS; i++) {
    init_pt_list();
    if ( ( coordinates[i][0] == coordinates[i+1][0] ) && ( coordinates[i][1] == coordinates[i+1][1] ) ) {
      ;
    }
    else {
      create_line(coordinates[i][0], coordinates[i][1],coordinates[i+1][0], coordinates[i+1][1]);
    }
  }
}

void create_ccw_arc(double starta, double enda, double radnm, double clong, double clat, double dlongs, double dlats, double dlonge, double dlate)
{
  double sa; 
  double ea;
  double ca;
  double ca_norm;

  double az1;
  double az2;
  double s;


  sa=starta;
  ea=enda;
  if (ea>sa) sa+=360.0;
//  sa+=180.0;//get ac to flighgear bearings aligned
//  ea+=180.0;
  double span = sa-ea;

  int i;
  double coordinates[NUM_ARC_SEGMENTS+1][2];
  double delta_d = span / ((double)(NUM_ARC_SEGMENTS));

  double xx;
  double yy;
 // double az2;
//unit circle
  ca=sa-delta_d;
  coordinates[0][0]=dlongs;
  coordinates[0][1]=dlats;
  coordinates[NUM_ARC_SEGMENTS][0]=dlonge;
  coordinates[NUM_ARC_SEGMENTS][1]=dlate;


  for (i=1; i<NUM_ARC_SEGMENTS; i++) {                       //   lat  long
    geo_direct_wgs_84( 0.0 , clat, clong, ca, radnm*SG_NM_TO_METER, &yy, &xx, &az2);
    if (fabs(xx) < SG_EPSILON) xx=0.0;
    if (fabs(yy) < SG_EPSILON) yy=0.0; 
    coordinates[i][0]=xx; //long
    coordinates[i][1]=yy; //lat
    geo_inverse_wgs_84(0.0, clat, clong, yy, xx, &az1, &az2, &s);
    ca_norm=ca; //+180;
    ca-=delta_d;
  }
  ca=sa;
  for (i=0; i<NUM_ARC_SEGMENTS+1; i++) {
    xx = coordinates[i][0];
    yy = coordinates[i][1];
    geo_inverse_wgs_84(0.0, clat, clong, yy, xx, &az1, &az2, &s);
    ca_norm=ca; //+180;
    if (ca_norm>360.0) ca_norm-=360.0;
    ca-=delta_d;
  }
  for (i=0; i<NUM_ARC_SEGMENTS; i++) {
    init_pt_list();
    if ( ( coordinates[i][0] == coordinates[i+1][0] ) && ( coordinates[i][1] == coordinates[i+1][1] ) ) {
      ;
    }
    else {
      create_line(coordinates[i][0], coordinates[i][1],coordinates[i+1][0], coordinates[i+1][1]);
    }

  }
}


void create_circle(double radnm, double clong, double clat)
{
//72 is nice and smooth on those humongous Class B outer rings...
#define NUM_SEGMENTS 72 
  int i;
//  double vertices[NUM_SEGMENTS][2];
  double coordinates[NUM_SEGMENTS][2];
  double delta_d = 360.0 / ((double)(NUM_SEGMENTS));
  double xx;
  double yy;
  double az1;
  double az2;
  double s;
  
//unit circle
  for (i=0; i<NUM_SEGMENTS; i++) {
    geo_direct_wgs_84( 0.0 , clat, clong, (double)(i)*delta_d, radnm*SG_NM_TO_METER, &yy, &xx, &az2);
    if (fabs(xx) < SG_EPSILON) xx=0.0;
    if (fabs(yy) < SG_EPSILON) yy=0.0;
    coordinates[i][0]=xx;
    coordinates[i][1]=yy;
    geo_inverse_wgs_84(0.0, clat, clong, yy, xx, &az1, &az2, &s);
  }
  for (i=0; i<NUM_SEGMENTS-1; i++) {
    init_pt_list();
    create_line(coordinates[i][0], coordinates[i][1],coordinates[i+1][0], coordinates[i+1][1]);
  }
  init_pt_list();
  create_line(coordinates[NUM_SEGMENTS-1][0], coordinates[NUM_SEGMENTS-1][1],coordinates[0][0], coordinates[0][1]);
}

//----------------------------------------------------- decode DAFIF record info routines ---------------------------------------

void set_subpath()
{
  if (dlong>=0.0) {
    ew='e';
    ilongxxx    = (int) dlong;  //truncate    
    fremlongxxx = dlong - trunc(dlong); //(double) ilongxxx;         
    ilongxxx    = (int) (dlong+0.999999);  
    ilongxx     = (int) (ilongxxx+9.999999)/10;
  }
  else {
    ew ='w';
    ilongxxx = (int) (0.9999999-dlong); 
    fremlongxxx = fabs((double) dlong) - trunc(fabs(dlong));         
    ilongxx = (int)(ilongxxx+9.999999)/10;
    ilongxxx = -ilongxxx;
    ilongxx  = -ilongxx;
  }
  if (dlat>=0) {
    ns='n';
    ilatxxx = (int)dlat;  //truncate    
    fremlatxxx = dlat - trunc(dlat);         
    ilatxxx = (int)dlat;  //round down
    ilatxx = (ilatxxx)/10;
  }
  else {
    ns='s';
    ilatxxx = (int)(-dlat);  //truncate    
    fremlatxxx = fabs(dlat) - trunc(fabs(dlat));// - (double) ilatxxx;         
    ilatxxx = (int)(-dlat);  //round down
    ilatxx = (ilatxxx)/10;
    ilatxxx = -ilatxxx;
    ilatxx  = -ilatxx;
  }
  fremlatxxx = fremlatxxx*8.0;
  fremlongxxx = fremlongxxx*8.0;
  x = (int)fremlongxxx; //+0.999999;
  y = (int)fremlatxxx; //+0.999999;
  sprintf(subpath,"%c",ew);      // [e]
  if (abs(ilongxx)<10) sprintf(subpath,"%s0%d",subpath,abs(ilongxx));  // [e0x]
  else sprintf(subpath,"%s%d",subpath,abs(ilongxx));      // [exx]
  sprintf(subpath,"%s0%c",subpath,ns);  //  [e0x0n] or [exx0xn]
  sprintf(subpath,"%s%d",subpath,abs(ilatxx));     // [e0xxnxx]
  sprintf(subpath,"%s0/%c",subpath,ew);        // [e0xxnxx0
  if (abs(ilongxxx)<100) sprintf(subpath,"%s0%d",subpath,abs(ilongxxx));
  else sprintf(subpath,"%s%d",subpath,abs(ilongxxx));
  sprintf(subpath,"%s%c",subpath,ns);
  if (abs(ilatxxx)<10) sprintf(subpath,"%s0%d/",subpath,abs(ilatxxx));
  else sprintf(subpath,"%s%d/",subpath,abs(ilatxxx));
}

double decode_altitude(char *a)
{
  float small_alt;

  bool fl=false;
  int i=0;
  int i2=0;
  int j=0;
  altitude=0.0;
  alt_digits[i]=type_digits[j]=0;
  if (a[0]=='U') {  //"UNLTD" for some warning and prohibited areas...like off the east coast of Cont. US
    return UPPER_ALTITUDE_LIMIT; //for unlimited upper altitudes set altitude to 14,000 feet
  }
  if (a[0]=='F') {
    fl=true;
    i2=2; 
  }
  while (isdigit(a[i2])) {
    alt_digits[i]=a[i2];
    i++;
    i2++;
    alt_digits[i]=0;
  }
  while (isupper(a[i2])) {
    type_digits[j]=a[i2];
    i2++;
    j++;
    type_digits[j]=0;
  }
  if (alt_digits[0]==0) {
    altitude=0.0; 
  }
  else {
    sscanf(alt_digits,"%f",&small_alt);
    altitude = (double) small_alt;
    if (fl) {
      altitude*=100.0;
    }
  }
  return altitude;
}

void set_floor_alt()  // sets agl's artificially to zero when alt_low_type is "AGL" 
{ 
  floor_alt = altitude_low-altitude_high;
  if (strncmp(alt_low_type,"AGL",3) ==0 ) floor_alt = -altitude_high;
}

void do_decode_altitudes()
{
  altitude_low = decode_altitude((char *)&plower_alt);
  sprintf(alt_low_type,"%s",type_digits);
  altitude_high = decode_altitude((char *)&pupper_alt);
  sprintf(alt_high_type,"%s",type_digits);
  if (strncmp(plower_alt,"SURFACE",7)==0) {
    floor_alt = -(class_height[iclass]+CLASS_FLOOR_ADD);
   }
  else { // does not start at surface need to calculate floor and generate a file..
    set_floor_alt();
  }
//bug hunting...set all lower altitudes to surface....
//    floor_alt = -(class_height[iclass]+CLASS_FLOOR_ADD);

  if (strncmp(alt_high_type,"AGL",3) !=0 ){
    high_agl_flag = false;
  }
  else {
    high_agl_flag = true;
  }
  if (altitude_low>= UPPER_ALTITUDE_LIMIT) high_altitude_flag=true; else high_altitude_flag=false;

  if (high_agl_flag) fprintf(fp_rep,"upper level is AGL, no output \n");
  if (high_altitude_flag) fprintf(fp_rep," lower level is at or above altitude limit [%f]\n",UPPER_ALTITUDE_LIMIT);
  if ((!high_agl_flag) && (!high_altitude_flag)) {
    if (altitude_high>UPPER_ALTITUDE_LIMIT) {
      fprintf(fp_rep, "upper altitude limit clipped from [%f] to [%f]\n",altitude_high,UPPER_ALTITUDE_LIMIT);
      altitude_high=UPPER_ALTITUDE_LIMIT;
    }
  }
}
void do_decode_radius()
{
  ir = (int) r;
  rr = ir;
  rr = (r-ir)*10.0;
  irr = (int)rr;
}
void do_decode_radius_from_start_r()
{
  ir = (int) start_r;
  rr = ir;
  rr = (start_r-ir)*10.0;
  irr = (int)rr;
}

void decode_line_parameters()
{
  dlongp1 = strtod((const char *)&swgs_dlong1,NULL);
  dlatp1 = strtod((const char *)&swgs_dlat1,NULL);
  dlongp2 = strtod((const char *)&swgs_dlong2,NULL);
  dlatp2 = strtod((const char *)&swgs_dlat2,NULL);
          
  geo_inverse_wgs_84(0.0, dlatp1, dlongp1, dlatp2,dlongp2, &start_az, &az2,&start_r);
  start_r*=SG_METER_TO_NM;
  do_decode_radius_from_start_r();
}

void decode_arc_parameters()
{
  dlongc = strtod((const char *)&swgs_dlong0,NULL);
  dlatc = strtod((const char *)&swgs_dlat0,NULL);
  if (strncmp(sshap,"R",1)==0) { // R - CLOCKWISE ARC 
    dlongp1 = strtod((const char *)&swgs_dlong1,NULL);
    dlatp1 = strtod((const char *)&swgs_dlat1,NULL);
    dlongp2 = strtod((const char *)&swgs_dlong2,NULL);
    dlatp2 = strtod((const char *)&swgs_dlat2,NULL);
  }
  if (strncmp(sshap,"L",1)==0) { // R - COUNTERCLOCKWISE ARC 
    dlongp2 = strtod((const char *)&swgs_dlong1,NULL);
    dlatp2 = strtod((const char *)&swgs_dlat1,NULL);
    dlongp1 = strtod((const char *)&swgs_dlong2,NULL);
    dlatp1 = strtod((const char *)&swgs_dlat2,NULL);
  }
  geo_inverse_wgs_84(0.0, dlatc, dlongc, dlatp1,dlongp1, &start_az, &az2,&start_r);
  geo_inverse_wgs_84(0.0, dlatc, dlongc, dlatp2,dlongp2, &end_az, &az2,&end_r);

  start_r*=SG_METER_TO_NM;
  end_r*=SG_METER_TO_NM;
  ave_r = (start_r+end_r)/2.0;      
  ave_r = round(ave_r*10.0)/10.0;

  if ( fabs(ave_r-start_r) > fabs(ave_r-end_r)) r_diff= fabs(ave_r-start_r); else r_diff= fabs(ave_r-end_r);
  if (r_diff>max_r_diff) max_r_diff=r_diff;

}

//-----------------------------------do routines...main translation of DAFIF information into formatted output and generated .ac and tile files...
///
void do_tile_list()
{
  fprintf(fp_tiles,"(%4s) %85s <-> %s",picao,tile_filename, tile_addline);
}

void do_tile_update()
{
  if (process_class(iclass)) {
    do_tile_list();
    string command = "mkdir -p "; command += SGPath(tile_filename).dir();
    system(command.c_str());
    FILE * ft = fopen(tile_filename,"a");
    if (ft!=NULL) {
      fprintf(ft,"%s\n",tile_addline);
      fclose(ft);
      fprintf(fp_rep,"added line: [%s]\nto file: [%s]\n",tile_addline,tile_filename);
    } 
    else { 
      fprintf(fp_rep,"WARNING! COULD NOT APPEND TO FILE [%s]\n",tile_filename);
    }
  }
}


#define SIGN_HEIGHT_ABOVE_FIELD 6000.00
#define SIGN_HEIGHT_ABOVE_NAVAID 10300.00
#define SIGN_HEIGHT_ABOVE_WAYPOINT -2200.00
#define SIGN_HEIGHT_ABOVE_T_WAYPOINT 100.0


void safe_line(char * tl) 
{
  int i=0;
  while (tl[i]!='\0') {
    if (tl[i]=='\'') tl[i]=' ';
    i++;
  }
}

void write_sign_files()
{
  FILE * s_fp;
  char filename[1000];
// do tile update
  dlong = strtod((const char *)&wgs_dlong,NULL);
  dlat = strtod((const char *)&wgs_dlat,NULL);

  double d_elev = strtod((const char *)&elev,NULL);

  set_subpath();
  set_bucket(dlong,dlat);
  tilenum = gen_index();
  sprintf( tile_filename, "%s/Objects/%s%d.stg",output_base.c_str(), subpath, tilenum );
  sprintf(tile_addline,"OBJECT_STATIC %s.xml %s %s %6.1f 0.0\n",airport_icao,wgs_dlong,wgs_dlat, (d_elev+SIGN_HEIGHT_ABOVE_FIELD)*SG_FEET_TO_METER);
  fprintf(fp_rep,"update tile file [%s]\nwith line [%s]\n",tile_filename,tile_addline);
  string command = "mkdir -p "; 
  command += SGPath(tile_filename).dir();
  // cout << command << endl;
  system(command.c_str());
  FILE * ft = fopen(tile_filename,"a");
  if (ft!=NULL) {
    fprintf(ft,"%s\n",tile_addline);
    fclose(ft);
  } 
  else { 
     fprintf(fp_rep,"WARNING! COULD NOT APPEND TO FILE [%s]\n",tile_filename);
  }
// write xml file
  sprintf( filename, "%s/Objects/%s%s.xml",output_base.c_str(), subpath,airport_icao);
  fprintf(fp_rep,"write xml file [%s]\n",filename);
  s_fp = fopen(filename,"w+");
  if (s_fp != NULL) {
     fprintf(s_fp,
       "<?xml version=\"1.0\"?>\n<PropertyList>\n  <path>%s.ac</path>\n  <animation>\n   <type>billboard</type>\n  </animation>\n  <animation>\n   <type>select</type>\n   <object-name>%s</object-name>\n",
       airport_icao, airport_icao);
     fprintf(s_fp,
       "   <condition>\n      <property>/sim/airspace_signs/enabled</property>\n      <property>/sim/airspace_signs_airport/enabled</property>\n   </condition>\n  </animation>\n</PropertyList>\n");
    fclose(s_fp);
  }
// write ac file
  sprintf( filename, "%s/Objects/%s%s.ac",output_base.c_str(), subpath,airport_icao);
  fprintf(fp_rep,"write ac file [%s]\n",filename);
  s_fp = fopen(filename,"w+");
  if (s_fp != NULL) {
     fprintf(s_fp,
     "AC3Db\nMATERIAL \"sign\" rgb 1 1 1 amb 1 1 1 emis 1 1 1 spec 0 0 0 shi 0 trans 0\nOBJECT poly\nname \"%s\"\ntexture \"%s.rgb\"",airport_icao, airport_icao);
     fprintf(s_fp,"\nnumvert 4\n-2048  400 0\n-2048 1424 0\n 2048 1424 0\n 2048  400 0\nnumsurf 2\nSURF 0x00\nmat 0\nrefs 3\n1 0 1\n0 0 0\n2 1 1\nSURF 0x00\nmat 0\nrefs 3\n2 1 1\n0 0 0\n3 1 0\nkids 0\n");
    fclose(s_fp);
  }
//create rgb file

  
  char command_str[1000];

  char * st = lookup_state((char *)&state_prov);
  if (strlen(st)!=2) 
    sprintf(command_str,"convert -size 512x128 xc:none -gravity center -encoding None -font Helvetica-Bold -fill black -pointsize 100 -gravity Center -draw \"text 0,-30 '%s'\" -pointsize 20 -draw \"text 0,40 '%s'\" -compress RLE SGI:",
           airport_icao,name);
  else
    sprintf(command_str,"convert -size 512x128 xc:none -gravity center -encoding None -font Helvetica-Bold -fill black -pointsize 100 -gravity Center -draw \"text 0,-30 '%s'\" -pointsize 20 -draw \"text 0,40 '%s, %s'\" -compress RLE SGI:",
           airport_icao,name,st);
  
  //string 
  command = command_str; 
  command += output_base.c_str();
  command += "/Objects/";
  command += subpath;
  command += airport_icao;
  command += ".rgb"; 
  //cout << "create rgb file: ]" <<command << endl;
  fprintf(fp_rep,"Create rgb file: [%s]\n",command.c_str());
  system(command.c_str());
}

void write_sign_files_special( double xlat, double xlong, double x_elev)
{
  FILE * s_fp;
  char filename[1000];
// do tile update
  dlong = xlong;
  dlat = xlat;

  double d_elev = x_elev;

  set_subpath();
  set_bucket(dlong,dlat);
  tilenum = gen_index();
  sprintf( tile_filename, "%s/Objects/%s%d.stg",output_base.c_str(), subpath, tilenum );
  sprintf(tile_addline,"OBJECT_STATIC %s.xml %f %f %6.1f 0.0\n",bdry_ident_safe,dlong,dlat, (d_elev+SIGN_HEIGHT_ABOVE_FIELD)*SG_FEET_TO_METER);
  fprintf(fp_rep,"update tile file [%s]\nwith line [%s]\n",tile_filename,tile_addline);
  string command = "mkdir -p "; command += SGPath(tile_filename).dir();
  // cout << command << endl;
  system(command.c_str());
  FILE * ft = fopen(tile_filename,"a");
  if (ft!=NULL) {
    fprintf(ft,"%s\n",tile_addline);
    fclose(ft);
  } 
  else { 
    fprintf(fp_rep,"WARNING! COULD NOT APPEND TO FILE [%s]\n",tile_filename);
  }
// write xml file
  sprintf( filename, "%s/Objects/%s%s.xml",output_base.c_str(), subpath,bdry_ident_safe);
  fprintf(fp_rep,"write xml file [%s]\n",filename);
  s_fp = fopen(filename,"w+");
  if (s_fp != NULL) {
     fprintf(s_fp,
       "<?xml version=\"1.0\"?>\n<PropertyList>\n  <path>%s.ac</path>\n  <animation>\n   <type>billboard</type>\n  </animation>\n  <animation>\n   <type>select</type>\n   <object-name>%s</object-name>\n",
        bdry_ident_safe,  bdry_ident_safe);
     fprintf(s_fp,
       "   <condition>\n      <property>/sim/airspace_signs/enabled</property>\n      <property>/sim/airspace_signs_suas/enabled</property>\n   </condition>\n  </animation>\n</PropertyList>\n");
    fclose(s_fp);
  }
// write ac file
  sprintf( filename, "%s/Objects/%s%s.ac",output_base.c_str(), subpath, bdry_ident_safe);
  fprintf(fp_rep,"write ac file [%s]\n",filename);
  s_fp = fopen(filename,"w+");
  if (s_fp != NULL) {
     fprintf(s_fp,
     "AC3Db\nMATERIAL \"sign\" rgb 1 1 1 amb 1 1 1 emis 1 1 1 spec 0 0 0 shi 0 trans 0\nOBJECT poly\nname \"%s\"\ntexture \"%s.rgb\"", bdry_ident_safe,  bdry_ident_safe);
     fprintf(s_fp,"\nnumvert 4\n-2048  400 0\n-2048 1424 0\n 2048 1424 0\n 2048  400 0\nnumsurf 2\nSURF 0x00\nmat 0\nrefs 3\n1 0 1\n0 0 0\n2 1 1\nSURF 0x00\nmat 0\nrefs 3\n2 1 1\n0 0 0\n3 1 0\nkids 0\n");
    fclose(s_fp);
  }
//create rgb file
  char command_str[1000];
  sprintf(command_str,"convert -size 512x128 xc:none -gravity center -encoding None -font Helvetica-Bold -fill black -pointsize 55 -gravity Center -draw \"text 0,-30 '%s'\" -pointsize 20 -draw \"text 0,40 '%s'\" -compress RLE SGI:",
          bdry_ident_safe,pname);
  
 // string 
  command = command_str; 
  command += output_base.c_str();
  command += "/Objects/";
  command += subpath;
  command += bdry_ident_safe;
  command += ".rgb"; 
//  cout << "create rgb file: ]" <<command << endl;
  fprintf(fp_rep,"Create rgb file: [%s]\n",command.c_str());

  system(command.c_str());
}

char nav_line_1[13];
char nav_line_2[39];


void write_sign_files_navaid( double xlat, double xlong, double x_elev)
{
  FILE * s_fp;
  char filename[1000];
// do tile update
  dlong = xlong;
  dlat = xlat;

  double d_elev = x_elev;

  set_subpath();
  set_bucket(dlong,dlat);
  tilenum = gen_index();
  sprintf( tile_filename, "%s/Objects/%s%d.stg",output_base.c_str(), subpath, tilenum );
//  sprintf(tile_addline,"OBJECT_STATIC %s.xml %f %f %6.1f 0.0\n",nav_ident,dlong,dlat, (d_elev+SIGN_HEIGHT_ABOVE_NAVAID)*SG_FEET_TO_METER);
  sprintf(tile_addline,"OBJECT_STATIC %s.xml %f %f %6.1f 0.0\nOBJECT_STATIC na-pillar.xml %f %f 0.0 0.0\n",nav_ident,dlong,dlat, (d_elev+SIGN_HEIGHT_ABOVE_NAVAID)*SG_FEET_TO_METER,dlong,dlat);

  fprintf(fp_rep,"update tile file [%s]\nwith line [%s]\n",tile_filename,tile_addline);
  string command = "mkdir -p "; command += SGPath(tile_filename).dir();
  // cout << command << endl;
  system(command.c_str());
  FILE * ft = fopen(tile_filename,"a");
  if (ft!=NULL) {
    fprintf(ft,"%s\n",tile_addline);
    fclose(ft);
  } 
  else { 
     fprintf(fp_rep,"WARNING! COULD NOT APPEND TO FILE [%s]\n",tile_filename);
  }
// write xml file
  sprintf( filename, "%s/Objects/%s%s.xml",output_base.c_str(), subpath,nav_ident);
  fprintf(fp_rep,"write xml file [%s]\n",filename);
  s_fp = fopen(filename,"w+");
  if (s_fp != NULL) {
     fprintf(s_fp,
       "<?xml version=\"1.0\"?>\n<PropertyList>\n  <path>%s.ac</path>\n  <animation>\n   <type>billboard</type>\n  </animation>\n  <animation>\n   <type>select</type>\n   <object-name>%s</object-name>\n",
        nav_ident,  nav_ident);
     fprintf(s_fp,
       "   <condition>\n      <property>/sim/airspace_signs/enabled</property>\n      <property>/sim/airspace_signs_navaid/enabled</property>\n   </condition>\n  </animation>\n</PropertyList>\n");
    fclose(s_fp);
  }
// write ac file
  sprintf( filename, "%s/Objects/%s%s.ac",output_base.c_str(), subpath, nav_ident);
  fprintf(fp_rep,"write ac file [%s]\n",filename);
  s_fp = fopen(filename,"w+");
  if (s_fp != NULL) {
     fprintf(s_fp,
     "AC3Db\nMATERIAL \"sign\" rgb 1 1 1 amb 1 1 1 emis 1 1 1 spec 0 0 0 shi 0 trans 0\nOBJECT poly\nname \"%s\"\ntexture \"%s.rgb\"", nav_ident,  nav_ident);
     fprintf(s_fp,"\nnumvert 4\n-2048  400 0\n-2048 1424 0\n 2048 1424 0\n 2048  400 0\nnumsurf 2\nSURF 0x00\nmat 0\nrefs 3\n1 0 1\n0 0 0\n2 1 1\nSURF 0x00\nmat 0\nrefs 3\n2 1 1\n0 0 0\n3 1 0\nkids 0\n");
    fclose(s_fp);
  }

// write na-pillar.xml file redundant writes on n>1 per tile...thems the way it goes for now...
  sprintf( filename, "%s/Objects/%sna-pillar.xml",output_base.c_str(), subpath);
  fprintf(fp_rep,"write xml file [%s]\n",filename);
  s_fp = fopen(filename,"w+");
  if (s_fp != NULL) {
    fprintf(s_fp,"<?xml version=\"1.0\"?>\n<PropertyList>\n  <path>na-pillar.ac</path>\n  <animation>\n <type>select</type>\n   <object-name>na-pillar</object-name>\n   <condition>\n");
    fprintf(s_fp,"      <property>/sim/airspace_signs/enabled</property>\n      <property>/sim/airspace_signs_navaid/enabled</property>\n   </condition>\n  </animation>\n</PropertyList>\n");
    fclose(s_fp);
  }
// write wp-pillar.ac file redundant writes on n>1 per tile...thems the way it goes for now...
  sprintf( filename, "%s/Objects/%sna-pillar.ac",output_base.c_str(), subpath);
 
  fprintf(fp_rep,"write na-pillar.ac file [%s]\n",filename);
  s_fp = fopen(filename,"w+");
  if (s_fp != NULL) {
    fprintf(s_fp,"AC3Db\nMATERIAL \"Material.003\" rgb 0 0 0 amb 1 1 1 emis 0 0 0 spec 1 1 1 shi 72 trans 0.7\nOBJECT poly\nname \"na-pillar\"\nnumvert 16\n");
    fprintf(s_fp," -25     0  -25\n -25  4000  -25\n  25  4000  -25\n  25     0  -25\n -25     0  -25\n -25  4000  -25\n -25  4000   25\n -25     0   25\n");
    fprintf(s_fp,"  25     0  -25\n  25  4000  -25\n  25  4000   25\n  25     0   25\n -25     0   25\n -25  4000   25\n  25  4000   25\n  25     0   25\n");
    fprintf(s_fp,"numsurf 4\nSURF 0x30\nmat 0\nrefs 4\n0 0 0\n1 0 0\n2 0 0\n3 0 0\nSURF 0x30\nmat 0\nrefs 4\n4 0 0\n5 0 0\n6 0 0\n7 0 0\nSURF 0x30\nmat 0\nrefs 4\n");
    fprintf(s_fp,"8 0 0\n9 0 0\n10 0 0\n11 0 0\nSURF 0x30\nmat 0\nrefs 4\n12 0 0\n13 0 0\n14 0 0\n15 0 0\nkids 0\n");
    fclose(s_fp);
  }
//create rgb file
  safe_line((char *) &nav_line_2);
  char command_str[1000];
  sprintf(command_str,"convert -size 512x128 xc:none -gravity center -encoding None -font Helvetica-Bold -fill darkblue -pointsize 55 -gravity Center -draw \"text 0,-30 '%s'\" -pointsize 20 -draw \"text 0,40 '%s'\" -compress RLE SGI:",
          nav_line_1,nav_line_2);
  //string 
  command = command_str; 
  command += output_base.c_str();
  command += "/Objects/";
  command += subpath;
  command += nav_ident;
  command += ".rgb"; 
//  cout << "create rgb file: ]" <<command << endl;
 fprintf(fp_rep,"Create rgb file: [%s]\n",command.c_str());

  system(command.c_str());
}

void write_sign_files_waypoint( double xlat, double xlong, double x_elev)
{
  FILE * s_fp;
  char filename[1000];
// do tile update
  dlong = xlong;
  dlat = xlat;

  double d_elev = x_elev;

  set_subpath();
  set_bucket(dlong,dlat);
  tilenum = gen_index();
  sprintf( tile_filename, "%s/Objects/%s%d.stg",output_base.c_str(), subpath, tilenum );
//  sprintf(tile_addline,"OBJECT_STATIC %s-wp.xml %f %f %6.1f 0.0\n",wpt_ident,dlong,dlat, (d_elev+SIGN_HEIGHT_ABOVE_NAVAID)*SG_FEET_TO_METER);
  sprintf(tile_addline,"OBJECT_STATIC %s-wp.xml %f %f %6.1f 0.0\nOBJECT_STATIC wp-pillar.xml %f %f 0.0 0.0\n",wpt_ident,dlong,dlat, (d_elev+SIGN_HEIGHT_ABOVE_WAYPOINT)*SG_FEET_TO_METER,dlong,dlat);
  fprintf(fp_rep,"update tile file [%s]\nwith line [%s]\n",tile_filename,tile_addline);
  string command = "mkdir -p "; command += SGPath(tile_filename).dir();
  // cout << command << endl;
  system(command.c_str());
  FILE * ft = fopen(tile_filename,"a");
  if (ft!=NULL) {
    fprintf(ft,"%s\n",tile_addline);
    fclose(ft);
  } 
  else { 
     fprintf(fp_rep,"WARNING! COULD NOT APPEND TO FILE [%s]\n",tile_filename);
  }
// write xml file
  sprintf( filename, "%s/Objects/%s%s-wp.xml",output_base.c_str(), subpath,wpt_ident);
  fprintf(fp_rep,"write xml file [%s]\n",filename);
  s_fp = fopen(filename,"w+");
  if (s_fp != NULL) {
   fprintf(s_fp,
       "<?xml version=\"1.0\"?>\n<PropertyList>\n  <path>%s-wp.ac</path>\n  <animation>\n   <type>billboard</type>\n  </animation>\n  <animation>\n   <type>select</type>\n   <object-name>%s</object-name>\n",
        wpt_ident,  wpt_ident);

     fprintf(s_fp,
       "   <condition>\n      <property>/sim/airspace_signs/enabled</property>\n      <property>/sim/airspace_signs_wp/enabled</property>\n   </condition>\n  </animation>\n</PropertyList>\n");
    fclose(s_fp);
  }
// write wp-pillar.xml file redundant writes on n>1 per tile...thems the way it goes for now...
    sprintf( filename, "%s/Objects/%swp-pillar.xml",output_base.c_str(), subpath);

  fprintf(fp_rep,"write xml file [%s]\n",filename);
  s_fp = fopen(filename,"w+");
  if (s_fp != NULL) {
    fprintf(s_fp,"<?xml version=\"1.0\"?>\n<PropertyList>\n  <path>wp-pillar.ac</path>\n  <animation>\n <type>select</type>\n   <object-name>wp-pillar</object-name>\n   <condition>\n");
    fprintf(s_fp,"      <property>/sim/airspace_signs/enabled</property>\n      <property>/sim/airspace_signs_wp/enabled</property>\n   </condition>\n  </animation>\n</PropertyList>\n");
    fclose(s_fp);
  }

// write ac file
    sprintf( filename, "%s/Objects/%s%s-wp.ac",output_base.c_str(), subpath, wpt_ident);
  fprintf(fp_rep,"write ac file [%s]\n",filename);
  s_fp = fopen(filename,"w+");
  if (s_fp != NULL) {
      fprintf(s_fp,
      "AC3Db\nMATERIAL \"sign\" rgb 1 1 1 amb 1 1 1 emis 1 1 1 spec 0 0 0 shi 0 trans 0\nOBJECT poly\nname \"%s\"\ntexture \"%s-wp.rgb\"", wpt_ident,  wpt_ident);
    fprintf(s_fp,"\nnumvert 4\n-512  400 0\n-512  912 0\n 512  912 0\n 512  400 0\nnumsurf 2\nSURF 0x00\nmat 0\nrefs 3\n1 0 1\n0 0 0\n2 1 1\nSURF 0x00\nmat 0\nrefs 3\n2 1 1\n0 0 0\n3 1 0\nkids 0\n");
    fclose(s_fp);
  }
// write wp-pillar.ac file redundant writes on n>1 per tile...thems the way it goes for now...
    sprintf( filename, "%s/Objects/%swp-pillar.ac",output_base.c_str(), subpath);
  fprintf(fp_rep,"write wp-pillar.ac file [%s]\n",filename);
  s_fp = fopen(filename,"w+");
  if (s_fp != NULL) {
      fprintf(s_fp,"AC3Db\nMATERIAL \"Material.003\" rgb 0 0 0 amb 1 1 1 emis 0 0 0 spec 1 1 1 shi 72 trans 0.7\nOBJECT poly\nname \"wp-pillar\"\nnumvert 16\n");
    fprintf(s_fp," -25     0  -25\n -25  4000  -25\n  25  4000  -25\n  25     0  -25\n -25     0  -25\n -25  4000  -25\n -25  4000   25\n -25     0   25\n");
    fprintf(s_fp,"  25     0  -25\n  25  4000  -25\n  25  4000   25\n  25     0   25\n -25     0   25\n -25  4000   25\n  25  4000   25\n  25     0   25\n");
    fprintf(s_fp,"numsurf 4\nSURF 0x30\nmat 0\nrefs 4\n0 0 0\n1 0 0\n2 0 0\n3 0 0\nSURF 0x30\nmat 0\nrefs 4\n4 0 0\n5 0 0\n6 0 0\n7 0 0\nSURF 0x30\nmat 0\nrefs 4\n");
    fprintf(s_fp,"8 0 0\n9 0 0\n10 0 0\n11 0 0\nSURF 0x30\nmat 0\nrefs 4\n12 0 0\n13 0 0\n14 0 0\n15 0 0\nkids 0\n");
    fclose(s_fp);
  }


//create rgb file
  safe_line((char *) &desc);
  char command_str[1000];
//  sprintf(command_str,"convert -size 512x128 xc:none -gravity center -encoding None -font Helvetica-Bold -fill blue -pointsize 55 -gravity Center -draw \"text 0,-30 '%s'\" -pointsize 27 -draw \"text 0,40 '%s'\" -compress RLE SGI:",
//          wpt_ident,desc);
  sprintf(command_str,"convert -size 512x128 xc:none -gravity center -encoding None -font Helvetica-Bold -fill darkblue -pointsize 120 -gravity Center -draw \"text 0,-15 '%s'\" -compress RLE SGI:",
          wpt_ident);  

  //string 
  command = command_str; 
  command += output_base.c_str();
  command += "/Objects/";
  command += subpath;
  command += wpt_ident;
//  if (s_fp != NULL) {
    command += "-wp.rgb"; 
 
// cout << "create rgb file: ]" <<command << endl;
 fprintf(fp_rep,"Create rgb file: [%s]\n",command.c_str());

  system(command.c_str());
}
void write_sign_files_t_waypoint( double xlat, double xlong, double x_elev)
{
  FILE * s_fp;
  char filename[1000];
// do tile update
  dlong = xlong;
  dlat = xlat;

  double d_elev = x_elev;

  set_subpath();
  set_bucket(dlong,dlat);
  tilenum = gen_index();
  sprintf( tile_filename, "%s/Objects/%s%d.stg",output_base.c_str(), subpath, tilenum );
//  sprintf(tile_addline,"OBJECT_STATIC %s-wp.xml %f %f %6.1f 0.0\n",wpt_ident,dlong,dlat, (d_elev+SIGN_HEIGHT_ABOVE_NAVAID)*SG_FEET_TO_METER);
  sprintf(tile_addline,"OBJECT_STATIC %s-twp.xml %f %f %6.1f 0.0\nOBJECT_STATIC twp-pillar.xml %f %f %6.1f 0.0\n",wpt_ident,dlong,dlat, (d_elev+SIGN_HEIGHT_ABOVE_T_WAYPOINT)*SG_FEET_TO_METER,dlong,dlat,((d_elev+SIGN_HEIGHT_ABOVE_T_WAYPOINT)*SG_FEET_TO_METER-3600.0));
  fprintf(fp_rep,"update tile file [%s]\nwith line [%s]\n",tile_filename,tile_addline);
  string command = "mkdir -p "; command += SGPath(tile_filename).dir();
  // cout << command << endl;
  system(command.c_str());
  FILE * ft = fopen(tile_filename,"a");
  if (ft!=NULL) {
    fprintf(ft,"%s\n",tile_addline);
    fclose(ft);
  } 
  else { 
     fprintf(fp_rep,"WARNING! COULD NOT APPEND TO FILE [%s]\n",tile_filename);
  }
// write xml file
  sprintf( filename, "%s/Objects/%s%s-twp.xml",output_base.c_str(), subpath,wpt_ident);
  fprintf(fp_rep,"write xml file [%s]\n",filename);
  s_fp = fopen(filename,"w+");
  if (s_fp != NULL) {
    fprintf(s_fp,
      "<?xml version=\"1.0\"?>\n<PropertyList>\n  <path>%s-twp.ac</path>\n  <animation>\n   <type>billboard</type>\n  </animation>\n  <animation>\n   <type>select</type>\n   <object-name>%s</object-name>\n",
       wpt_ident,  wpt_ident);

    fprintf(s_fp,
      "   <condition>\n      <property>/sim/airspace_signs/enabled</property>\n      <property>/sim/airspace_signs_term_wp/enabled</property>\n   </condition>\n  </animation>\n</PropertyList>\n");
    fclose(s_fp);
  }
// write wp-pillar.xml file redundant writes on n>1 per tile...thems the way it goes for now...
  sprintf( filename, "%s/Objects/%stwp-pillar.xml",output_base.c_str(), subpath);
  fprintf(fp_rep,"write xml file [%s]\n",filename);
  s_fp = fopen(filename,"w+");
  if (s_fp != NULL) {
    fprintf(s_fp,"<?xml version=\"1.0\"?>\n<PropertyList>\n  <path>twp-pillar.ac</path>\n  <animation>\n <type>select</type>\n   <object-name>twp-pillar</object-name>\n   <condition>\n");
    fprintf(s_fp,"      <property>/sim/airspace_signs/enabled</property>\n      <property>/sim/airspace_signs_term_wp/enabled</property>\n   </condition>\n  </animation>\n</PropertyList>\n");
    fclose(s_fp);
  }

// write ac file
  sprintf( filename, "%s/Objects/%s%s-twp.ac",output_base.c_str(), subpath, wpt_ident);
  fprintf(fp_rep,"write ac file [%s]\n",filename);
  s_fp = fopen(filename,"w+");
  if (s_fp != NULL) {
    fprintf(s_fp,
      "AC3Db\nMATERIAL \"sign\" rgb 1 1 1 amb 1 1 1 emis 1 1 1 spec 0 0 0 shi 0 trans 0\nOBJECT poly\nname \"%s\"\ntexture \"%s-twp.rgb\"", wpt_ident,  wpt_ident);
    fprintf(s_fp,"\nnumvert 4\n-256  400 0\n-256 656 0\n 256 656 0\n 256  400 0\nnumsurf 2\nSURF 0x00\nmat 0\nrefs 3\n1 0 1\n0 0 0\n2 1 1\nSURF 0x00\nmat 0\nrefs 3\n2 1 1\n0 0 0\n3 1 0\nkids 0\n");
    fclose(s_fp);
  }
// write wp-pillar.ac file redundant writes on n>1 per tile...thems the way it goes for now...
  sprintf( filename, "%s/Objects/%stwp-pillar.ac",output_base.c_str(), subpath);
  fprintf(fp_rep,"write twp-pillar.ac file [%s]\n",filename);
  s_fp = fopen(filename,"w+");
  if (s_fp != NULL) {
    fprintf(s_fp,"AC3Db\nMATERIAL \"Material.003\" rgb 0 0 0 amb 1 1 1 emis 0 0 0 spec 1 1 1 shi 72 trans 0.7\nOBJECT poly\nname \"twp-pillar\"\nnumvert 16\n");
    fprintf(s_fp," -25     0  -25\n -25  4000  -25\n  25  4000  -25\n  25     0  -25\n -25     0  -25\n -25  4000  -25\n -25  4000   25\n -25     0   25\n");
    fprintf(s_fp,"  25     0  -25\n  25  4000  -25\n  25  4000   25\n  25     0   25\n -25     0   25\n -25  4000   25\n  25  4000   25\n  25     0   25\n");
    fprintf(s_fp,"numsurf 4\nSURF 0x30\nmat 0\nrefs 4\n0 0 0\n1 0 0\n2 0 0\n3 0 0\nSURF 0x30\nmat 0\nrefs 4\n4 0 0\n5 0 0\n6 0 0\n7 0 0\nSURF 0x30\nmat 0\nrefs 4\n");
    fprintf(s_fp,"8 0 0\n9 0 0\n10 0 0\n11 0 0\nSURF 0x30\nmat 0\nrefs 4\n12 0 0\n13 0 0\n14 0 0\n15 0 0\nkids 0\n");
    fclose(s_fp);
  }


//create rgb file
  safe_line((char *) &desc);
  char command_str[1000];
  sprintf(command_str,"convert -size 256x128 xc:none -gravity center -encoding None -font Helvetica-Bold -fill darkred -pointsize 60 -gravity Center -draw \"text 0,-40 '%s'\" -pointsize 40 -draw \"text 0,30 '%s'\" -compress RLE SGI:",
          wpt_ident,icao);
//  sprintf(command_str,"convert -size 512x128 xc:none -gravity center -encoding None -font Helvetica-Bold -fill darkred -pointsize 40 -gravity Center -draw \"text 0,-15 '%s'\" -compress RLE SGI:",
//          wpt_ident);  

  //string 
  command = command_str; 
  command += output_base.c_str();
  command += "/Objects/";
  command += subpath;
  command += wpt_ident;
//  if (s_fp != NULL) {
  command += "-twp.rgb"; 
 
 fprintf(fp_rep,"Create rgb file: [%s]\n",command.c_str());

// cout << "create rgb file: ]" <<command << endl;
  system(command.c_str());
}

void do_circle_segment()
{
  fprintf(fp_rep," C - CIRCLE: %s / %s radius: %s Nautical Miles",swgs_dlat0,swgs_dlong0,sradius1);  
  dlong = strtod((const char *)&swgs_dlong0,NULL);
  dlat = strtod((const char *)&swgs_dlat0,NULL);
  set_subpath();
  set_bucket(dlong,dlat);
  tilenum = gen_index();
  sprintf( tile_filename, "%s/Objects/%s%d.stg",output_base.c_str(), subpath, tilenum );
  r=strtod((const char *)&sradius1,NULL);
  do_decode_radius();
  if ( (!high_agl_flag) && (!high_altitude_flag) ) {
    if (process_class(iclass)) {
      fprintf(fp_rep," circle generated...\n"); 
      create_circle(r,dlong, dlat);
    }
    else {
      fprintf(fp_rep," circle not generated...\n");
    } 
  }
  else {
    fprintf(fp_rep," circle not generated...AGL or high altitude\n");
  }
  npoints++;
}

void do_rhumb_line_segment()
{
//  cout << " H - RHUMB LINE: " << swgs_dlat1 << " / " << swgs_dlong1 << " - " << swgs_dlat2 << " / " << swgs_dlong2; 
  fprintf(fp_rep," H - RHUMB LINE: %s / %s - %s / %s",swgs_dlat1, swgs_dlong1, swgs_dlat2, swgs_dlong2); 
  if ( (!high_agl_flag) && (!high_altitude_flag) ) {
    if (process_class(iclass)) {
      fprintf(fp_rep, " Line generated...\n");
      dlong = strtod((const char *)&swgs_dlong1,NULL);
      dlat = strtod((const char *)&swgs_dlat1,NULL);
      dlong2 = strtod((const char *)&swgs_dlong2,NULL);
      dlat2 = strtod((const char *)&swgs_dlat2,NULL);      
      create_line(dlong, dlat, dlong2, dlat2);
      set_bucket(dlong,dlat);
      set_subpath();
      set_bucket(dlong,dlat);
      tilenum = gen_index();
      sprintf( tile_filename, "%s/Objects/%s%d.stg", output_base.c_str(), subpath, tilenum );
      decode_line_parameters();
    }
  }
  else fprintf(fp_rep,"Line not generated upper altitude is AGL or high altituded\n");
}

void do_arc_segment()
{
  if (strncmp(sshap,"L",1)==0) fprintf(fp_rep," L - COUNTERCLOCKWISE ARC -- ");
  if (strncmp(sshap,"R",1)==0) fprintf(fp_rep," R - CLOCKWISE ARC -- ");
  if (strncmp(sderivation,"B",1)==0) fprintf(fp_rep,"   B - DISTANCE AND BEARING: \n");
  if (strncmp(sderivation,"E",1)==0) fprintf(fp_rep,"   E - END COORDINATES: \n");
  if ( (strncmp(sderivation,"R",1)==0) || (strncmp(sderivation,"E",1)==0) || (strncmp(sderivation,"B",1)==0))
  {
//    cout <<"from " << swgs_dlat1 << " / " << swgs_dlong1 << " to " << swgs_dlat2 << " / " << swgs_dlong2 
//         << " centered at " << swgs_dlat0 << " / " << swgs_dlong0 ; 
    fprintf(fp_rep,"from %s / %s to %s / %s centered at %s / %s",swgs_dlat1, swgs_dlong1, swgs_dlat2, swgs_dlong2, swgs_dlat0, swgs_dlong0 ); 
    if ( (!high_agl_flag) && (!high_altitude_flag) ) {
      if (process_class(iclass)) {
        fprintf(fp_rep," arc generated...\n");
        dlong = strtod((const char *)&swgs_dlong0,NULL);
        dlat = strtod((const char *)&swgs_dlat0,NULL);
        set_subpath();
        set_bucket(dlong,dlat);
        tilenum = gen_index();
        sprintf( tile_filename, "%s/Objects/%s%d.stg",  output_base.c_str(), subpath, tilenum );
        decode_arc_parameters();
        if (strncmp(sshap,"R",1)==0) { // R - CLOCKWISE ARC 
          create_cw_arc(start_az, end_az, ave_r, dlong, dlat, dlongp1, dlatp1, dlongp2, dlatp2);
        }
        if (strncmp(sshap,"L",1)==0) { // L - COUNTERCLOCKWISE ARC 
          create_ccw_arc(end_az, start_az, ave_r, dlong, dlat, dlongp2, dlatp2, dlongp1, dlatp1);
        }
      }
    } 
    else {
     fprintf(fp_rep," arc not generated...upper altitude limit is AGL or high altitude\n");     
    }    
  }  
}

void do_generalized_segment()
{
//  cout << " G - GENERALIZED:" << swgs_dlat1 << " / " << swgs_dlong1 << " - " << swgs_dlat2 << " / " << swgs_dlong2 << " (handled as rhumb line)"; 
  fprintf(fp_rep," G - GENERALIZED:%s / %s - %s / %s (handled as rhumb line", swgs_dlat1, swgs_dlong1, swgs_dlat2, swgs_dlong2); 
  do_rhumb_line_segment();
}

void do_greatcircle_segment()
{
  fprintf(fp_rep," B - GREAT CIRCLE\n...ignored...\n");
}

void do_point_segment()
{
  fprintf(fp_rep," A - POINT (WITHOUT RADIUS OR BEARING)\n...ignored...\n");
}

//************************************************ DAFIF record reading routines ****************************************


bool nextok(FILE* t)
{
  char tt;
  int status;
//printf("[");
  status =feof(t);
  if (status!=0) return false;
 // printf(".");
  tt=fgetc(t);
  status=feof(t);
  tt=ungetc(tt,t);
 // if(status!=0)  printf("x]"); else printf(".]");
  if (status!=0) return false; else return true;
//  return status;
}
char peek(FILE* t)
{
  char tt;
  char tback;
  char tret;
  tt=fgetc(t);
  tback=tt;
  tret=ungetc(tback,t);
  return tt;
}


void read_field(FILE * fp, char * f, int fs, char  d) {
  int i=0; 
  char ch;
  f[0]=i=0; 
  bool done =false;
  while (!done) {
    ch=fgetc(fp); 
    if (ch != d) {
      f[i]=ch;
      i++;
      if (i<fs) f[i] =0;
    } else done=true;
  }
}

void skip_record(FILE * fp)
{
  char ch;
  while (peek(fp)!='\n') { 
    ch=fgetc(fp); 
  }
  ch=fgetc(fp); 
}


void read_segment()
{
  read_field(fp_segment,(char *)&sseg_nbr,sizeof(sseg_nbr),tc);
  read_field(fp_segment,(char *)&sname,sizeof(sname),tc);
  read_field(fp_segment,(char *)&stype,sizeof(stype),tc);
  read_field(fp_segment,(char *)&sicao,sizeof(sicao),tc);
  read_field(fp_segment,(char *)&sshap,sizeof(sshap),tc);
  read_field(fp_segment,(char *)&sderivation,sizeof(sderivation),tc);
  read_field(fp_segment,(char *)&swgs_lat1,sizeof(swgs_lat1),tc);
  read_field(fp_segment,(char *)&swgs_dlat1,sizeof(swgs_dlat1),tc);
  read_field(fp_segment,(char *)&swgs_long1,sizeof(swgs_long1),tc);
  read_field(fp_segment,(char *)&swgs_dlong1,sizeof(swgs_dlong1),tc);
  read_field(fp_segment,(char *)&swgs_lat2,sizeof(swgs_lat2),tc);
  read_field(fp_segment,(char *)&swgs_dlat2,sizeof(swgs_dlat2),tc);
  read_field(fp_segment,(char *)&swgs_long2,sizeof(swgs_long2),tc);
  read_field(fp_segment,(char *)&swgs_dlong2,sizeof(swgs_dlong2),tc);
  read_field(fp_segment,(char *)&swgs_lat0,sizeof(swgs_lat0),tc);
  read_field(fp_segment,(char *)&swgs_dlat0,sizeof(swgs_dlat0),tc);
  read_field(fp_segment,(char *)&swgs_long0,sizeof(swgs_long0),tc);
  read_field(fp_segment,(char *)&swgs_dlong0,sizeof(swgs_dlong0),tc);
  read_field(fp_segment,(char *)&sradius1,sizeof(sradius1),tc);
  read_field(fp_segment,(char *)&sradius2,sizeof(sradius2),tc);
  read_field(fp_segment,(char *)&sbearing1,sizeof(sbearing1),tc);
  read_field(fp_segment,(char *)&sbearing2,sizeof(sbearing2),tc);
  read_field(fp_segment,(char *)&snav_ident,sizeof(snav_ident),tc);
  read_field(fp_segment,(char *)&snav_type,sizeof(snav_type),tc);
  read_field(fp_segment,(char *)&snav_ctry,sizeof(snav_ctry),tc);
  read_field(fp_segment,(char *)&snav_key_cd,sizeof(snav_key_cd),tc);
  read_field(fp_segment,(char *)&scycle_date,sizeof(scycle_date),tc2);

  fprintf(fp_rep,"%s",sseg_nbr); 

  if (strncmp(sshap,"A",1)==0) do_point_segment();
  if (strncmp(sshap,"B",1)==0) do_greatcircle_segment();
  if (strncmp(sshap,"C",1)==0) do_circle_segment();
  if (strncmp(sshap,"G",1)==0) do_generalized_segment();
  if (strncmp(sshap,"H",1)==0) do_rhumb_line_segment();
  if ((strncmp(sshap,"L",1)==0)||((strncmp(sshap,"R",1)==0))) do_arc_segment();
}
void read_suas_segment()
{
  read_field(fp_segment,(char *)&suas_sector,sizeof(suas_sector),tc);
  read_field(fp_segment,(char *)&sseg_nbr,sizeof(sseg_nbr),tc);
  read_field(fp_segment,(char *)&sname,sizeof(sname),tc);
  read_field(fp_segment,(char *)&stype,sizeof(stype),tc);
  read_field(fp_segment,(char *)&sicao,sizeof(sicao),tc);
  read_field(fp_segment,(char *)&sshap,sizeof(sshap),tc);
  read_field(fp_segment,(char *)&sderivation,sizeof(sderivation),tc);
  read_field(fp_segment,(char *)&swgs_lat1,sizeof(swgs_lat1),tc);
  read_field(fp_segment,(char *)&swgs_dlat1,sizeof(swgs_dlat1),tc);
  read_field(fp_segment,(char *)&swgs_long1,sizeof(swgs_long1),tc);
  read_field(fp_segment,(char *)&swgs_dlong1,sizeof(swgs_dlong1),tc);
  read_field(fp_segment,(char *)&swgs_lat2,sizeof(swgs_lat2),tc);
  read_field(fp_segment,(char *)&swgs_dlat2,sizeof(swgs_dlat2),tc);
  read_field(fp_segment,(char *)&swgs_long2,sizeof(swgs_long2),tc);
  read_field(fp_segment,(char *)&swgs_dlong2,sizeof(swgs_dlong2),tc);
  read_field(fp_segment,(char *)&swgs_lat0,sizeof(swgs_lat0),tc);
  read_field(fp_segment,(char *)&swgs_dlat0,sizeof(swgs_dlat0),tc);
  read_field(fp_segment,(char *)&swgs_long0,sizeof(swgs_long0),tc);
  read_field(fp_segment,(char *)&swgs_dlong0,sizeof(swgs_dlong0),tc);
  read_field(fp_segment,(char *)&sradius1,sizeof(sradius1),tc);
  read_field(fp_segment,(char *)&sradius2,sizeof(sradius2),tc);
  read_field(fp_segment,(char *)&sbearing1,sizeof(sbearing1),tc);
  read_field(fp_segment,(char *)&sbearing2,sizeof(sbearing2),tc);
  read_field(fp_segment,(char *)&snav_ident,sizeof(snav_ident),tc);
  read_field(fp_segment,(char *)&snav_type,sizeof(snav_type),tc);
  read_field(fp_segment,(char *)&snav_ctry,sizeof(snav_ctry),tc);
  read_field(fp_segment,(char *)&snav_key_cd,sizeof(snav_key_cd),tc);
  read_field(fp_segment,(char *)&scycle_date,sizeof(scycle_date),tc2);

  fprintf(fp_rep,"%s",sseg_nbr); 

  if (strncmp(sshap,"A",1)==0) do_point_segment();
  if (strncmp(sshap,"B",1)==0) do_greatcircle_segment();
  if (strncmp(sshap,"C",1)==0) do_circle_segment();
  if (strncmp(sshap,"G",1)==0) do_generalized_segment();
  if (strncmp(sshap,"H",1)==0) do_rhumb_line_segment();
  if ((strncmp(sshap,"L",1)==0)||((strncmp(sshap,"R",1)==0))) do_arc_segment();
}

void do_runway()
{
  bool good_elevations =true;
 
 // return; 
  if (!do_vfr_glideslope) return;

  if (strncmp(he_elev,"U",1)==0) good_elevations=false;
  if (strncmp(le_elev,"U",1)==0) good_elevations=false;
  if (good_elevations) {
    double dhe_elev = strtod((const char *)&he_elev,NULL);
    double dle_elev = strtod((const char *)&le_elev,NULL);  
// low
    dlong = strtod((const char *)&le_wgs_dlong,NULL);
    dlat = strtod((const char *)&le_wgs_dlat,NULL);
    set_subpath();
    set_bucket(dlong,dlat);
    tilenum = gen_index();
    sprintf( tile_filename, "%s/Objects/%s%d.stg",output_base.c_str(), subpath, tilenum );
//    sprintf(tile_addline,"OBJECT_SHARED Models/Airport/glide-slope-1nm-by-3d.ac %s %s %6.1f -%s\n",he_wgs_dlong,he_wgs_dlat, dhe_elev*SG_FEET_TO_METER,he_true_hdg);
    sprintf(tile_addline,"OBJECT_SHARED Models/Airport/glide-slope.xml %s %s %6.1f -%s\n",le_wgs_dlong,le_wgs_dlat, (dle_elev+50.0)*SG_FEET_TO_METER,le_true_hdg);
    iclass=3;  //kludge
    do_tile_update();
//high
    dlong = strtod((const char *)&he_wgs_dlong,NULL);
    dlat = strtod((const char *)&he_wgs_dlat,NULL);
    set_subpath();
    set_bucket(dlong,dlat);
    tilenum = gen_index();
    sprintf( tile_filename, "%s/Objects/%s%d.stg",output_base.c_str(), subpath, tilenum );
    sprintf(tile_addline,"OBJECT_SHARED Models/Airport/glide-slope.xml %s %s %6.1f -%s\n",he_wgs_dlong,he_wgs_dlat, (dhe_elev+5.0)*SG_FEET_TO_METER,he_true_hdg);
//    sprintf(tile_addline,"OBJECT_SHARED Models/Airport/glide-slope-1nm-by-3d.ac %s %s %6.1f -%s\n",le_wgs_dlong,le_wgs_dlat, dle_elev*SG_FEET_TO_METER,le_true_hdg);
    iclass=3;  //kludge
    do_tile_update();

  }
  else {
    fprintf(fp_rep," elevations for runway ends undefined\n");
  }
 
}

void read_runway()
{
  read_field(fp_runway,(char *)&high_ident,sizeof(high_ident),tc);	//	HIGH_IDENT
  read_field(fp_runway,(char *)&low_ident,sizeof(low_ident),tc);	//	LOW_IDENT
  read_field(fp_runway,(char *)&high_hdg,sizeof(high_hdg),tc);		//	HIGH_HDG
  read_field(fp_runway,(char *)&low_hdg,sizeof(low_hdg),tc);		//	LOW_HDG
  read_field(fp_runway,(char *)&rwy_length,sizeof(rwy_length),tc);		//	LENGTH
  read_field(fp_runway,(char *)&rwy_width,sizeof(rwy_width),tc);	//	RWY_WIDTH
  read_field(fp_runway,(char *)&surface,sizeof(surface),tc);		//	SURFACE
  read_field(fp_runway,(char *)&pcn,sizeof(pcn),tc);			//	PCN
  read_field(fp_runway,(char *)&he_wgs_lat,sizeof(he_wgs_lat),tc);	//	HE_WGS_LAT
  read_field(fp_runway,(char *)&he_wgs_dlat,sizeof(he_wgs_dlat),tc);	//	HE_WGS_DLAT
  read_field(fp_runway,(char *)&he_wgs_long,sizeof(he_wgs_long),tc);	//	HE_WGS_LONG
  read_field(fp_runway,(char *)&he_wgs_dlong,sizeof(he_wgs_dlong),tc);	//	HE_WGS_DLONG
  read_field(fp_runway,(char *)&he_elev,sizeof(he_elev),tc);		//	HE_ELEV
  read_field(fp_runway,(char *)&he_slope,sizeof(he_slope),tc);		//	HE_SLOPE
  read_field(fp_runway,(char *)&he_tdze,sizeof(he_tdze),tc);		//	HE_TDZE
  read_field(fp_runway,(char *)&he_dt,sizeof(he_dt),tc);		//	HE_DT
  read_field(fp_runway,(char *)&he_dt_elev,sizeof(he_dt_elev),tc);	//	HE_DT_ELEV
  read_field(fp_runway,(char *)&hlgt_sys_1,sizeof(hlgt_sys_1),tc);	//	HLGT_SYS_1
  read_field(fp_runway,(char *)&hlgt_sys_2,sizeof(hlgt_sys_2),tc);	//	HLGT_SYS_2
  read_field(fp_runway,(char *)&hlgt_sys_3,sizeof(hlgt_sys_3),tc);	//	HLGT_SYS_3
  read_field(fp_runway,(char *)&hlgt_sys_4,sizeof(hlgt_sys_4),tc);	//	HLGT_SYS_4
  read_field(fp_runway,(char *)&hlgt_sys_5,sizeof(hlgt_sys_5),tc);	//	HLGT_SYS_5
  read_field(fp_runway,(char *)&hlgt_sys_6,sizeof(hlgt_sys_6),tc);	//	HLGT_SYS_6
  read_field(fp_runway,(char *)&hlgt_sys_7,sizeof(hlgt_sys_7),tc);	//	HLGT_SYS_7
  read_field(fp_runway,(char *)&hlgt_sys_8,sizeof(hlgt_sys_8),tc);	//	HLGT_SYS_8
  read_field(fp_runway,(char *)&le_wgs_lat,sizeof(le_wgs_lat),tc);	//	LE_WGS_LAT
  read_field(fp_runway,(char *)&le_wgs_dlat,sizeof(le_wgs_dlat),tc);	//	LE_WGS_DLAT
  read_field(fp_runway,(char *)&le_wgs_long,sizeof(le_wgs_long),tc);	//	LE_WGS_LONG
  read_field(fp_runway,(char *)&le_wgs_dlong,sizeof(le_wgs_dlong),tc);	//	LE_WGS_DLONG
  read_field(fp_runway,(char *)&le_elev,sizeof(le_elev),tc);		//	LE_ELEV
  read_field(fp_runway,(char *)&le_slope,sizeof(le_slope),tc);		//	LE_SLOPE
  read_field(fp_runway,(char *)&le_tdze,sizeof(le_tdze),tc);		//	LE_TDZE
  read_field(fp_runway,(char *)&le_dt,sizeof(le_dt),tc);		//	LE_DT
  read_field(fp_runway,(char *)&le_dt_elev,sizeof(le_dt_elev),tc);	//	LE_DT_ELEV
  read_field(fp_runway,(char *)&llgt_sys_1,sizeof(llgt_sys_1),tc);	//	LLGT_SYS_1
  read_field(fp_runway,(char *)&llgt_sys_2,sizeof(llgt_sys_2),tc);	//	LLGT_SYS_2
  read_field(fp_runway,(char *)&llgt_sys_3,sizeof(llgt_sys_3),tc);	//	LLGT_SYS_3
  read_field(fp_runway,(char *)&llgt_sys_4,sizeof(llgt_sys_4),tc);	//	LLGT_SYS_4
  read_field(fp_runway,(char *)&llgt_sys_5,sizeof(llgt_sys_5),tc);	//	LLGT_SYS_5
  read_field(fp_runway,(char *)&llgt_sys_6,sizeof(llgt_sys_6),tc);	//	LLGT_SYS_6
  read_field(fp_runway,(char *)&llgt_sys_7,sizeof(llgt_sys_7),tc);	//	LLGT_SYS_7
  read_field(fp_runway,(char *)&llgt_sys_8,sizeof(llgt_sys_8),tc);	//	LLGT_SYS_8
  read_field(fp_runway,(char *)&he_true_hdg,sizeof(he_true_hdg),tc);	//	HE_TRUE_HDG
  read_field(fp_runway,(char *)&le_true_hdg,sizeof(le_true_hdg),tc);	//	LE_TRUE_HDG
  read_field(fp_runway,(char *)&cld_rwy,sizeof(cld_rwy),tc);		//	CLD_RWY
  read_field(fp_runway,(char *)&heland_dis,sizeof(heland_dis),tc);	//	HELAND_DIS
  read_field(fp_runway,(char *)&he_takeoff,sizeof(he_takeoff),tc);	//	HE_TAKEOFF
  read_field(fp_runway,(char *)&leland_dis,sizeof(leland_dis),tc);	//	LELAND_DIS
  read_field(fp_runway,(char *)&le_takeoff,sizeof(le_takeoff),tc);	//	LE_TAKEOFF
  read_field(fp_runway,(char *)&rcycle_date,sizeof(rcycle_date),tc2);	//	CYCLE_DATE

  fprintf(fp_rep, " runway     :    %s - %s\n",high_ident, low_ident);
  fprintf(fp_rep, " true h     : %s - %s\n",he_true_hdg, le_true_hdg);
  fprintf(fp_rep, " lat        : %s - %s\n", he_wgs_dlat, le_wgs_dlat); 
  fprintf(fp_rep, " long       : %s - %s\n", he_wgs_dlong, le_wgs_dlong);
  fprintf(fp_rep, " elev       : %s - %s\n", he_elev, le_elev);
  fprintf(fp_rep, " disp-thr   : %s - %s\n", he_dt, le_dt);
  fprintf(fp_rep, " tdze       : %s - %s\n", he_tdze, le_tdze);
  fprintf(fp_rep, " elev d.t   : %s - %s\n", he_dt_elev, le_dt_elev);
  fprintf(fp_rep, " length     :    %s\n",rwy_length);
  fprintf(fp_rep, " width      :    %s\n",rwy_width);
  fprintf(fp_rep, " type       :    ");

  strncpy((char *)&_rwy_ident[num_rwy],(const char *) high_ident,4);
  strncpy((char *)&_rwy_ident[num_rwy+1],(const char *)low_ident,4);

  strncpy((char *)&_rwy_elev[num_rwy],(const char *)he_elev,8);
  strncpy((char *)&_rwy_elev[num_rwy+1],(const char *)le_elev,8);

  strncpy((char *)&_rwy_width[num_rwy],(const char *)rwy_width,8);
  strncpy((char *)&_rwy_width[num_rwy+1],(const char *)rwy_width,8);

  strncpy((char *)&_rwy_lat[num_rwy],(const char *)he_wgs_dlat,12);
  strncpy((char *)&_rwy_lat[num_rwy+1],(const char *)le_wgs_dlat,12);

  strncpy((char *)&_rwy_long[num_rwy],(const char *)he_wgs_dlong,12);
  strncpy((char *)&_rwy_long[num_rwy+1],(const char *)le_wgs_dlong,12);

  strncpy((char *)&_rwy_t_heading[num_rwy],(const char *)he_true_hdg,12);
  strncpy((char *)&_rwy_t_heading[num_rwy+1],(const char *)le_true_hdg,12);


  num_rwy+=2;

//[%s]\n",type);

/**
DEFINITION: THE VISIBLE MATERIAL OR COMPOSITION OF THE MAJOR PORTION OF THE LANDING PORTION OF RUNWAY.

SURFACE	DEFINITION
CODE	HARD(PERMANENT)SURFACE
ASP	ASPHALT, ASPHALTIC CONCRETE, 
TAR	MACADAM, OR BITUMEN BOUND MACADAM 	(INCLUDING ANY OF THESE SURFACE TYPES WITH CONCRETE ENDS).
BRI	BRICK, LAID OR MORTARED.
CON	CONCRETE.
COP	COMPOSITE, 50 PERCENT OR MORE OF THE RUNWAY LENGTH IS  PERMANENT.
PEM	PART CONCRETE, PART ASPHALT, OR PART BITUMEN-BOUND MACADAM.
PER	PERMANENT, SURFACE TYPE UNKNOWN.SOFT(NON-PERMANENT)SURFACE
BIT	BITUMINOUS, TAR OR ASPHALT MIXED IN PLACE, OILED.
CLA	CLAY
COM	COMPOSITE, LESS THAN 50 PERCENT OF THE RUNWAY LENGTH IS PERMANENT.
COR	CORAL.
GRE	GRADED OR ROLLED EARTH, GRASS ON GRADED EARTH.
GRS	GRASS OR EARTH NOT GRADED OR ROLLED.
GVL	GRAVEL.
ICE	ICE.
LAT	LATERITE.
MAC	MACADAM - CRUSHED ROCK WATER BOUND.
MEM	MEMBRANE - PLASTIC OR OTHER COATED FIBER MATERIAL.
MIX	MIX IN PLACE USING NONBITUMIOUS BINDERS SUCH AS PORTLAND CEMENT
PSP	PIECED STEEL PLANKING.
SAN	SAND, GRADED, ROLLED OR OILED.
SNO	SNOW.
U	SURFACE UNKNOWN.
**/
  if (strncmp(type,"ASP",3)) fprintf(fp_rep,"Asphalt\n");
  else
  if (strncmp(type,"TAR",3)) fprintf(fp_rep,"Macadam\n");
  else
  if (strncmp(type,"BRI",3)) fprintf(fp_rep,"Brick\n");
  else
  if (strncmp(type,"CON",3)) fprintf(fp_rep,"Concrete\n");
  else
  if (strncmp(type,"COP",3)) fprintf(fp_rep,"Composite\n");
  else
  if (strncmp(type,"PEM",3)) fprintf(fp_rep,"Parts concrete, asphalt, or bitumen\n");
  else
  if (strncmp(type,"PER",3)) fprintf(fp_rep,"Permanent of unknown soft type\n");
  else
  if (strncmp(type,"BIT",3)) fprintf(fp_rep,"Bituminous\n");
  else
  if (strncmp(type,"CLA",3)) fprintf(fp_rep,"Clay\n");
  else
  if (strncmp(type,"COM",3)) fprintf(fp_rep,"Composit\n");
  else
  if (strncmp(type,"COR",3)) fprintf(fp_rep,"Coral\n");
  else
  if (strncmp(type,"GRE",3)) fprintf(fp_rep,"Graded\n");
  else
  if (strncmp(type,"GRS",3)) fprintf(fp_rep,"Grass\n");
  else
  if (strncmp(type,"GVL",3)) fprintf(fp_rep,"Gravel\n");
  else
  if (strncmp(type,"ICE",3)) fprintf(fp_rep,"Ice\n");
  else
  if (strncmp(type,"LAT",3)) fprintf(fp_rep,"Laterite\n");
  else
  if (strncmp(type,"MAC",3)) fprintf(fp_rep,"Macadam\n");
  else
  if (strncmp(type,"MEM",3)) fprintf(fp_rep,"Membrane\n");
  else
  if (strncmp(type,"MIX",3)) fprintf(fp_rep,"Mix such as portland cement\n");
  else
  if (strncmp(type,"PSP",3)) fprintf(fp_rep,"Pieced steel planking\n");
  else
  if (strncmp(type,"SAN",3)) fprintf(fp_rep,"Sanded, graded, rolled or oiled\n");
  else
  if (strncmp(type,"SNO",3)) fprintf(fp_rep,"Snow\n");
  else
  if (strncmp(type,"U",1)) fprintf(fp_rep,"Uknown\n");

  if (strncmp(cld_rwy,"C",1) !=0) {
    do_runway();
  }
  else {
    fprintf(fp_rep, "runway is closed, skipping \n");
  }
}

char fsegment[13];
char frunway[8];
char fairport[8];

bool putbackflag=false;
void find_segments ( char *f, int fs)
{
  // char tc='\t';
  // char tc2='\n';
  bool found=false;
  int i=0; 
  // int n;
  char ch;
  while (!found) {
    if (!putbackflag) {
      fsegment[0]=i=0; 
      while (peek(fp_segment) != '\t') { 
        fsegment[i]=fgetc(fp_segment); 
        i++;
        if (i<fs) fsegment[i] =0;
      }
      ch = fgetc(fp_segment);
    }
    else {
      putbackflag=false;
    }
    if (strncmp(fsegment,f,fs) != 0) {
      skip_record(fp_segment);
    }
    else {
      found=true;
      read_segment();
    }
  }
  while (found) {

   fsegment[0]=i=0; 
    while (peek(fp_segment)!='\t') {
      fsegment[i]=fgetc(fp_segment); 
      i++;
      if (i<fs) fsegment[i] =0;
    }
    ch=fgetc(fp_segment);
    if (strncmp(fsegment,f,fs) == 0) {
      read_segment();
      found=true;

    }
    else {
      found=false;
      putbackflag=true;
    }
  }
  if (head_pt!=NULL) {
 //   printf("call add_pts_to_master\n");
    add_pts_to_master();
  }
  fprintf(fp_rep,"---end of segments---\n");
  if (head_pt_m!=NULL) {
 //   list_line(head_pt_m);
    create_tile_list();
    erase_master_list();
  }
}

void find_suas_segments ( char *f, int fs)
{
  // char tc='\t';
  // char tc2='\n';
  bool found=false;
  int i=0; 
  // int n;
  char ch;
  while (!found) {
    if (!putbackflag) {
      fsegment[0]=i=0; 
      while (peek(fp_segment)!= '\t') { 
        fsegment[i]=fgetc(fp_segment); 
        i++;
        if (i<fs) fsegment[i] =0;
      }
      ch=fgetc(fp_segment);
    }
    else {
      putbackflag=false;
    }
    if (strncmp(fsegment,f,fs) != 0) {
      skip_record(fp_segment);
    }
    else {
       found=true;
      read_suas_segment();
    }
  }
  while (found) {
   fsegment[0]=i=0; 
    while (peek(fp_segment) !='\t') { 
      fsegment[i]=fgetc(fp_segment); 
      i++;
      if (i<fs) fsegment[i] =0;
    }
    ch=fgetc(fp_segment);
    if (strncmp(fsegment,f,fs) == 0) {
      read_suas_segment();
      found=true;
    }
    else {
      found=false;
      putbackflag=true;
    }
  }
  if (head_pt!=NULL) add_pts_to_master();
  fprintf(fp_rep,"---end of segments---\n");
  if (head_pt_m!=NULL)  {
  //  list_line(head_pt_m);
    create_tile_list();
    erase_master_list();
  }
}

void find_runways ( char *f, int fs)
{
  // char tc='\t';
  // char tc2='\n';
  bool found=false;
  int i=0; 
  // int n;
  char ch;
  num_rwy=0;
  while (!found) {
    if (!putbackflag) {
      frunway[0]=i=0; 
      while (peek(fp_runway)!= '\t') { 
        frunway[i]=fgetc(fp_runway); 
        i++;
        if (i<fs) frunway[i] =0;
      }
      ch=fgetc(fp_runway);
    }
    else {
      putbackflag=false;
    }
    if (strncmp(frunway,f,fs) != 0) {
      skip_record(fp_runway);
    }
    else {
      found=true;
      read_runway();
    }
  }
  while (found) {
   frunway[0]=i=0; 
    while (peek(fp_runway) !='\t') { 
      frunway[i]=fgetc(fp_runway); 
      i++;
      if (i<fs) frunway[i] =0;
    }
    ch=fgetc(fp_runway);
    if (strncmp(frunway,f,fs) == 0) {
      read_runway();
      found=true;
    }
    else {
      found=false;
      putbackflag=true;
    }
  }
}

void find_parent( char * f, int fs)
{
  char fparent[13];
  bool found=false;
  int i=0; 
  char ch;
  while (!found) {
    fparent[0]=i=0; 
    while (peek(fp_parent)!='\t') {   
      fparent[i]=fgetc(fp_parent); 
      i++;
      if (i<fs) fparent[i] =0;
    }
    ch=fgetc(fp_parent);
    if (strncmp(fparent,f,fs) != 0) {
      skip_record(fp_parent);
    }
    else {
      found=true;
      read_field(fp_parent,(char *)&ptype,sizeof(ptype),tc);//
      read_field(fp_parent,(char *)&pname,sizeof(pname),tc);//
      read_field(fp_parent,(char *)&picao,sizeof(picao),tc);
      read_field(fp_parent,(char *)&pcon_auth,sizeof(pcon_auth),tc);
      read_field(fp_parent,(char *)&ploc_hdatum,sizeof(ploc_hdatum),tc);
      read_field(fp_parent,(char *)&pwgs_datum,sizeof(pwgs_datum),tc);
      read_field(fp_parent,(char *)&pcomm_name,sizeof(pcomm_name),tc);
      read_field(fp_parent,(char *)&pcomm_freq1,sizeof(pcomm_freq1),tc);
      read_field(fp_parent,(char *)&pcomm_freq2,sizeof(pcomm_freq2),tc);
      read_field(fp_parent,(char *)&pclass,sizeof(pclass),tc);//
      read_field(fp_parent,(char *)&pclass_exc,sizeof(pclass_exc),tc);
      read_field(fp_parent,(char *)&pclass_ex_rmk,sizeof(pclass_ex_rmk),tc);
      read_field(fp_parent,(char *)&plevel,sizeof(plevel),tc);
      read_field(fp_parent,(char *)&pupper_alt,sizeof(pupper_alt),tc);
      read_field(fp_parent,(char *)&plower_alt,sizeof(plower_alt),tc);
      read_field(fp_parent,(char *)&prnp,sizeof(prnp),tc);
      read_field(fp_parent,(char *)&pcycle_date,sizeof(pcycle_date),tc2);

        
  //    fprintf( fp_icao, "%s\t[%s] [%s] [%s] [%s]\n",picao,pname,pcon_auth,pcomm_freq1,pcomm_freq2);

      fprintf(fp_rep, "     Airspace type: %s", ptype);

      if (strncmp(ptype,"01",2)==0) fprintf(fp_rep, " - ADVISORY AREA (ADA) OR (UDA)\n");
      if (strncmp(ptype,"02",2)==0) fprintf(fp_rep, " - AIR DEFENSE IDENTIFICATION ZONE (ADIZ)\n");
      if (strncmp(ptype,"03",2)==0) fprintf(fp_rep, " - AIR ROUTE TRAFFIC CONTROL CENTER (ARTCC)\n");
      if (strncmp(ptype,"04",2)==0) fprintf(fp_rep, " - AREA CONTROL CENTER (ACC)\n");
      if (strncmp(ptype,"05",2)==0) fprintf(fp_rep, " - BUFFER ZONE (BZ)\n");
      if (strncmp(ptype,"06",2)==0) fprintf(fp_rep, " - CONTROL AREA (CTA) (UTA) SPECIAL RULES AREA (SRA, U.K. ONLY)\n");
      if (strncmp(ptype,"07",2)==0) fprintf(fp_rep, " - CONTROL ZONE (CTLZ)  SPECIAL RULES ZONE (SRZ, U.K.  ONLY) MILITARY AERODROME TRAFFIC ZONE (MATZ, U.K. ONLY)\n");
      if (strncmp(ptype,"08",2)==0) fprintf(fp_rep, " - FLIGHT INFORMATION REGION (FIR)\n");
      if (strncmp(ptype,"09",2)==0) fprintf(fp_rep, " - OCEAN CONTROL AREA (OCA)\n");
      if (strncmp(ptype,"10",2)==0) fprintf(fp_rep, " - RADAR AREA\n");
      if (strncmp(ptype,"11",2)==0) fprintf(fp_rep, " - TERMINAL CONTROL AREA (TCA) OR (MTCA)\n");
      if (strncmp(ptype,"12",2)==0) fprintf(fp_rep, " - UPPER FLIGHT INFORMATION REGION (UIR)\n");

      fprintf(fp_rep, "     Name: %s\n", pname);
      fprintf(fp_rep, "     ICAO ID: %s", picao);
      fprintf(fp_rep, "     OFFICE CONTROLLING AIRSPACE: %s", pcon_auth);        
      fprintf(fp_rep, "     CALL SIGN : %s", pcomm_name);
      fprintf(fp_rep, "     FREQ.: %s %s", pcomm_freq1, pcomm_freq2);
      fprintf(fp_rep, "     Class: %s\n", pclass);

      iclass = -1;
      if (strncmp(pclass,"A",1)==0) iclass=0;
      if (strncmp(pclass,"B",1)==0) iclass=1;
      if (strncmp(pclass,"C",1)==0) iclass=2;
      if (strncmp(pclass,"D",1)==0) iclass=3;
      if (strncmp(pclass,"E",1)==0) iclass=4;
      if (iclass != -1) {
        cum_class[iclass]++;
      }  
      else {
        fprintf(fp_rep, "WARNING unknown AIRSPACE class [%s], artificially setting class to Class A\n",pclass);
        iclass=0;
      }
      if (strncmp(pclass_exc,"Y",1)==0) {
        fprintf(fp_rep, "     Class exception flag: %s\n",pclass_exc);  
        fprintf(fp_rep, "     Class exception remarks: %s\n",pclass_ex_rmk);
      } 
      fprintf(fp_rep, "     Level: ");
      if (strncmp(plevel,"B",1)==0) fprintf(fp_rep, "B - HIGH AND LOW LEVEL\n");
      if (strncmp(plevel,"H",1)==0) fprintf(fp_rep, "H - HIGH LEVEL\n");
      if (strncmp(plevel,"L",1)==0) fprintf(fp_rep, "L - LOW LEVEL\n");

      fprintf(fp_rep, "     Upper Altitude limit: %s\n", pupper_alt);
      fprintf(fp_rep, "     Lower ALtitude limit: %s\n", plower_alt);

      if (strncmp(plower_alt,"SURFACE",7)==0) cum_class_surface[iclass]++;
      if (iclass == 3) {
        if (strncmp(plower_alt,"SURFACE",7)!=0) fprintf(fp_rep,"CLASS D WITH NON SURFACE FLOOR?\n");
      }
        
      if (prnp[0]!=0) fprintf(fp_rep, "     Required Navigation Performance: [%s] NAUTICAL MILES\n", prnp);
      do_decode_altitudes();
      fprintf(fp_rep,"Segment information: \n");
     // init_boundary_ave();
      

      set_safe_bdry();
      find_segments((char *) &bdry_ident,sizeof(bdry_ident));
     // printf("airspace is centered at lat [%f] long  [%f]\n",ave_boundary_lat(), ave_boundary_long());

    }
  } 
}


void reset_moa_files()
{
  int ix;
  int nx;
  bool done=false;
  ix=0;

  while (!done) {
    if (pname[ix]!=',') ix++;
    else {
      nx=ix;//-1;
      done=true;
    }
    if (ix>=sizeof(pname)) done=true;
  }
  if (nx!=0) {
    ix=0;
  //  printf("use pname [%s] up to the %d'th character\n",pname,nx+1); 
    for (ix=0; ix<nx; ix++) {
      bdry_ident_safe[ix] = pname[ix];
      if (bdry_ident_safe[ix]== ' ') bdry_ident_safe[ix] = '-'; 
      if (bdry_ident_safe[ix]== '(') bdry_ident_safe[ix] = '-'; 
      if (bdry_ident_safe[ix]== ')') bdry_ident_safe[ix] = '-'; 
      if (bdry_ident_safe[ix]== ',') bdry_ident_safe[ix] = '-'; 
      bdry_ident_safe[ix+1]='\0';
    }
    fprintf(fp_rep,"bdry_ident_safe reset to [%s]\n",bdry_ident_safe);
    sprintf(pname,"%s %s %s",pcomm_name,pcomm_freq1,pcomm_freq2);
    fprintf(fp_rep,"pname reset to [%s]\n",pname);
  }
  else return;  // use the faa internal id for the moa
}

void find_suas_parent( char * f, int fs)
{
  char fparent[13];
  bool found=false;
  int i=0; 
  char ch;
  while (!found) {
    fparent[0]=i=0; 
    while (peek(fp_parent) !='\t') { 
      fparent[i]= fgetc(fp_parent); 
      i++;
      if (i<fs) fparent[i] =0;
    }
    ch=fgetc(fp_parent);
    if (strncmp(fparent,f,fs) != 0) {
      skip_record(fp_parent);
    }
    else {
      found=true;

      read_field(fp_parent,(char *)&suas_sector,sizeof(suas_sector),tc);//
      read_field(fp_parent,(char *)&ptype,sizeof(ptype),tc);//
      read_field(fp_parent,(char *)&pname,sizeof(pname),tc);//
      read_field(fp_parent,(char *)&picao,sizeof(picao),tc);
      read_field(fp_parent,(char *)&suas_con_agcy,sizeof(suas_con_agcy),tc);
      read_field(fp_parent,(char *)&ploc_hdatum,sizeof(ploc_hdatum),tc);
      read_field(fp_parent,(char *)&pwgs_datum,sizeof(pwgs_datum),tc);
      read_field(fp_parent,(char *)&pcomm_name,sizeof(pcomm_name),tc);
      read_field(fp_parent,(char *)&pcomm_freq1,sizeof(pcomm_freq1),tc);
      read_field(fp_parent,(char *)&pcomm_freq2,sizeof(pcomm_freq2),tc);
      read_field(fp_parent,(char *)&plevel,sizeof(plevel),tc);
      read_field(fp_parent,(char *)&pupper_alt,sizeof(pupper_alt),tc);
      read_field(fp_parent,(char *)&plower_alt,sizeof(plower_alt),tc);
      read_field(fp_parent,(char *)&suas_eff_times,sizeof(suas_eff_times),tc);
      read_field(fp_parent,(char *)&suas_wx,sizeof(suas_wx),tc);
      read_field(fp_parent,(char *)&pcycle_date,sizeof(pcycle_date),tc);
      read_field(fp_parent,(char *)&suas_eff_date,sizeof(suas_eff_date),tc2);

      fprintf(fp_rep, "     (Special Use) Airspace type: %s", ptype);

      if (strncmp(ptype,"A",1)==0) fprintf(fp_rep, "		A - ALERT\n");
      if (strncmp(ptype,"D",1)==0) fprintf(fp_rep, "		D - DANGER\n");
      if (strncmp(ptype,"M",1)==0) fprintf(fp_rep, "		M - MILITARY OPERATIONS AREA\n");
      if (strncmp(ptype,"P",1)==0) fprintf(fp_rep, "		P - PROHIBITED\n");
      if (strncmp(ptype,"R",1)==0) fprintf(fp_rep, "		R - RESTRICTED\n");
      if (strncmp(ptype,"T",1)==0) fprintf(fp_rep, "		T - TEMPORARY RESERVED AIRSPACE\n");
      if (strncmp(ptype,"W",1)==0) fprintf(fp_rep, "		W - WARNING\n"); 

      iclass = -1;           
      switch (ptype[0]) {
        case 'A':  { 
          iclass = CLASS_SA;
          break;
        }
        case 'D': {
          iclass = CLASS_SD;
          break;
        }
        case 'M': {
         iclass = CLASS_SM;
          break;
        }
        case 'P':  {
          iclass = CLASS_SP;
          break;
        } 
        case 'R':  {
          iclass = CLASS_SR;
          break;
        }
        case 'T': {
          iclass = CLASS_ST;
          break;
        }  
        case 'W':  {
          iclass = CLASS_SW;
          break;
        }  
      }
    //  cout << "iclass code is " << iclass << "\n";
      fprintf(fp_rep, "     Name: %s", pname);
      fprintf(fp_rep, "     ICAO ID: %s",picao);
      fprintf(fp_rep, "     CONTROLLING AGENCY: %s\n",suas_con_agcy);        
      fprintf(fp_rep, "     CALL SIGN : %s",pcomm_name);
      fprintf(fp_rep, "     FREQ.: %s %s\n",pcomm_freq1, pcomm_freq2);
//      cout << "     Class: " << pclass << '\n';    
      if (iclass != -1) {
        cum_class[iclass]++;
      }  
      else {
        fprintf(fp_rep, "WARNING WHAT IS SPECIAL AIRSPACE [%s]?, arbitrarily setting to Warning type special use airspace\n",ptype);
        iclass= CLASS_SW;
      }  
      fprintf(fp_rep, "     Level: ");
      if (strncmp(plevel,"B",1)==0) fprintf(fp_rep, "B - HIGH AND LOW LEVEL\n");
      if (strncmp(plevel,"H",1)==0) fprintf(fp_rep, "H - HIGH LEVEL\n");
      if (strncmp(plevel,"L",1)==0) fprintf(fp_rep, "L - LOW LEVEL\n");

      fprintf(fp_rep, "     Upper Altitude limit: %s\n", pupper_alt);
      fprintf(fp_rep, "     Lower ALtitude limit: %s\n", plower_alt);
      fprintf(fp_rep, "     Effective Times:      %s\n", suas_eff_times);
      fprintf(fp_rep, "     Weather:              %s\n", suas_wx);
      fprintf(fp_rep, "     Effective Date:       %s\n", suas_eff_date);

      do_decode_altitudes();

      fprintf(fp_rep,"Segment information:\n");
      init_boundary_ave();

//      printf("Segment information: altitude_low %f altitude_high %f from strings [%s] [%s]\n",altitude_low, altitude_high,plower_alt, pupper_alt);
      set_safe_bdry();
      find_suas_segments((char *) &bdry_ident,sizeof(bdry_ident));
      fprintf(fp_rep,"special use airspace is centered at lat [%f] long  [%f]\n",ave_boundary_lat(), ave_boundary_long());
        if ( (!high_agl_flag) && (!high_altitude_flag) ) {
          if (iclass==CLASS_SM) {
            reset_moa_files();
          }
          write_sign_files_special( ave_boundary_lat(),ave_boundary_long(), altitude_high);

        }
    }
  } 
}

void read_boundary_country()
{
  read_field(fp_country,(char *)&bdry_ident,sizeof(bdry_ident),tc);
  read_field(fp_country,(char *)&seg_nbr,sizeof(seg_nbr),tc);
  read_field(fp_country,(char *)&ctry_1,sizeof(ctry_1),tc);
  read_field(fp_country,(char *)&ctry_2,sizeof(ctry_2),tc);
  read_field(fp_country,(char *)&ctry_3,sizeof(ctry_3),tc);
  read_field(fp_country,(char *)&ctry_4,sizeof(ctry_4),tc);
  read_field(fp_country,(char *)&ctry_5,sizeof(ctry_5),tc);
  read_field(fp_country,(char *)&cycle_date,sizeof(cycle_date),'\n');
}

void read_suas_boundary_country()
{
  read_field(fp_country,(char *)&bdry_ident,sizeof(bdry_ident),tc);
  read_field(fp_country,(char *)&suas_sector,sizeof(suas_sector),tc);
  read_field(fp_country,(char *)&suas_icao,sizeof(suas_icao),tc);
  read_field(fp_country,(char *)&ctry_1,sizeof(ctry_1),tc);
  read_field(fp_country,(char *)&ctry_2,sizeof(ctry_2),tc);
  read_field(fp_country,(char *)&ctry_3,sizeof(ctry_3),tc);
  read_field(fp_country,(char *)&ctry_4,sizeof(ctry_4),tc);
  read_field(fp_country,(char *)&cycle_date,sizeof(cycle_date),'\n');
}

void read_airport()
{
  read_field(fp_airport,(char *)&arpt_ident,sizeof(arpt_ident),tc);
  read_field(fp_airport,(char *)&name,sizeof(name),tc);
  read_field(fp_airport,(char *)&state_prov,sizeof(state_prov),tc);
  read_field(fp_airport,(char *)&icao,sizeof(icao),tc);				//note if icao is KZ the KXXX identifier is in faa_host_id....
  read_field(fp_airport,(char *)&faa_host_id,sizeof(faa_host_id),tc);
  read_field(fp_airport,(char *)&loc_hdatum,sizeof(loc_hdatum),tc);
  read_field(fp_airport,(char *)&wgs_datum,sizeof(wgs_datum),tc);
  read_field(fp_airport,(char *)&wgs_lat,sizeof(wgs_lat),tc);
  read_field(fp_airport,(char *)&wgs_dlat,sizeof(wgs_dlat),tc);
  read_field(fp_airport,(char *)&wgs_long,sizeof(wgs_long),tc);
  read_field(fp_airport,(char *)&wgs_dlong,sizeof(wgs_dlong),tc);
  read_field(fp_airport,(char *)&elev,sizeof(elev),tc);
  read_field(fp_airport,(char *)&type,sizeof(type),tc);
  read_field(fp_airport,(char *)&mag_var,sizeof(mag_var),tc);
  read_field(fp_airport,(char *)&wac,sizeof(wac),tc);
  read_field(fp_airport,(char *)&beacon,sizeof(beacon),tc);
  read_field(fp_airport,(char *)&second_arpg,sizeof(second_arpg),tc);
  read_field(fp_airport,(char *)&opr_agy,sizeof(opr_agy),tc);
  read_field(fp_airport,(char *)&sec_name,sizeof(sec_name),tc);
  read_field(fp_airport,(char *)&sec_icao,sizeof(sec_icao),tc);
  read_field(fp_airport,(char *)&sec_faa,sizeof(sec_faa),tc);
  read_field(fp_airport,(char *)&sec_opr_agy,sizeof(sec_opr_agy),tc);
  read_field(fp_airport,(char *)&acycle_date,sizeof(acycle_date),tc2);
//  read_field(fp_airport,(char *)&terrain,sizeof(terrain),tc);  //old temporary add on field no longer present as of DAFIFT edition 7 downloaded 2005-12-13
//  read_field(fp_airport,(char *)&hydro,sizeof(hydro),'\n');

}
void read_airport_have_id()
{
//  read_field(fp_airport,(char *)&arpt_ident,sizeof(arpt_ident),tc);
  read_field(fp_airport,(char *)&name,sizeof(name),tc);
  read_field(fp_airport,(char *)&state_prov,sizeof(state_prov),tc);
  read_field(fp_airport,(char *)&icao,sizeof(icao),tc);				//note if icao is KZ the KXXX identifier is in faa_host_id....
  read_field(fp_airport,(char *)&faa_host_id,sizeof(faa_host_id),tc);
  read_field(fp_airport,(char *)&loc_hdatum,sizeof(loc_hdatum),tc);
  read_field(fp_airport,(char *)&wgs_datum,sizeof(wgs_datum),tc);
  read_field(fp_airport,(char *)&wgs_lat,sizeof(wgs_lat),tc);
  read_field(fp_airport,(char *)&wgs_dlat,sizeof(wgs_dlat),tc);
  read_field(fp_airport,(char *)&wgs_long,sizeof(wgs_long),tc);
  read_field(fp_airport,(char *)&wgs_dlong,sizeof(wgs_dlong),tc);
  read_field(fp_airport,(char *)&elev,sizeof(elev),tc);
  read_field(fp_airport,(char *)&type,sizeof(type),tc);
  read_field(fp_airport,(char *)&mag_var,sizeof(mag_var),tc);
  read_field(fp_airport,(char *)&wac,sizeof(wac),tc);
  read_field(fp_airport,(char *)&beacon,sizeof(beacon),tc);
  read_field(fp_airport,(char *)&second_arpg,sizeof(second_arpg),tc);
  read_field(fp_airport,(char *)&opr_agy,sizeof(opr_agy),tc);
  read_field(fp_airport,(char *)&sec_name,sizeof(sec_name),tc);
  read_field(fp_airport,(char *)&sec_icao,sizeof(sec_icao),tc);
  read_field(fp_airport,(char *)&sec_faa,sizeof(sec_faa),tc);
  read_field(fp_airport,(char *)&sec_opr_agy,sizeof(sec_opr_agy),tc);
  read_field(fp_airport,(char *)&acycle_date,sizeof(acycle_date),tc2);
//  read_field(fp_airport,(char *)&terrain,sizeof(terrain),tc);
//  read_field(fp_airport,(char *)&hydro,sizeof(hydro),'\n');

}

int nwp=0;
int nwpskip=0;

int type_unnamed=0;
int type_ndb=0;
int type_named=0;
int type_off=0;
int type_vfr=0; 

void print_wpt_type()
{
  fprintf(fp_rep," type(s) [");
  if (strchr((const char *) &wpt_type,'I')!=NULL) { fprintf(fp_rep," [UNNAMED] "); type_unnamed++; }
  if (strchr((const char *) &wpt_type,'N')!=NULL) { fprintf(fp_rep," [NDB] ");     type_ndb++; }
  if (strchr((const char *) &wpt_type,'R')!=NULL) { fprintf(fp_rep," [NAMED] ");   type_named++; }
  if (strchr((const char *) &wpt_type,'F')!=NULL) { fprintf(fp_rep," [OFF ROUTE] "); type_off++; }
  if (strchr((const char *) &wpt_type,'V')!=NULL) { fprintf(fp_rep," [VFR REPORTING POINT] "); type_vfr++; }
  fprintf(fp_rep,"] ");
/**
	I - UNNAMED, CHARTED or Computer nav Fix
	N - NDB
	R - NAMED
	F - OFF ROUTE
	V - VFR REPORTING POINT

**/
}


int high_usage=0;
int low_usage=0;
int both_usage=0;
int rnav_usage=0;
int term_usage=0;


void print_usage_cd()
{
/**
	H - HIGH LEVEL
	L - LOW LEVEL
	B - BOTH
	R - RNAV
	T - TERMINAL WAYPOINT
**/
  fprintf(fp_rep," usage [");
  if (strchr((const char *) &usage_cd,'H')!=NULL) { fprintf(fp_rep," HIGH"); high_usage++; }
  if (strchr((const char *) &usage_cd,'L')!=NULL) { fprintf(fp_rep," LOW"); low_usage++;    }
  if (strchr((const char *) &usage_cd,'B')!=NULL) { fprintf(fp_rep," BOTH"); both_usage++;  }
  if (strchr((const char *) &usage_cd,'R')!=NULL) { fprintf(fp_rep," RNAV");  rnav_usage++; }
  if (strchr((const char *) &usage_cd,'T')!=NULL) { fprintf(fp_rep," TERM WP"); term_usage++; }
  fprintf(fp_rep,"] ");

}

void print_nav_type()
{
/**
	1 - VOR
	2 - VORTAC
	3 - TACAN
	4 - VOR-DME
	5 - NDB
	7 - NDB-DME
	9 - DME (EXCLUDING ILS-DME)
**/
  fprintf(fp_rep," nav_type [");
  if (strchr((const char *) &nav_type,'1')!=NULL) fprintf(fp_rep," VOR");
  if (strchr((const char *) &nav_type,'2')!=NULL) fprintf(fp_rep," VORTAC");
  if (strchr((const char *) &nav_type,'3')!=NULL) fprintf(fp_rep," TACAN");
  if (strchr((const char *) &nav_type,'4')!=NULL) fprintf(fp_rep," VOR-DME");
  if (strchr((const char *) &nav_type,'5')!=NULL) fprintf(fp_rep," NDB");
  if (strchr((const char *) &nav_type,'7')!=NULL) fprintf(fp_rep," NDB-DME");
  if (strchr((const char *) &nav_type,'9')!=NULL) fprintf(fp_rep," DME");
  fprintf(fp_rep,"] ");
}

void read_waypoint()
{

  read_field(fp_waypoint,(char *)&wpt_ident,sizeof(wpt_ident),tc);
  read_field(fp_waypoint,(char *)&ctry,sizeof(ctry),tc);
  read_field(fp_waypoint,(char *)&state_prov,sizeof(state_prov),tc);
  read_field(fp_waypoint,(char *)&wpt_nav_flag,sizeof(wpt_nav_flag),tc);
  read_field(fp_waypoint,(char *)&wpt_type,sizeof(wpt_type),tc); //modified field name from type to avoid size conflict...
  read_field(fp_waypoint,(char *)&desc,sizeof(desc),tc);
  read_field(fp_waypoint,(char *)&icao,sizeof(icao),tc);
  read_field(fp_waypoint,(char *)&usage_cd,sizeof(usage_cd),tc);
  read_field(fp_waypoint,(char *)&bearing,sizeof(bearing),tc);
  read_field(fp_waypoint,(char *)&wpt_distance,sizeof(wpt_distance),tc);
  read_field(fp_waypoint,(char *)&wac,sizeof(wac),tc);
  read_field(fp_waypoint,(char *)&loc_hdatum,sizeof(loc_hdatum),tc);
  read_field(fp_waypoint,(char *)&wgs_datum,sizeof(wgs_datum),tc);
  read_field(fp_waypoint,(char *)&wgs_lat,sizeof(wgs_lat),tc);
  read_field(fp_waypoint,(char *)&wgs_dlat,sizeof(wgs_dlat),tc);
  read_field(fp_waypoint,(char *)&wgs_long,sizeof(wgs_long),tc);
  read_field(fp_waypoint,(char *)&wgs_dlong,sizeof(wgs_dlong),tc);
  read_field(fp_waypoint,(char *)&mag_var,sizeof(mag_var),tc);	
  read_field(fp_waypoint,(char *)&nav_ident,sizeof(nav_ident),tc);	
  read_field(fp_waypoint,(char *)&nav_type,sizeof(nav_type),tc);	
  read_field(fp_waypoint,(char *)&nav_ctry,sizeof(nav_ctry),tc);	
  read_field(fp_waypoint,(char *)&nav_key_cd,sizeof(nav_key_cd),tc);
  read_field(fp_waypoint,(char *)&cycle_date,sizeof(cycle_date),tc2);
//  printf("cycle_date=[%s]\n",cycle_date);

//  

  if (strncmp(ctry,country,2)==0) {
    nwp++;
    if (strncmp(desc,"(RW",3)==0) nwpskip++;
    else {
      if (strncmp(wpt_nav_flag,"Y",1)==0) { 
//        printf("wpt_ident[%s] collocated with [%s] ",wpt_ident,nav_ident);
        nwpskip++;
      }
      else {
        if ( (strchr((const char *) &usage_cd,'L')!=NULL)  || (strchr((const char *) &usage_cd,'B')!=NULL) ) {
          if (strchr((const char *) &wpt_type,'R')!=NULL) { 
            if (strchr((const char *) &wpt_type,'F')==NULL) { 
              fprintf(fp_rep,"[%13s] - [%13s] ",wgs_dlat, wgs_dlong);
              fprintf(fp_rep,"wpt_ident[%s] ",wpt_ident);
              print_wpt_type();
              fprintf(fp_rep,"desc[%s] icao[%s] ",   desc, icao); 
              print_usage_cd(); 
              fprintf(fp_rep," bearing[%s] wpt_distance[%s] ",bearing, wpt_distance);
              fprintf(fp_rep," nav_ident[%s] ",nav_ident);
              print_nav_type();     
              fprintf(fp_rep," nav_key_cd[%s]\n",nav_key_cd);
              write_sign_files_waypoint( strtod((const char *) wgs_dlat,NULL), strtod((const char *) wgs_dlong,NULL), 14000.0);
            } 
            else {
              nwpskip++;
              fprintf(fp_rep,"waypoint [%s] of type [%s] skipped\n",icao,wpt_type);  
            }
          } 
          else {
            nwpskip++;
            fprintf(fp_rep,"waypoint [%s] of type [%s] skipped\n",icao,wpt_type);  
          }
        }
        else { //not low or both low and high coverage
          if  (strchr((const char *) &usage_cd,'T')==NULL) {
            nwpskip++;  
            fprintf(fp_rep,"waypoint [%s] with usage_cd [%s] skipped\n",icao,usage_cd);  
          }
          else { // terminal ...flag it in red...
            struct icao_list * twp = get_icao((char *) &icao);
            if (twp != NULL) {
              fprintf(fp_rep,"[%13s] - [%13s] ",wgs_dlat, wgs_dlong);
              fprintf(fp_rep,"wpt_ident[%s] ",wpt_ident);
              print_wpt_type();
              fprintf(fp_rep,"desc[%s] icao[%s] ",   desc, icao); 
              print_usage_cd(); 
              fprintf(fp_rep," bearing[%s] wpt_distance[%s] ",bearing, wpt_distance);
              fprintf(fp_rep," nav_ident[%s] ",nav_ident);
              print_nav_type();     
              fprintf(fp_rep," nav_key_cd[%s]    *** TERMINAL WP ***\n",nav_key_cd);
              write_sign_files_t_waypoint( strtod((const char *) wgs_dlat,NULL), strtod((const char *) wgs_dlong,NULL), twp->elevation+1000.0);
            }
            else fprintf(fp_rep,"terminal waypoint [%s] not associated with airport...skipped\n",icao);
          }
        } 
      } 
    } 
  }
//  else {
//    printf("skip country code [%s]\n",ctry);
//  }

}


void read_navaid()
{
  read_field(fp_navaid,(char *)&nav_ident,sizeof(nav_ident),tc);
  read_field(fp_navaid,(char *)&type,sizeof(type),tc);
  read_field(fp_navaid,(char *)&ctry,sizeof(ctry),tc);
  read_field(fp_navaid,(char *)&nav_key_cd,sizeof(nav_key_cd),tc);
  read_field(fp_navaid,(char *)&state_prov,sizeof(state_prov),tc);
  read_field(fp_navaid,(char *)&name,sizeof(name),tc);
  read_field(fp_navaid,(char *)&icao,sizeof(icao),tc);
  read_field(fp_navaid,(char *)&wac,sizeof(wac),tc);
  read_field(fp_navaid,(char *)&freq,sizeof(freq),tc);
  read_field(fp_navaid,(char *)&usage_cd,sizeof(usage_cd),tc);
  read_field(fp_navaid,(char *)&chan,sizeof(chan),tc);
  read_field(fp_navaid,(char *)&rcc,sizeof(rcc),tc);
  read_field(fp_navaid,(char *)&freq_prot,sizeof(freq_prot),tc);
  read_field(fp_navaid,(char *)&power,sizeof(power),tc);
  read_field(fp_navaid,(char *)&nav_range,sizeof(nav_range),tc);
  read_field(fp_navaid,(char *)&loc_hdatum,sizeof(loc_hdatum),tc);
  read_field(fp_navaid,(char *)&wgs_datum,sizeof(wgs_datum),tc);
  read_field(fp_navaid,(char *)&wgs_lat,sizeof(wgs_lat),tc);
  read_field(fp_navaid,(char *)&wgs_dlat,sizeof(wgs_dlat),tc);
  read_field(fp_navaid,(char *)&wgs_long,sizeof(wgs_long),tc);
  read_field(fp_navaid,(char *)&wgs_dlong,sizeof(wgs_dlong),tc);
  read_field(fp_navaid,(char *)&slaved_var,sizeof(slaved_var),tc);
  read_field(fp_navaid,(char *)&mag_var,sizeof(mag_var),tc);	
  read_field(fp_navaid,(char *)&elev,sizeof(elev),tc);
  read_field(fp_navaid,(char *)&dme_wgs_lat,sizeof(dme_wgs_lat),tc);
  read_field(fp_navaid,(char *)&dme_wgs_dlat,sizeof(dme_wgs_dlat),tc);
  read_field(fp_navaid,(char *)&dme_wgs_long,sizeof(dme_wgs_long),tc);
  read_field(fp_navaid,(char *)&dme_wgs_dlong,sizeof(dme_wgs_dlong),tc);
  read_field(fp_navaid,(char *)&dme_elev,sizeof(dme_elev),tc);
  read_field(fp_navaid,(char *)&arpt_icao,sizeof(arpt_icao),tc);
  read_field(fp_navaid,(char *)&os,sizeof(os),tc);
  read_field(fp_navaid,(char *)&cycle_date,sizeof(cycle_date),tc2);

 if (strncmp(ctry,country,2)==0) {



/**

	1 - VOR
	2 - VORTAC
	3 - TACAN
	4 - VOR-DME
	5 - NDB
	7 - NDB-DME
	9 - DME (EXCLUDING ILS-DME)


**/
  if (strncmp(elev,"U",1)!=0) {
/*
xxxx --------yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy -------- ---- ---- [-------------] - [-------------] 55555
*/

  fprintf(fp_rep,"%4s ",nav_ident);
  if (strncmp(type,"1",1)==0) 
    fprintf(fp_rep," VOR    ");
  else 
  if (strncmp(type,"2",1)==0) 
    fprintf(fp_rep," VORTAC ");
  else 
  if (strncmp(type,"3",1)==0) 
    fprintf(fp_rep," TACAN  ");
  else 
  if (strncmp(type,"4",1)==0) 
    fprintf(fp_rep," VOR-DME");
  else 
  if (strncmp(type,"5",1)==0) 
    fprintf(fp_rep," NDB    ");
  else 
  if (strncmp(type,"7",1)==0) 
    fprintf(fp_rep," NDB-DME");
  else 
  if (strncmp(type,"9",1)==0) 
    fprintf(fp_rep," DME    ");
  else 
    fprintf(fp_rep," UNKOWN ");
  
//xx  printf(" key [%2s] ",nav_key_cd);
//  printf("%38s ",name);
//xx  printf(" icao [%4s] ",icao);
//xx printf(" icao (airport) [%4s] ",arpt_icao);
//  printf("%8s ",freq);
/**
	H - HIGH LEVEL
	L - LOW LEVEL
	B - BOTH
	R - RNAV
	T - TERMINAL

**/

  if (strncmp(usage_cd,"H",1)==0) 
    fprintf(fp_rep," HIGH LEVEL ");
  else 
  if (strncmp(usage_cd,"L",1)==0) 
    fprintf(fp_rep," LOW LEVEL  ");
  else 
  if (strncmp(usage_cd,"B",1)==0) 
    fprintf(fp_rep," BOTH       ");
  else 
   if (strncmp(usage_cd,"H",1)==0) 
    fprintf(fp_rep," RNAV       ");
  else 
  if (strncmp(usage_cd,"H",1)==0) 
    fprintf(fp_rep," TERMINAL   ");
  else 
    fprintf(fp_rep," unknwn use ");

//  printf("%4s ",chan);

  fprintf(fp_rep," rcc [%5s] ",rcc);
  if (strncmp(rcc," H",2)==0) 
    fprintf(fp_rep," NON-DIRECTIONAL RADIO BEACON (HOMING), 	POWER 50 WATTS TO LESS THAN 2000 WATTS.");
  else 
  if (strncmp(rcc," L",2)==0) 
    fprintf(fp_rep," NORMAL ANTICIPATED INTERFERENCE-FREE 	SERVICE 40 NM UP TO 18,000 FEET.");
  else 
  if (strncmp(rcc," T",2)==0) 
    fprintf(fp_rep," NORMAL ANTICIPATED INTERFERENCE-FREE 	SERVICE 25 NM UP TO 12,000 FEET.");
  else 
  if (strncmp(rcc,"MH",2)==0) 
    fprintf(fp_rep," NON-DIRECTIONAL RADIO BEACON (HOMING) 	POWERLESS THAN 50 WATTS.");
  else 
  if (strncmp(rcc,"HH",2)==0) 
    fprintf(fp_rep," NON-DIRECTIONAL RADIO BEACON (HOMING),  	POWER 2000 WATTS OR MORE.");
  else 
  if (strncmp(rcc,"HA",2)==0) 
    fprintf(fp_rep," NORMAL ANTICIPATED INTERFERENCE - 	FREE SERVICE BELOW 18,000 FEET - 40 	NM; 14,500 - 17,999 FEET - 100 NM 	(CONTIGUOUS 48 STATES ONLY); 18,000 	FEET TO FL 450 - 130 NM;  ABOVE FL 450 - 100");
  else 
  if (strncmp(rcc,"HH",2)==0) 
    fprintf(fp_rep," UNKNOWN.");
 

/**
	H -	NON-DIRECTIONAL RADIO BEACON (HOMING), 	POWER 50 WATTS TO LESS THAN 2000 WATTS.

	L -	NORMAL ANTICIPATED INTERFERENCE-FREE 	SERVICE 40 NM UP TO 18,000 FEET.

	T -	NORMAL ANTICIPATED INTERFERENCE-FREE 	SERVICE 25 NM UP TO 12,000 FEET.

	MH - NON-DIRECTIONAL RADIO BEACON (HOMING) 	POWERLESS THAN 50 WATTS.-

	HH - NON-DIRECTIONAL RADIO BEACON (HOMING),  	POWER 2000 WATTS OR MORE.

	HA - NORMAL ANTICIPATED INTERFERENCE - 	FREE SERVICE BELOW 18,000 FEET - 40 	NM; 14,500 - 17,999 FEET - 100 NM 	(CONTIGUOUS 48 STATES ONLY); 18,000 	FEET TO FL 450 - 130 NM;  ABOVE FL 450 - 100
		  NM.

	U - UNKNOWN


**/


//xx  printf(" freq_prot [%9s] ",freq_prot);
//xx  printf(" power [%4s] ",power);
//xx  printf(" nav_range [%3s] ",nav_range);
//xx  printf(" dme_elev [%5s] ",dme_elev);
//xx  printf(" os [%s] ",os);


//  printf("[%13s] - [%13s] ",wgs_dlat, wgs_dlong);
//  printf("%5s",elev);


//  printf("\n");
  fprintf(fp_rep,"[");  
  fprintf(fp_rep,"%s",nav_ident);
  sprintf(nav_line_1,"%s",nav_ident);

  if (strncmp(type,"1",1)==0) {
    fprintf(fp_rep," VOR    ");
    sprintf(nav_line_1,"%s VOR",nav_line_1);
  }
  else 
  if (strncmp(type,"2",1)==0) {
    fprintf(fp_rep," VORTAC ");
    sprintf(nav_line_1,"%s VORTAC",nav_line_1);
  }
  else 
  if (strncmp(type,"3",1)==0) {
    fprintf(fp_rep," TACAN  ");
    sprintf(nav_line_1,"%s TACAN",nav_line_1);
  }
  else 
  if (strncmp(type,"4",1)==0) {
    fprintf(fp_rep," VOR-DME");
    sprintf(nav_line_1,"%s VOR-DME",nav_line_1);
  }
  else 
  if (strncmp(type,"5",1)==0) {
    fprintf(fp_rep," NDB    ");
    sprintf(nav_line_1,"%s NDB",nav_line_1);
  }
  else 
  if (strncmp(type,"7",1)==0) {
    fprintf(fp_rep," NDB-DME");
    sprintf(nav_line_1,"%s NDB-DME",nav_line_1);
  }
  else 
  if (strncmp(type,"9",1)==0) {
    fprintf(fp_rep," DME    ");
    sprintf(nav_line_1,"%s DME",nav_line_1);
  }
  else {
    fprintf(fp_rep," UNKOWN ");
    sprintf(nav_line_1,"%s UNKNOWN",nav_line_1);
   }
 
  fprintf(fp_rep,"][%s ",name);
    sprintf(nav_line_2,"%s",name);

 // if (frequency()>0.0) {
  //  printf("[%f]",strtod((const char *) &freq,NULL)/1000.0);
    if (freq[6]=='K') {
      fprintf(fp_rep,"%3.0fK",strtod((const char *) &freq,NULL)/1000.0);
      sprintf(nav_line_2,"%s %3.0fK",nav_line_2, strtod((const char *) &freq,NULL)/1000.0);
    }
    else {
      fprintf(fp_rep,"%5.1fM",strtod((const char *) &freq,NULL)/1000.0);
      sprintf(nav_line_2,"%s %5.1fM",nav_line_2, strtod((const char *) &freq,NULL)/1000.0);
    }
//  }
  if (chan[0]!='\0') {
     chan[3]='\0';
     fprintf(fp_rep," %s",chan);
     sprintf(nav_line_2,"%s %s",nav_line_2, chan);
  }
 fprintf(fp_rep," coordinates [%f] - [%f] [%f]",strtod((const char *) wgs_dlat,NULL), strtod((const char *) wgs_dlong,NULL), strtod((const char *) elev, NULL));
 write_sign_files_navaid( strtod((const char *) wgs_dlat,NULL), strtod((const char *) wgs_dlong,NULL), strtod((const char *) elev, NULL));

//  printf("[%13s] - [%13s] ",wgs_dlat, wgs_dlong);
//  printf("%5s",elev);

  fprintf(fp_rep,"]\n");
 
  }
  } //not country...
//  else printf("skipped navaid\n");
}

void find_airport ( char *f, int fs)
{
  // char tc='\t';
  // char tc2='\n';
  bool found=false;
  int i=0; 
  // int n;
  char ch;
  while (!found) {
    if (!putbackflag) {
      fairport[0]=i=0; 
      while (peek(fp_airport)!= '\t') { 
        fairport[i]=fgetc(fp_airport); 
        i++;
        if (i<fs) fairport[i] =0;
      }
      ch=fgetc(fp_airport);
    }
    else {
      putbackflag=false;
    }
    if (strncmp(fairport,f,fs) != 0) {
      skip_record(fp_airport);
    }
    else {
      found=true;
      read_airport_have_id();
    }
  }
  while (found) {
   fairport[0]=i=0; 
    while (peek(fp_airport) !='\t') { 
      fairport[i]=fgetc(fp_airport); 
      i++;
      if (i<fs) fairport[i] =0;
    }
    ch=fgetc(fp_airport);
    if (strncmp(fairport,f,fs) == 0) {
      read_airport_have_id();
      found=true;
    }
    else {
      found=false;
      putbackflag=true;
    }
  }
}


void generate_glideslope();

void read_ils()
{

  read_field(fp_ils,(char *)&rwy_ident,sizeof(rwy_ident),tc);			//  RUNWAY IDENTIFIER	3	A/N	4-2
  read_field(fp_ils,(char *)&comp_type,sizeof(comp_type),tc);			//  COMPONENT TYPE	1	A	4-3
  read_field(fp_ils,(char *)&colctn,sizeof(colctn),tc);			//  COLLOCATION	2	A	4-4
  read_field(fp_ils,(char *)&name,sizeof(name),tc);			//  NAME	38	A/N	4-5
  read_field(fp_ils,(char *)&freq,sizeof(freq),tc);				//  FREQUENCY	8	A/N	4-6
  read_field(fp_ils,(char *)&chan,sizeof(chan),tc);				//  CHANNEL	4	A/N	4-7
  read_field(fp_ils,(char *)&gs_angle,sizeof(gs_angle),tc);			//  GLIDE SLOPE ANGLE	3	A/N	4-8
  read_field(fp_ils,(char *)&lczr_gslctn,sizeof(lczr_gslctn),tc);			//  LOCALIZER OR GLIDE SLOPE  LOCATION	6	a/n	4-9
  read_field(fp_ils,(char *)&loc_mkrlctn,sizeof(loc_mkrlctn),tc);			//  LOCATOR OR MARKER LOCATION	5	A/N	4-11
  read_field(fp_ils,(char *)&elev,sizeof(elev),tc);				//  ELEVATION	5	a/n	4-12
  read_field(fp_ils,(char *)&loc_hdatum,sizeof(loc_hdatum),tc);			//  LOCAL HORIZONTAL DATUM	3	A	4-13
  read_field(fp_ils,(char *)&wgs_datum,sizeof(wgs_datum),tc);			//  LATITUDE	9	a/n	4-14
  read_field(fp_ils,(char *)&ils_cat,sizeof(ils_cat),tc);			//  longitude	10	a/n	4-15
  read_field(fp_ils,(char *)&wgs_lat,sizeof(wgs_lat),tc);			//  world geodetic datum	3	a/n	4-16
  read_field(fp_ils,(char *)&wgs_dlat,sizeof(wgs_dlat),tc);			//  ILS/MLS CATEGORY	1	A/N	4-17
  read_field(fp_ils,(char *)&wgs_long,sizeof(wgs_long),tc);			//  geodetic latitude	9	a/n	4-18
  read_field(fp_ils,(char *)&wgs_dlong,sizeof(wgs_dlong),tc);			//  geodetic longitude	10	a/n	4-19
  read_field(fp_ils,(char *)&nav_ident,sizeof(nav_ident),tc);			//  IDENTIFIER	5	a	4-20
  read_field(fp_ils,(char *)&nav_type,sizeof(nav_type),tc);			//  NAVAID TYPE	1	n	4-21
  read_field(fp_ils,(char *)&nav_ctry,sizeof(nav_ctry),tc);			//  NAVAID country code	2	a	4-22
  read_field(fp_ils,(char *)&nav_key_cd,sizeof(nav_key_cd),tc);			//  NAVAID key code	2	n	4-23
  read_field(fp_ils,(char *)&mag_var,sizeof(mag_var),tc);			//  magnetic variation	12	a/n	4-24
  read_field(fp_ils,(char *)&slave_var,sizeof(slave_var),tc);			//  SLAVED VARIATION	9	A/N	4-25
  read_field(fp_ils,(char *)&ils_brg,sizeof(ils_brg),tc);			//  ILS BEARING COURSE	4	A/N	4-26
  read_field(fp_ils,(char *)&loc_width,sizeof(loc_width),tc);			//  LOCALIZER WIDTH	4	N	4-27
  read_field(fp_ils,(char *)&thd_crossing_hgt,sizeof(thd_crossing_hgt),tc);		//  THRESHOLD CROSSING HEIGHT	2	N	4-28
  read_field(fp_ils,(char *)&dme_bias,sizeof(dme_bias),tc);			//  ILS DME BIAS	2	N	4-29
  read_field(fp_ils,(char *)&cycle_date,sizeof(cycle_date),tc2);			//  cycle date	4	n	4-30
  
  if (rwy_ident_ref[0]=='\0') {
    strncpy( (char *) rwy_ident_ref, (char *) rwy_ident,sizeof(rwy_ident));
    fprintf(fp_rep,"     rwy_ident [%s]\n",rwy_ident);
  }
  else {
     if (strncmp(rwy_ident_ref, rwy_ident,sizeof(rwy_ident))!=0) {
       strncpy( (char *) rwy_ident_last, (char *) rwy_ident_ref,sizeof(rwy_ident_ref));
       generate_glideslope();
       strncpy( (char *) rwy_ident_ref, (char *) rwy_ident,sizeof(rwy_ident));
       rwy_ident_changed=true;
       fprintf(fp_rep,"     rwy_ident [%s]\n",rwy_ident);
     }
  }

//                                         comp_type

   if (mag_var[0]!='\0')   fprintf(fp_rep,"       mag_var: [%s]",mag_var); 

//   if (wgs_lat[0]!='\0')   printf("[%s]-",wgs_lat); 
   if (wgs_dlat[0]!='\0')   fprintf(fp_rep," [%s] -",wgs_dlat); 
//   if (wgs_long[0]!='\0')   printf(" [%s]-",wgs_long); 
   if (wgs_dlong[0]!='\0')   fprintf(fp_rep,"[%s]",wgs_dlong); 

  fprintf(fp_rep," type: "); 

  if (strncmp(comp_type,"L",1)==0) {
     fprintf(fp_rep,"Locator freq: [%s] loc_mkrlctn: [%s] ils_brg: [%s]",freq,loc_mkrlctn,ils_brg);
     if (strncmp(colctn,"L",1)==0) fprintf(fp_rep," collocation: Locator");
     fprintf(fp_rep,"\n");
  }
  else 
  if (strncmp(comp_type,"D",1)==0) fprintf(fp_rep,"DME freq: [%s] chan: [%s] elev: [%s] dme_bias: [%s] ils_brg: [%s]\n",freq,chan,elev,dme_bias,ils_brg);
  else 
  if (strncmp(comp_type,"Z",1)==0) fprintf(fp_rep,"Localizer freq: [%s] lczr_gslctn: [%s] elev: [%s] ils_cat: [%s] slave_var: [%s] loc_width: [%s] ils_brg: [%s]\n",freq,lczr_gslctn,elev,ils_cat,slave_var,loc_width,ils_brg);
  else 
  if (strncmp(comp_type,"G",1)==0) {
    fprintf(fp_rep,"Glide Slope freq: [%s] angle [%s] lczr_gslctn: [%s] elev: [%s] thd_crossing_hgt: [%s]\n",freq, gs_angle,lczr_gslctn,elev,thd_crossing_hgt);
    ils_thd_crossing_hgt = strtod(thd_crossing_hgt, NULL);
    ils_gs_angle = strtod(gs_angle,NULL);
  }
  else 
  if (strncmp(comp_type,"B",1)==0) fprintf(fp_rep,"Back Course Marker freq: [%s] elev: [%s]\n",freq,elev);
  else 
  if (strncmp(comp_type,"I",1)==0) {
    fprintf(fp_rep,"Inner Marker name: [%s] freq: [%s] elev: [%s] loc_mkrlctn: [%s] ils_brg: [%s]\n",name,freq,elev,loc_mkrlctn,ils_brg);
      ils_inner_long  = strtod(wgs_dlong,NULL);
      ils_inner_lat   = strtod(wgs_dlat,NULL);
      strncpy((char *) &ils_inner_name,name,sizeof(ils_inner_name));
  }
  else 
  if (strncmp(comp_type,"M",1)==0) {
    fprintf(fp_rep,"Middle Marker name: [%s] freq: [%s] elev: [%s] loc_mkrlctn: [%s] ils_brg: [%s]",name,freq,elev,loc_mkrlctn,ils_brg);
    if (strncmp(colctn,"L",1)==0) fprintf(fp_rep," collocation: Locator");
    fprintf(fp_rep,"\n");
      ils_middle_long  = strtod(wgs_dlong,NULL);
      ils_middle_lat  = strtod(wgs_dlat,NULL);
      strncpy((char *) &ils_middle_name,name,sizeof(ils_middle_name));
  }
  else 
  if (strncmp(comp_type,"O",1)==0) {
    fprintf(fp_rep,"Outer Marker, name: [%s] freq: [%s] elev: [%s] loc_mkrlctn: [%s] ils_brg: [%s]",name,freq,elev,loc_mkrlctn,ils_brg);
    if (strncmp(colctn,"L",1)==0) fprintf(fp_rep," collocation: Locator");
    fprintf(fp_rep,"\n");
      ils_outer_long  = strtod(wgs_dlong,NULL);
      ils_outer_lat  = strtod(wgs_dlat,NULL);
      strncpy((char *) &ils_outer_name,name,sizeof(ils_outer_name));
  } 
  else 
  if (strncmp(comp_type,"S",1)==0) fprintf(fp_rep,"mls localizer freq: [%s] elev: [%s] ils_cat: [%s] loc_width: [%s] ils_brg: [%s]\n",freq,elev,ils_cat,loc_width,ils_brg);
  else 
  if (strncmp(comp_type,"P",1)==0) fprintf(fp_rep,"mls dme freq: [%s] chan: [%s] elev: [%s] dme_bias: [%s]\n",freq,chan,elev,dme_bias);
  else 
  fprintf(fp_rep,"Warning: unknown ils equipment type, probably military... chan: [%s]\n",chan);
  
//                                          colctn
//  if (strncmp(colctn,"L",1)==0) printf("         collocation: Locator\n");
//  else
  if (strncmp(colctn,"N",1)==0) {
    fprintf(fp_rep,"         collocation: Navaid other than locator");
 
   if (nav_ident[0]!='\0')   fprintf(fp_rep," nav_ident: [%s]",nav_ident); 
   if (nav_key_cd[0]!='\0')   fprintf(fp_rep," nav_key_cd: [%s]",nav_key_cd); 
   if (strncmp(nav_type,"1",1)==0)   fprintf(fp_rep," nav_type: VOR\n"); 
   if (strncmp(nav_type,"2",1)==0)   fprintf(fp_rep," nav_type: VORTAC\n"); 
   if (strncmp(nav_type,"3",1)==0)   fprintf(fp_rep," nav_type: TACAN\n"); 
   if (strncmp(nav_type,"4",1)==0)   fprintf(fp_rep," nav_type: VOR-DME\n"); 
   if (strncmp(nav_type,"5",1)==0)   fprintf(fp_rep," nav_type: NDB\n"); 
   if (strncmp(nav_type,"7",1)==0)   fprintf(fp_rep," nav_type: NDB-DME\n"); 
   if (strncmp(nav_type,"9",1)==0)   fprintf(fp_rep," nav_type: DME (EXCLUDING ILS-DME)\n"); 

  }
  else
  if (strncmp(colctn,"B",1)==0) fprintf(fp_rep,"         collocation: Backcourse marker\n");
  else
  if (strncmp(colctn,"I",1)==0) fprintf(fp_rep,"         collocation: Inner\n");
  else
  if (strncmp(colctn,"M",1)==0) fprintf(fp_rep,"         collocation: Middle\n");
  else
  if (strncmp(colctn,"O",1)==0) fprintf(fp_rep,"         collocation: Outer\n");

//                          name
//  if (name[0]!='\0')   printf("       name: [%s]\n",name); 
//  if (freq[0]!='\0')   printf("       freq: [%s]\n",freq); 
//  if (chan[0]!='\0')   printf("       chan: [%s]\n",chan); 
//  if (lczr_gslctn[0]!='\0')   printf("       lczr_gslctn: [%s]\n",lczr_gslctn); 
//  if (loc_mkrlctn[0]!='\0')   printf("       loc_mkrlctn: [%s]\n",loc_mkrlctn); 
//  if (elev[0]!='\0')   printf("       elev: [%s]\n",elev); 
//   if (loc_hdatum[0]!='\0')   printf("       loc_hdatum: [%s]\n",loc_hdatum); 
//   if (wgs_datum[0]!='\0')   printf("       wgs_datum: [%s]\n",wgs_datum); 
//   if (ils_cat[0]!='\0')   printf("       ils_cat: [%s]\n",ils_cat); 

//   if (nav_ctry[0]!='\0')   printf("       nav_ctry: [%s]\n",nav_ctry); 

//   if (nav_key_cd[0]!='\0')   printf("       nav_key_cd: [%s]\n",nav_key_cd); 
//   if (mag_var[0]!='\0')   printf("       mag_var: [%s]\n",mag_var); 
//   if (slave_var[0]!='\0')   printf("       slave_var: [%s]\n",slave_var); 

//   if (ils_brg[0]!='\0')   printf("       ils_brg: [%s]\n",ils_brg); 
//   if (loc_width[0]!='\0')   printf("       loc_width: [%s]\n",loc_width); 
//   if (thd_crossing_hgt[0]!='\0')   printf("       thd_crossing_hgt: [%s]\n",thd_crossing_hgt); 
//   if (dme_bias[0]!='\0')   printf("       dme_bias: [%s]\n",dme_bias); 



//  if (strncmp(wpt_desc_cd,"S",1)=0) printf(".... see TER_SEG record(s)\n");

}

int lookup_rwy_index(char * rw_nums)
{
  int i;
//  printf("lookup_rwy_index(%s)\n",rw_nums);
  for (i=0; i<MAX_RWY; i++) {
  //  printf("lookup_rwy_index(%s): i= %d [%s]\n",rw_nums,i,_rwy_ident[i]);  
    if (strncmp(rw_nums,(const char *)&_rwy_ident[i],strlen(rw_nums)) ==0) {
//      printf("found at i= %d\n",i);
      return i;
    }
  }
  return -1;
}
void write_ils_glideslope()
{
  char ils_glideslope_filename_xml[1000];
  char ils_glideslope_filename_ac[1000];
  char ils_glideslope_filename_ac_long[1000];

  dlat = strtod((const char *)&_rwy_lat[i_rwy],NULL);
  dlong = strtod((const char *)&_rwy_long[i_rwy],NULL);

  set_subpath_for(dlat,dlong);

  sprintf(ils_glideslope_filename_ac,"%s-%s.ac",icao,_rwy_ident[i_rwy]);
  sprintf(ils_glideslope_filename_xml, "%s/Objects/%s%s-%s.xml", output_base.c_str(), subpath, icao,_rwy_ident[i_rwy]);
  sprintf(ils_glideslope_filename_ac_long, "%s/Objects/%s%s-%s.ac", output_base.c_str(), subpath, icao,_rwy_ident[i_rwy]);

  fprintf(fp_rep,"Glideslope xml filename: [%s] ac filename [%s]\n",ils_glideslope_filename_xml, ils_glideslope_filename_ac);

//  dlong = strtod((const char *)&le_wgs_dlong,NULL);
//  dlat = strtod((const char *)&le_wgs_dlat,NULL);

//  set_subpath();

  set_bucket(dlong,dlat);
  tilenum = gen_index();
  sprintf( tile_filename, "%s/Objects/%s%d.stg",output_base.c_str(), subpath, tilenum );
//    sprintf(tile_addline,"OBJECT_SHARED Models/Airport/glide-slope-1nm-by-3d.ac %s %s %6.1f -%s\n",he_wgs_dlong,he_wgs_dlat, dhe_elev*SG_FEET_TO_METER,he_true_hdg);
  sprintf(tile_addline,"OBJECT_STATIC %s-%s.xml %s %s %6.1f -%s\n",icao,_rwy_ident[i_rwy],_rwy_long[i_rwy],_rwy_lat[i_rwy],ils_elev*SG_FEET_TO_METER,_rwy_t_heading[i_rwy]);
  iclass=3;  //kludge
  do_tile_update();

// write xml file
  FILE * ils_gs_fp;

//  sprintf( filename, "%s/Objects/%s%s.xml",output_base.c_str(), subpath,airport_icao);
//  fprintf(fp_rep,"write xml file [%s]\n",filename);
  ils_gs_fp = fopen(ils_glideslope_filename_xml,"w+");
  if (ils_gs_fp != NULL) {
    fprintf(ils_gs_fp,
       "<?xml version=\"1.0\"?>\n<PropertyList>\n  <path>%s</path>\n  <animation>\n    <type>select</type>\n    <object-name>ils-glideslope</object-name>\n    <condition>\n      <property>/sim/airspace_ils_glideslope/enabled</property>\n    </condition>\n  </animation>\n</PropertyList>\n"
       ,ils_glideslope_filename_ac);
    fclose(ils_gs_fp);
  }
// write ac file
  ils_gs_fp = fopen(ils_glideslope_filename_ac_long,"w+");
  if (ils_gs_fp != NULL) {
    fprintf(ils_gs_fp,
      "AC3Db\nMATERIAL \"Material.003\" rgb 1 0 0 amb 0.5 0.5 0.5 emis 0 0 0 spec 1 1 1 shi 72 trans 0\nOBJECT world\nname \"ils-glideslope\"\nnumvert 6\n");

    double gs_data[3][3];
    double ils_gs_length_nm=5.0;
    double ils_gs_angle_plus=1.0;
    double ils_gs_width_ft=strtod((const char *)&_rwy_width[i_rwy],NULL); //150.0;
    double ils_gs_length=ils_gs_length_nm*SG_NM_TO_METER;
    gs_data[0][0]=0.0;
    gs_data[0][1]=0.0;
    gs_data[0][2]=15.2;

    gs_data[1][0]=ils_gs_length;

    gs_data[1][1]=sin(ils_gs_angle*SGD_DEGREES_TO_RADIANS)*ils_gs_length;
    gs_data[1][2]=SG_FEET_TO_METER*ils_gs_width_ft/2.0;

    gs_data[2][0]=ils_gs_length;
    gs_data[2][1]=sin((ils_gs_angle+ils_gs_angle_plus)*SGD_DEGREES_TO_RADIANS)*ils_gs_length;
    gs_data[2][2]=SG_FEET_TO_METER*ils_gs_width_ft/2.0;;

    int gsi;
    for (gsi=0; gsi<3; gsi++) {
      fprintf(ils_gs_fp,"%f %f %f\n",gs_data[gsi][0],gs_data[gsi][1],-gs_data[gsi][2]);
    }
    for (gsi=0; gsi<3; gsi++) {
      fprintf(ils_gs_fp,"%f %f %f\n",gs_data[gsi][0],gs_data[gsi][1],gs_data[gsi][2]);
    }
    fprintf(ils_gs_fp,
      "numsurf 2\nSURF 0x01\nmat 0\nrefs 3\n0 0 0\n1 0 0\n2 0 0\nSURF 0x01\nmat 0\nrefs 3\n3 0 0\n4 0 0\n5 0 0\nkids 0\n");
    fclose(ils_gs_fp);
  }
}
void generate_glideslope()
{
  i_rwy = lookup_rwy_index((char *)&rwy_ident_ref); //_last);
  if (i_rwy >=0) {
    if (ils_thd_crossing_hgt<=BAD_ELEVATION) { 
      ils_thd_crossing_hgt=50.0;  
      fprintf(fp_rep,"Threshold crossing height set to [%f]\n",ils_thd_crossing_hgt); 
    }
    if (ils_gs_angle<=BAD_GS_ANGLE) { 
      ils_gs_angle=3.0;  
      fprintf(fp_rep,"Glide slope angle not specified, set to [%f] degrees\n",ils_gs_angle); 
    }

    ils_elev = strtod((const char *)_rwy_elev,NULL) + ils_thd_crossing_hgt;

    fprintf(fp_rep,"Set glideslope visual for runway [%s] at lat [%s] long [%s] at elevation [%f] (thd_x_hgt) [%f] with angle [%f]\n",_rwy_ident[i_rwy],_rwy_lat[i_rwy],_rwy_long[i_rwy],ils_elev,ils_thd_crossing_hgt,ils_gs_angle); 
    if (ils_inner_lat!=BAD_LAT_LONG) {
      fprintf(fp_rep,"Set inner marker at lat [%f] long [%f] for marker [%s]\n",ils_inner_lat,ils_inner_long,ils_inner_name);
    }
    if (ils_middle_lat!=BAD_LAT_LONG) {
      fprintf(fp_rep,"Set middle marker at lat [%f] long [%f] for marker [%s]\n",ils_middle_lat,ils_middle_long,ils_middle_name);
    }
    if (ils_outer_lat!=BAD_LAT_LONG) {
      fprintf(fp_rep,"Set outer marker at lat [%f] long [%f] for marker [%s]\n",ils_outer_lat,ils_outer_long,ils_outer_name);
    }
    write_ils_glideslope();
  }
  else {
    fprintf(fp_rep,"Warning: no runway record for ILS runway [%s] [%s]\n",rwy_ident_last, rwy_ident);
  }
}
void open_files()
{
  fp_tiles = fopen("tile_pathlist.txt","w+");
}

void close_files()
{

  fclose(fp_tiles);
//  fclose(fp_icao);
}


int main(int argc, char **argv)
{
  int narg;
  int jarg;
  unsigned int i;

 
  bool no_special=false;
  bool no_class_bcd=false;
  bool no_airport=false; 
  bool no_navaid=false;
  bool no_waypoint=false;
  bool no_ils=false; 

  if (argc<2) {
    cout << "form: airspace code options \nwhere code is a two character mnemomic\n";
    cout << "or\n";
    cout << "      airspace code\nwhere code is a two character mnemomic\n";
    cout << "      options are --no-special  --no-class-bcd --no-airport --no-navaid --no-waypoint --no-ils\n";
    return 1;
  }
 // printf("okay 1 argc is %d\n",argc);
  if (argc>2){
    narg=argc-2;
    for (jarg=0; jarg<narg; jarg++) {
      if (strncmp("--no-special",argv[jarg+2],12)==0) no_special=true;
      else 
      if (strncmp("--no-class-bcd",argv[jarg+2],14)==0) no_class_bcd=true;
      else 
      if (strncmp("--no-airport",argv[jarg+2],12)==0) no_airport=true;
      else 
      if (strncmp("--no-navaid",argv[jarg+2],11)==0) no_navaid=true;
      else 
      if (strncmp("--no-waypoint",argv[jarg+2],13)==0) no_waypoint=true;
      else 
      if (strncmp("--no-ils",argv[jarg+2],8)==0) no_ils=true;
      else 
      {
        cout << "unrecognized options " << jarg+1 << " " << argv[jarg+2] << "\n"; 
        exit(1);
      }
      
    } 
  }  

  open_files();

  sprintf(country,"%s",argv[1]);
 
 // no_class_bcd=true;
  fp_rep   = fopen("summary-airspace.txt","w+");  //airspace processing
  if (!no_class_bcd) {
    SGPath d_country(dafift_base);
    d_country.append("./DAFIFT/BDRY/BDRY_CTRY.TXT");
    SGPath d_parent(dafift_base);
    d_parent.append("./DAFIFT/BDRY/BDRY_PAR.TXT");
    SGPath d_segment(dafift_base);
    d_segment.append("./DAFIFT/BDRY/BDRY.TXT");

    fp_country = fopen( d_country.c_str(), "r" ); 
    fp_parent  = fopen( d_parent.c_str(), "r" );
    fp_segment = fopen( d_segment.c_str(), "r" );

    read_field(fp_country,(char *)&header,sizeof(header),'\n');
    read_field(fp_parent,(char *)&header,sizeof(header),'\n');
    read_field(fp_segment,(char *)&header,sizeof(header),'\n');

    bdry_ident_last[0]=0;

    while (nextok(fp_country)) { 
      read_boundary_country();
      if (strncmp(ctry_1,country,2)==0) {
        if (strncmp(bdry_ident,bdry_ident_last,sizeof(bdry_ident)) != 0) {
          ncountry++;
          fprintf(fp_rep, "\nAirspace Identifier: %s cycle date: %s\n", bdry_ident, cycle_date);
          find_parent((char *) &bdry_ident,sizeof(bdry_ident));
        }
        for (i=0; i<sizeof(bdry_ident); i++) bdry_ident_last[i]=bdry_ident[i];
      }
    }

    fprintf(fp_rep,"number of records for country %s with unique boundary identifiers is %d\n",country,ncountry);
    fprintf(fp_rep,"number of points %d\n",npoints);
    fprintf(fp_rep,"number of airspace elements by class number, number starting at surface\n");
    for (iclass=0; iclass < MAX_CLASSES; iclass++) {
      fprintf(fp_rep," %d %d, %d\n",iclass, cum_class[iclass],cum_class_surface[iclass]);
    }
    fprintf(fp_rep,"max_r_diff was %f\n",max_r_diff);
  
    fclose(fp_country);
    fclose(fp_parent);
    fclose(fp_segment);
  }  
  else fprintf(fp_rep,"no Class B, C, D processing \n");

  fclose(fp_rep); //close the summary report...

//  no_special=true;
  fp_rep  = fopen("summary-suas.txt","w+"); //special use airspace processing
  if (!no_special) {
    SGPath d_country(dafift_base); 
    d_country.append("./DAFIFT/SUAS/SUAS_CTRY.TXT");
    SGPath d_parent(dafift_base);
    d_parent.append("./DAFIFT/SUAS/SUAS_PAR.TXT");
    SGPath d_segment(dafift_base);
    d_segment.append("./DAFIFT/SUAS/SUAS.TXT");
    fp_country = fopen( d_country.c_str(), "r" ); 
    fp_parent  = fopen( d_parent.c_str(), "r" );
    fp_segment = fopen( d_segment.c_str(), "r" );
    read_field(fp_country,(char *)&header,sizeof(header),'\n');
    read_field(fp_parent,(char *)&header,sizeof(header),'\n');
    read_field(fp_segment,(char *)&header,sizeof(header),'\n');
    bdry_ident_last[0]=0;
    putbackflag=false;  //segment carry over
    while (nextok(fp_country)) {  
      read_suas_boundary_country();
      if (strncmp(ctry_1,country,2)==0) {
        if (strncmp(bdry_ident,bdry_ident_last,sizeof(bdry_ident)) != 0) {
          ncountry++;
          fprintf(fp_rep, "\nAirspace Identifier: [%s] cycle date: %s\n", bdry_ident, cycle_date);
          find_suas_parent((char *) &bdry_ident,sizeof(bdry_ident));
        }
        for (i=0; i<sizeof(bdry_ident); i++) bdry_ident_last[i]=bdry_ident[i];
      }
    }
    fprintf(fp_rep,"number of records for country %s with unique boundary identifiers is %d\n",country,ncountry);
    fprintf(fp_rep,"number of points %d\n",npoints);
    fprintf(fp_rep,"number of airspace elements by class number, number starting at surface\n");
    for (iclass=0; iclass < MAX_CLASSES; iclass++) {
      fprintf(fp_rep," %d %d, %d\n",iclass, cum_class[iclass],cum_class_surface[iclass]);
    }
    fprintf(fp_rep,"max_r_diff was %f\n",max_r_diff);

    fclose(fp_country);
    fclose(fp_parent);
    fclose(fp_segment);

  } else fprintf(fp_rep,"no Special Use Airspace processing \n");

  fclose(fp_rep); //close the summary report...

  init_icao_list(); // NOTE: If no-airport=true and no_waypoint=false, the waypoint routine will not be able to lookup airports for terminal waypoints

  //no_airport=true;
  fp_rep  = fopen("summary-airport.txt","w+"); //airport processing
  if (!no_airport) {
    SGPath d_airport(dafift_base); 
    d_airport.append("./DAFIFT/ARPT/ARPT.TXT");
    SGPath d_runway(dafift_base);
    d_runway.append("./DAFIFT/ARPT/RWY.TXT");
    fp_airport = fopen( d_airport.c_str(), "r" ); 
    fp_runway  = fopen( d_runway.c_str(), "r" );
    read_field(fp_airport,(char *)&header,sizeof(header),'\n');
    read_field(fp_runway,(char *)&header,sizeof(header),'\n');
    bdry_ident_last[0]=0;
    putbackflag=false;  //segment carry over
    while (nextok(fp_airport)) {  
      read_airport();
      if (strncmp(arpt_ident,country,2)==0) {
        nairports++;
//        fprintf(fp_rep, "\nAirport Identifier: [" << arpt_ident <<  "] state [" << lookup_state((char *)&state_prov)  << "] name: " << name << " ICAO: "<< icao << " faa host id: " << faa_host_id 
//              <<" type: " << type << "cycle date: " << acycle_date << "\n";
        fprintf(fp_rep,"\nAirport Identifier: [%s] state [%s] name: [%s] ICAO: [%s] elev: %s faa host id [%s] type %s cycle date: %s\n",
                       arpt_ident, lookup_state((char *)&state_prov), name, icao, elev, faa_host_id, type, acycle_date);
        if ((strncmp(icao,"KZ",2) !=0) || (strlen(icao)>2) ) {
          if (strncmp(icao,"PH",2) !=0)  { ///hawai
            if (strncmp(icao,"PA",2) !=0)  { ///ALASKA
              strncpy((char *)&airport_icao,(char *)&icao,sizeof(airport_icao)); 
            }
            else { 
               sprintf(airport_icao,"%s",faa_host_id);    
             } 
          }
          else {
            sprintf(airport_icao,"%s",faa_host_id);    
          }
        } else {
          sprintf(airport_icao,"%s",faa_host_id);    
          if (isdigit(airport_icao[1])) {   
            int ai;
            for (ai=0; ai < (strlen(airport_icao)); ai++) {
              airport_icao[ai]=airport_icao[ai+1];
            } 
          } 
        }      
        fprintf(fp_rep,"airport_icao [%s] icao [%s] faa_host_id [%s] name [%s]\n",airport_icao,icao, faa_host_id,name);
        if (get_icao((char *)&airport_icao)==NULL) add_icao((char *)&airport_icao,strtod((char *)&elev,NULL));

        write_sign_files();
        find_runways((char *) &arpt_ident,sizeof(arpt_ident));
      }
    }
    fprintf(fp_rep,"\nNumber of Airports %d\n",nairports);
    fclose(fp_airport);
    fclose(fp_runway);
  }

   current_icao=head_icao;
   int n_icao=1;
   while (current_icao!=NULL) {
     fprintf(fp_rep,"%d [%s] [%f]\n",n_icao, current_icao->_icao,current_icao->elevation);
     n_icao++;
     current_icao = current_icao->next;
   }


  fclose(fp_rep); //close the summary report...


//**************************************************************************** Navaid signage ***********************************************************
//  no_navaid=true;
  fp_rep   = fopen("summary-navaid.txt","w+");  //navaid processing
  if (!no_navaid) {
    SGPath d_navaid(dafift_base);
    d_navaid.append("./DAFIFT/NAV/NAV.TXT");
    fp_navaid  = fopen( d_navaid.c_str(), "r" );
    read_field(fp_navaid,(char *)&header,sizeof(header),'\n');
    fprintf(fp_rep," id     type               name                       freq   chan       lat              long       elev \n");     
    fprintf(fp_rep,"xxxx --------yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy -------- ---- [-------------] - [-------------] -----\n");
    while (nextok(fp_navaid)) {  
      read_navaid();
    }
    fclose(fp_navaid);
  }
  fclose(fp_rep); //close the summary report...
//**************************************************************************** Waypoint signage ***********************************************************
//  no_waypoint=true;
  fp_rep    = fopen("summary-waypoint.txt","w+");   //waypoint processing
  if (!no_waypoint) {
    SGPath d_waypoint(dafift_base);
    d_waypoint.append("./DAFIFT/WPT/WPT.TXT");
    fp_waypoint  = fopen( d_waypoint.c_str(), "r" );
    if (fp_waypoint !=NULL) {
      read_field(fp_waypoint,(char *)&header,sizeof(header),'\n');
      fprintf(fp_rep," id     type               name                       freq   chan       lat              long       elev \n");     
      fprintf(fp_rep,"xxxx --------yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy -------- ---- [-------------] - [-------------] -----\n");
      while (nextok(fp_waypoint)) {  
        read_waypoint();
      }
    }
    fclose(fp_waypoint);
    fprintf(fp_rep," Waypoints by type: unnamed [%d] ndb [%d] named [%d] off [%d] vfr [%d]\n",type_unnamed,type_ndb,type_named,type_off,type_vfr); 
    fprintf(fp_rep," Usage:  high [%d] low [%d] both [%d] rnav [%d] term [%d]\n",high_usage,low_usage,both_usage, rnav_usage,term_usage);
    fprintf(fp_rep," processed %d total %d skips %d\n",nwp-nwpskip,nwp, nwpskip);
  }
  else { 
    fprintf(fp_rep,"could not open waypoint file\n");
  }
  fclose(fp_rep); //close the summary report...


//**************************************************************************** ILS ***********************************************************
//snooping stage...
//  fp_rep   = fopen("stdout","w+");  //ils processing;
//  no_ils=true;
//                           ILS records
  int nils=0;
  fp_rep   = fopen("summary-ils.txt","w+");  //ils processing;

  if (!no_ils) {
    do_vfr_glideslope=false;  //suppress double vfr glideslope generation...
// ILS.TXT leads the show with runways by airport code so you then look up the airport...
    SGPath d_ils(dafift_base);
    d_ils.append("./DAFIFT/ARPT/ILS.TXT");
    SGPath d_airport(dafift_base);
    d_airport.append("./DAFIFT/ARPT/ARPT.TXT");
    fp_ils  = fopen( d_ils.c_str(), "r" );
    fp_airport = fopen( d_airport.c_str(), "r" ); 

    SGPath d_runway(dafift_base);
    d_runway.append("./DAFIFT/ARPT/RWY.TXT");
    fp_runway  = fopen( d_runway.c_str(), "r" );
    read_field(fp_runway,(char *)&header,sizeof(header),'\n');
 
    read_field(fp_ils,(char *)&header,sizeof(header),'\n');
    read_field(fp_airport,(char *)&header,sizeof(header),'\n');
    bdry_ident_last[0]=0;
    putbackflag=false;  //segment carry over
    char ref_arpt_ident[8];
    read_field(fp_ils,(char *)&arpt_ident,sizeof(arpt_ident),tc);  
    bool done =false;
    while (!done) { //nextok(fp_ils)) { 
      if (strncmp(arpt_ident,country,2)==0) {
        strncpy((char *)&ref_arpt_ident,(char *)&arpt_ident,sizeof(arpt_ident));
//        printf("find airport: [%s]\n",arpt_ident);
        find_airport((char *) &arpt_ident,sizeof(arpt_ident));
        
//        fprintf(fp_rep, "\nAirport Identifier: [" << arpt_ident <<  "] name: [" << name << "] ICAO: ["<< icao << "] faa host id: [" << faa_host_id 
//             << "] type: [" << type << "] lat [" << wgs_dlat << "] long [" << wgs_dlong << "]\n";  //cycle date: [" << acycle_date << "]\n";
        fprintf(fp_rep,"\nAirport Identifier: [%s] name: [%s] ICAO: [%s] faa host id: [%s] type: [%s] lat [%s] long [%s]\n",
                       arpt_ident, name, icao, faa_host_id, type, wgs_dlat, wgs_dlong);

        find_runways((char *) &arpt_ident,sizeof(arpt_ident));


        rwy_ident_ref[0]='\0';
        rwy_ident_changed=false;

        ils_elev             = BAD_ELEVATION;
        ils_thd_crossing_hgt = BAD_ELEVATION;
        ils_gs_angle         = BAD_GS_ANGLE;
        ils_inner_long       = BAD_LAT_LONG;
        ils_middle_long      = BAD_LAT_LONG;
        ils_outer_long       = BAD_LAT_LONG;
        ils_inner_lat        = BAD_LAT_LONG;
        ils_middle_lat       = BAD_LAT_LONG;
        ils_outer_lat        = BAD_LAT_LONG;

        while (strncmp(ref_arpt_ident,arpt_ident,sizeof(arpt_ident))==0) {
          read_ils();
          nils++;
          read_field(fp_ils,(char *)&arpt_ident,sizeof(arpt_ident),tc);
        }
        generate_glideslope();
    
        if (ils_elev <= BAD_ELEVATION) fprintf(fp_rep,"WARNING no elevation referenced for ILS Glideslope!\n");
//      fprintf(fp_rep,"\nGlide slope located at lat [%f] long [%f] at elevation [%f]\n",ils_lat, ils_long, ils_elev);

      }
      else {
   //   printf("[%s] not country [%s] so skip_record\n",arpt_ident,country);
        skip_record(fp_ils);
        if ((nextok(fp_ils)))
          read_field(fp_ils,(char *)&arpt_ident,sizeof(arpt_ident),tc);
        else {  
          done=true; 
     //   printf("end of file!\n"); 
        }
      }  
    }

//  printf("number of records for country %s with unique boundary identifiers is %d\n",country,ncountry);
    fprintf(fp_rep, "\nNumber of ILS runways %d\n", nils);
    fclose(fp_airport);
    fclose(fp_ils);
  }
  fclose(fp_rep); //close the summary report...

  close_files();
//  printf("maxtilespan was %d\n",maxtilespan);
//  printf("good lists were  %d, bad lists were %d\n",good_lists, bad_lists); 
  return 0; 
 }

//////////////////////////////////////////////////////////////////////////////////////////////
/////                           filename:  airspace.cxx                                 //////
//////////////////////////////////////////////////////////////////////////////////////////////
//////                                                                                  //////
//////               airspace: generates files for 3d depiction of airspace             //////
//////               for flighgear                                                      //////
//////               version 0.1 released 2005-12-12 Merry Christmas!                   //////
//////               by Philip Cobbin                                                   //////
//////                                                                                  //////
//////               running:  ./airspace US >airspace.txt                              //////
//////                         to generate files for United States                      //////
//////                         with the formatted report info on airspace               //////
//////                         piped to the text file airspace.txt (5+mb)               //////
//////               running:  ./airspace US clean >airspace.txt                        //////
//////                         to clean out all the auto generated files.               //////
//////                         (Technically anything after the first auto generated     //////
//////                         line is wiped out of tile file                           //////
//////                                                                                  //////
//////               required directories for "Airspace"                                //////
//////                                                                                  //////
//////                   create: /usr/local/share/FlightGear/data/Models/Airspace/      //////
//////                    then create the following subdirectories                      //////
//////                      Class_A  Class_C  Class_E   Class_SD  Class_SP  Class_ST    //////
//////                      Class_B  Class_D  Class_SA  Class_SM  Class_SR  Class_SW    //////
//////                                                                                  //////
//////               compiling:                                                         //////
//   g++ airspace.cxx  /usr/local/lib/libsgmagvar.a /usr/local/lib/libsgmath.a -o airspace  //
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
//#include <simgear/bucket/newbucket.hxx>
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/misc/sg_path.hxx>
/************************************************************/


// Configure the following for your particular directory structure ...
const string dafift_base = "/stage/fgfs05/curt/RawData/Airports";
// const string output_base = "/usr/local/share/FlightGear/Scenery-Airspace";
const string output_base = "./Scenery-Airspace";
const string airspace_path = "/home/curt/Projects/FlightGear/data/Models/Airspace";



/**************** simgear hybred of newbucket ***************/


/*  routines modified from simgear to get tile numbers right */

    int lon;        // longitude index (-180 to 179)
    int lat;        // latitude index (-90 to 89)
    int x;          // x subdivision (0 to 7)
    int y;          // y subdivision (0 to 7)


/** For divide by zero avoidance, this will be close enough to zero */
//#define SG_EPSILON 0.0000001

//#define SG_BUCKET_SPAN      0.125

/**
 * half of a standard SG_BUCKET_SPAN
 */
//#define SG_HALF_BUCKET_SPAN ( 0.5 * SG_BUCKET_SPAN )


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

/********************************************************************* globals ********************************************************/
/*************************************************** what can I say...an ol FORTRAN'r ************************************************/


// ------------------------------------------------DAFIF fields-----------------------------------------------------------

  char tc = '\t';
  char header[1000];  // to read header into to skip over ....

/** boundary (country) record **/
  char bdry_ident[13];
  char bdry_ident_last[13];
  char seg_nbr[6];
  char ctry_1[5];
  char ctry_2[5];
  char ctry_3[5];
  char ctry_4[5];
  char ctry_5[5];
  char cycle_date[8];

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
  char length[6];		//	LENGTH
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


//--------------------------------------------------------------------- now for the fun stuff -----------------------------

//------------------------files---------------------

  FILE * fp_country;
  FILE * fp_parent;
  FILE * fp_segment;
  FILE * fp_airport;
  FILE * fp_runway;

  FILE * fp_arcs;
  FILE * fp_arcs_surface;
  FILE * fp_arcs_amsl;

  FILE * fp_lines;
  FILE * fp_lines_surface;
  FILE * fp_lines_amsl;


  FILE * fp_tile_numbers;

  FILE * fp_class_e;
  FILE * fp_class_d;
  FILE * fp_class_b;
  FILE * fp_class_c;
  FILE * fp_tiles;
  FILE * fp_tiles_b;
  FILE * fp_tiles_c;
  FILE * fp_tiles_d;
  FILE * fp_tiles_e;

//-----------------------various constants------------------------------

#define MAX_CLASSES 12

#define CLASS_A 0
#define CLASS_B 1
#define CLASS_C 2
#define CLASS_D 3
#define CLASS_E 4

#define CLASS_SA 5   // Special use "S" A-Alert
#define CLASS_SD 6   // Special use "S" D-Danger
#define CLASS_SM 7   // Special use "S" M-Military Operations Area
#define CLASS_SP 8   // Special use "S" P-Prohibited
#define CLASS_SR 9   // Special use "S" R-Restricted
#define CLASS_ST 10  // Special use "S" T-Temporary Reserved Airspace
#define CLASS_SW 11  // Special use "S" W-Warning


bool generate_class[MAX_CLASSES] = {
  false,
  true,
  true,
  true,
  false,
  true,
  true,
  true,
  true,
  true,
  true,
  true
};

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
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
};

//idiot proof lower boundary so the depiction goes to the surface...
#define CLASS_FLOOR_ADD 500.0


float rgb_colors[MAX_CLASSES][3]= { 
                         {1.0, 1.0, 1.0},  // Class A not used until I own the 777!    (white)
                         {0.0, 0.0, 1.0},  // class B  // hmm...need some texturing?   (blue)
                         {1.0, 0.0, 1.0},  // class C                                  (magenta)
                         {0.0, 0.0, 1.0},  // class D                                  (blue...dashing used)
                         {1.0, 0.0, 1.0},  // class E                                  (magenta)
                         {0.5, 0.5, 0.0},  // class S A Alert                          (1/2 yellow)
                         {1.0, 0.0, 0.0},  // class S D Danger                         (red)
                         {1.0, 1.0, 0.0},  // class S M MOA                            (yellow)
                         {0.0, 0.0, 0.0},  // class S P Prohibited                     (black)
                         {0.25, 0.25, 0.25},  // class S R Restricted                  (3/4   black)
                         {0.0, 0.0, 0.0},  // class S T Temporary                      (black)
                         {1.0, 0.5, 0.0}   // class S W Warning                        (red/yellow)
                         };

float rgb_transparency[MAX_CLASSES]= {  // transparency used as a que as to whether you should enter lower number is f16 bait...
    0.5,
    0.5,  //B
    0.5,  //C
    0.7,  //D
    0.7,  //E
    0.4,  //S A
    0.4,  //S D
    0.4,  //S M
    0.4,  //S P
    0.4,  //S R
    0.4,  //S T
    0.4,  //S W
}; 

  char * class_arc[MAX_CLASSES] = {
          "class_a_arc_r",
          "class_b_arc_r",
          "class_c_arc_r",
          "class_d_arc_r",
          "class_e_arc_r",
          "class_sa_arc_r",
          "class_sd_arc_r",
          "class_sm_arc_r",
          "class_sp_arc_r",
          "class_sr_arc_r",
          "class_st_arc_r",
          "class_sw_arc_r"
};
  char * class_circle[MAX_CLASSES] = {
          "class_a_circle_r",
          "class_b_circle_r",
          "class_c_circle_r",
          "class_d_circle_r",
          "class_e_circle_r",
          "class_sa_circle_r",
          "class_sd_circle_r",
          "class_sm_circle_r",
          "class_sp_circle_r",
          "class_sr_circle_r",
          "class_st_circle_r",
          "class_sw_circle_r"
};
  char * class_line[MAX_CLASSES] = {
          "class_a_line_d",
          "class_b_line_d",
          "class_c_line_d",
          "class_d_line_d",
          "class_e_line_d",
          "class_sa_line_d",
          "class_sd_line_d",
          "class_sm_line_d",
          "class_sp_line_d",
          "class_sr_line_d",
          "class_st_line_d",
          "class_sw_line_d"
};
  bool class_dash[MAX_CLASSES] = {  // solid versus dashing of boundary,  
                                    // Class D is typically blue dashed lines on sectionals.
                                    // may add a secondary color to dashing for further queing of airspace type as the 
                                    // variations of blue and magenta on sectionals is very heavily overloaded visually.
          false,
          false,
          false,
          true,
          false,
          false,
          false,
          false,
          false,
          false,
          false,
          false
    };



  char * class_path[MAX_CLASSES] = {
                         "Class_A/",
                         "Class_B/",
                         "Class_C/",
                         "Class_D/",
                         "Class_E/",
                         "Class_SA/",
                         "Class_SD/",
                         "Class_SM/",
                         "Class_SP/",
                         "Class_SR/",
                         "Class_ST/",
                         "Class_SW/"
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


//------------------- DAFIF conversions i.e. string latitude to the actual number...---------------------------

  float r;                      // my wonderful collection of kludges...
  float rr;                     // first you make em work then you make em elegant...
  int ir;//
  int irr;//
  int ilatxx;//
  int ilongxx;//
  int ilatxxx;//
  int ilongxxx;//

  float dlat;			//  longitude
  float dlong;			//  latitude
  float dlatc;			//  longitude center
  float dlongc;			//  latitude center
  float dlatp1;			//  longitude p1
  float dlongp1;		//  latitude p1
  float dlatp2;			//  longitude p2
  float dlongp2;		//  latitude p2
  float flatxx;//
  float flongxx;//
  float flatxxx;//
  float flongxxx;//

  float fremlatxxx;//
  float fremlongxxx;//
  char ns;			// n or s
  char ew;			// e or w

  float floor_alt,ceiling_alt;

  float altitude_high;
  float altitude_low;
  char alt_low_type[10];
  char alt_high_type[10];


// --------------------------------------misc--------------------------------------------

  bool cleanfiles=false;        // backup current .stg file and then clean out anything from the first automatically added line to eof

  bool floor_is = true;

  char class_x_filename[200];

  float altitude;
  char alt_digits[10];
  char type_digits[10];
  char tile_filename[1000];
  char tile_filename_bak[1000];
  char tile_addline[1000];

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

//---------------------------------- .ac file generators for circles arcs and lines -----------------------------------

void do_circle(const string &fn, float radnm, float lalt, float halt, int cl_clr,bool dash) 
{
// NUM_SEGMENTS should be even number for dashing to work...
#define NUM_SEGMENTS 72
  int i;
  float vertices[NUM_SEGMENTS*2][3];
  double delta_d = 360.0 / ((double)(NUM_SEGMENTS));
  double xx;
  double yy;
  for (i=0; i<NUM_SEGMENTS; i++) {
    xx = cos( ((double)(i-1))*delta_d*SGD_DEGREES_TO_RADIANS);
    yy = sin( ((double)(i-1))*delta_d*SGD_DEGREES_TO_RADIANS);
    if (fabs(xx) < SG_EPSILON) xx=0.0;
    if (fabs(yy) < SG_EPSILON) yy=0.0;
    vertices[i][0]=xx;
    vertices[i][1]=1.0;
    vertices[i][2]=yy;
  }
  for (i=0; i<NUM_SEGMENTS; i++) {
    xx = cos( ((double)(i-1))*delta_d*SGD_DEGREES_TO_RADIANS);
    yy = sin( ((double)(i-1))*delta_d*SGD_DEGREES_TO_RADIANS);
    if (fabs(xx) < SG_EPSILON) xx=0.0;
    if (fabs(yy) < SG_EPSILON) yy=0.0;
    vertices[i+NUM_SEGMENTS][0]=xx;
    vertices[i+NUM_SEGMENTS][1]=1.0;
    vertices[i+NUM_SEGMENTS][2]=yy;

  }

  SGPath airspace_filename(airspace_path);
  airspace_filename.append(fn);
  string command = "mkdir -p "; command += airspace_filename.dir();
  cout << command << endl;
  system(command.c_str());
  ofstream fout(airspace_filename.c_str());

  fout << 
       "AC3Db\n"<<
       "MATERIAL \"Material.001\" rgb " << 
         rgb_colors[cl_clr][0] << " " << rgb_colors[cl_clr][1] << " "<< rgb_colors[cl_clr][2] <<
       " amb 0.5 0.5 0.5 emis 0 0 0 spec 1 1 1 shi  80 trans "  << rgb_transparency[cl_clr] << "\n"<< 
       "OBJECT world\n"<< 
       "kids 1\n"<< 
       "OBJECT poly\n"<< 
       "name \"Cylinder\"\n"<< 
       "numvert " << NUM_SEGMENTS*2 << "\n";
  for (i=0; i<NUM_SEGMENTS; i++) {
    vertices[i][0]*=radnm*SG_NM_TO_METER;
    vertices[i][1]*=lalt*SG_FEET_TO_METER;
    vertices[i][2]*=radnm*SG_NM_TO_METER;
    fout.width(10);
    fout << vertices[i][0];
    fout.width(10);
    fout << vertices[i][1];
    fout.width(10);
    fout << vertices[i][2] << "\n"; 
  }
  for (i=NUM_SEGMENTS; i<NUM_SEGMENTS*2; i++) {
    vertices[i][0]*=radnm*SG_NM_TO_METER;
    vertices[i][1]*=halt*SG_FEET_TO_METER;
    vertices[i][2]*=radnm*SG_NM_TO_METER;
    fout.width(10);
    fout << vertices[i][0];
    fout.width(10);
    fout << vertices[i][1];
    fout.width(10);
    fout << vertices[i][2] << "\n"; 
  }
  int ns=NUM_SEGMENTS;
  if (dash) ns/=2;
  fout <<
    "numsurf " << ns << "\n";
  for (i=0; i<NUM_SEGMENTS; i++) {
    fout <<
      "SURF 0x30\n"
      "mat 0\n"
      "refs 4\n";
    if (dash) i++;
    if (i<(NUM_SEGMENTS-1)) {
      fout.width(5);
      fout << i                << " 0.0 0.0 " << endl;
      fout.width(5);
      fout << i+NUM_SEGMENTS   << " 0.0 0.0 " << endl;
      fout.width(5);
      fout << i+NUM_SEGMENTS+1 << " 0.0 0.0 " << endl;
      fout.width(5);
      fout << i+1              << " 0.0 0.0 " << endl;
    }
    else {
      fout.width(5);
      fout <<   NUM_SEGMENTS     << " 0.0 0.0 " << endl;
      fout.width(5);
      fout << 0                  << " 0.0 0.0 " << endl;
      fout.width(5);
      fout << NUM_SEGMENTS-1     << " 0.0 0.0 " << endl;
      fout.width(5);
      fout << NUM_SEGMENTS*2-1   << " 0.0 0.0 " << endl;
    }   
  }
  fout << "kids 0\n";
  fout.close();
}
void do_arc(const string &fn, float starta, float enda, float radnm, float lalt, float halt, int cl_clr,bool dash) 
{
// NUM_SEGMENTS should be even number for dashing to work...
#define NUM_ARC_SEGMENTS 72
  float sa; 
  float ea;
  float ca;
  sa=starta;
  ea=enda;
  if (sa>ea) sa-=360.0;
  sa+=180.0;//get ac to flighgear bearings aligned
  ea+=180.0;
  float span = ea-sa;
  int i;
  float vertices[NUM_ARC_SEGMENTS*2][3];
  double delta_d = span / ((double)(NUM_ARC_SEGMENTS-1));
  double xx;
  double yy;
  ca=sa;
  for (i=0; i<NUM_ARC_SEGMENTS; i++) {
    xx = cos( ca*SGD_DEGREES_TO_RADIANS);
    yy = sin( ca*SGD_DEGREES_TO_RADIANS);
    if (fabs(xx) < SG_EPSILON) xx=0.0;
    if (fabs(yy) < SG_EPSILON) yy=0.0;
    vertices[i][0]=xx;
    vertices[i][1]=1.0;
    vertices[i][2]=yy;
    ca+=delta_d;
  }
  ca=sa;
  for (i=0; i<NUM_ARC_SEGMENTS; i++) {
    xx = cos( ca*SGD_DEGREES_TO_RADIANS);
    yy = sin( ca*SGD_DEGREES_TO_RADIANS);
    if (fabs(xx) < SG_EPSILON) xx=0.0;
    if (fabs(yy) < SG_EPSILON) yy=0.0;
    vertices[i+NUM_ARC_SEGMENTS][0]=xx;
    vertices[i+NUM_ARC_SEGMENTS][1]=1.0;
    vertices[i+NUM_ARC_SEGMENTS][2]=yy;
    ca+=delta_d;
  }
  SGPath airspace_filename(airspace_path);
  airspace_filename.append(fn);
  string command = "mkdir -p "; command += airspace_filename.dir();
  cout << command << endl;
  system(command.c_str());
  ofstream fout(airspace_filename.c_str());
  fout << 
       "AC3Db\n"<<
       "MATERIAL \"Material.001\" rgb " << 
         rgb_colors[cl_clr][0] << " " << rgb_colors[cl_clr][1] << " "<< rgb_colors[cl_clr][2] <<
       " amb 0.5 0.5 0.5 emis 0 0 0 spec 1 1 1 shi  80 trans "  << rgb_transparency[cl_clr] << "\n"<< 
       "OBJECT world\n"<< 
       "kids 1\n"<< 
       "OBJECT poly\n"<< 
       "name \"Cylinder\"\n"<< 
       "numvert " << NUM_ARC_SEGMENTS*2 << "\n";
  for (i=0; i<NUM_ARC_SEGMENTS; i++) {
    vertices[i][0]*=radnm*SG_NM_TO_METER;
    vertices[i][1]*=lalt*SG_FEET_TO_METER;
    vertices[i][2]*=radnm*SG_NM_TO_METER;
    fout.width(10);
    fout << vertices[i][0];
    fout.width(10);
    fout << vertices[i][1];
    fout.width(10);
    fout << vertices[i][2] << "\n"; 
  }
  for (i=NUM_ARC_SEGMENTS; i<NUM_ARC_SEGMENTS*2; i++) {
    vertices[i][0]*=radnm*SG_NM_TO_METER;
    vertices[i][1]*=halt*SG_FEET_TO_METER;
    vertices[i][2]*=radnm*SG_NM_TO_METER;
    fout.width(10);
    fout << vertices[i][0];
    fout.width(10);
    fout << vertices[i][1];
    fout.width(10);
    fout << vertices[i][2] << "\n"; 
  }
  int ns=NUM_ARC_SEGMENTS;
  if (dash) ns=(ns/2)+1;
  fout <<
    "numsurf " << ns-1 << "\n";
  for (i=0; i<NUM_ARC_SEGMENTS-1; i++) {
    fout <<
    "SURF 0x30\n"
    "mat 0\n"
    "refs 4\n";
    fout.width(5);
    fout << i                << " 0.0 0.0 " << endl;
    fout.width(5);
    fout << i+NUM_ARC_SEGMENTS   << " 0.0 0.0 " << endl;
    fout.width(5);
    fout << i+NUM_ARC_SEGMENTS+1 << " 0.0 0.0 " << endl;
    fout.width(5);
    fout << i+1              << " 0.0 0.0 " << endl;
    if (dash) i++;
  }
  fout << "kids 0\n";
  fout.close();
}


void do_line(const string &fn, float starta,float distnm, float lalt, float halt, int cl_clr,bool dash) 
{
// NUM_SEGMENTS should be even number for dashing to work...
#define NUM_LINE_SEGMENTS 72
  float cp = 0;
  float dp = 1.0/(float)(NUM_LINE_SEGMENTS-1); 
  float px = 0;
  float py = 0;
  starta+=180.0;//get bearing aligned to flightgear

  int i;
  float vertices[NUM_LINE_SEGMENTS*2][3];
 
  cp=0.0;
  for (i=0; i<NUM_LINE_SEGMENTS; i++) {
    px = cp*cos( starta*SGD_DEGREES_TO_RADIANS);
    py = cp*sin( starta*SGD_DEGREES_TO_RADIANS);
    if (fabs(px) < SG_EPSILON) px=0.0;
    if (fabs(py) < SG_EPSILON) py=0.0;
    vertices[i][0]=px;
    vertices[i][1]=1.0;
    vertices[i][2]=py;
    cp+=dp;
  }
  cp=0.0;
  for (i=0; i<NUM_LINE_SEGMENTS; i++) {
    px = cp*cos( starta*SGD_DEGREES_TO_RADIANS);
    py = cp*sin( starta*SGD_DEGREES_TO_RADIANS);
    if (fabs(px) < SG_EPSILON) px=0.0;
    if (fabs(py) < SG_EPSILON) py=0.0;
    vertices[i+NUM_LINE_SEGMENTS][0]=px;
    vertices[i+NUM_LINE_SEGMENTS][1]=1.0;
    vertices[i+NUM_LINE_SEGMENTS][2]=py;
    cp+=dp;
  }
  SGPath airspace_filename(airspace_path);
  airspace_filename.append(fn);
  string command = "mkdir -p "; command += airspace_filename.dir();
  cout << command << endl;
  system(command.c_str());
  ofstream fout(airspace_filename.c_str());

  fout << 
       "AC3Db\n"<<
       "MATERIAL \"Material.001\" rgb " << 
         rgb_colors[cl_clr][0] << " " << rgb_colors[cl_clr][1] << " "<< rgb_colors[cl_clr][2] <<
       " amb 0.5 0.5 0.5 emis 0 0 0 spec 1 1 1 shi  80 trans "  << rgb_transparency[cl_clr] << "\n"<< 
       "OBJECT world\n"<< 
       "kids 1\n"<< 
       "OBJECT poly\n"<< 
       "name \"Cylinder\"\n"<< 
       "numvert " << NUM_LINE_SEGMENTS*2 << "\n";
  for (i=0; i<NUM_LINE_SEGMENTS; i++) {
    vertices[i][0]*=distnm*SG_NM_TO_METER;
    vertices[i][1]*=lalt*SG_FEET_TO_METER;
    vertices[i][2]*=distnm*SG_NM_TO_METER;
    fout.width(10);
    fout << vertices[i][0];
    fout.width(10);
    fout << vertices[i][1];
    fout.width(10);
    fout << vertices[i][2] << "\n"; 
  }
  for (i=NUM_LINE_SEGMENTS; i<NUM_LINE_SEGMENTS*2; i++) {
    vertices[i][0]*=distnm*SG_NM_TO_METER;
    vertices[i][1]*=halt*SG_FEET_TO_METER;
    vertices[i][2]*=distnm*SG_NM_TO_METER;
    fout.width(10);
    fout << vertices[i][0];
    fout.width(10);
    fout << vertices[i][1];
    fout.width(10);
    fout << vertices[i][2] << "\n"; 
  }
  int ns=NUM_LINE_SEGMENTS;
  if (dash) ns=(ns/2)+1;
  fout <<
    "numsurf " << ns-1 << "\n";
  for (i=0; i<NUM_LINE_SEGMENTS-1; i++) {
    fout <<
    "SURF 0x30\n"
    "mat 0\n"
    "refs 4\n";
    fout.width(5);
    fout << i                << " 0.0 0.0 " << endl;
    fout.width(5);
    fout << i+NUM_LINE_SEGMENTS   << " 0.0 0.0 " << endl;
    fout.width(5);
    fout << i+NUM_LINE_SEGMENTS+1 << " 0.0 0.0 " << endl;
    fout.width(5);
    fout << i+1              << " 0.0 0.0 " << endl;
    if (dash) i++;
  }
  fout << "kids 0\n";
  fout.close();
}

//----------------------------------------------------- decode DAFIF record info routines ---------------------------------------

void set_subpath()
{
  if (dlong>=0.0) {
    ew='e';
    ilongxxx    = (int) dlong;  //truncate    
    fremlongxxx = dlong - truncf(dlong); //(float) ilongxxx;         
    ilongxxx    = (int) (dlong+0.999999);  
    ilongxx     = (int) (ilongxxx+9.999999)/10;
  }
  else {
    ew ='w';
    ilongxxx = (int) (0.9999999-dlong); 
    fremlongxxx = fabsf((float) dlong) - trunc(fabsf(dlong));         
    ilongxx = (int)(ilongxxx+9.999999)/10;
    ilongxxx = -ilongxxx;
    ilongxx  = -ilongxx;
  }
  if (dlat>=0) {
    ns='n';
    ilatxxx = (int)dlat;  //truncate    
    fremlatxxx = dlat - truncf(dlat);         
    ilatxxx = (int)dlat;  //round down
    ilatxx = (ilatxxx)/10;
  }
  else {
    ns='s';
    ilatxxx = (int)(-dlat);  //truncate    
    fremlatxxx = fabsf(dlat) - trunc(fabsf(dlat));// - (float) ilatxxx;         
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

float decode_altitude(char *a)
{
  bool fl=false;
  int i=0;
  int j=0;
  altitude=0.0;
  alt_digits[i]=type_digits[j]=0;
  if (a[0]=='F') {
     fl=true;
     i=2; 
  }
  while (isdigit(a[i])) {
      alt_digits[i]=a[i];
      i++;
      alt_digits[i]=0;
  }
  while (isupper(a[i])) {
    type_digits[j]=a[i];
    i++;
    j++;
    type_digits[j]=0;
  }
  if (alt_digits[0]==0) altitude=0.0; else {
    sscanf(alt_digits,"%f",&altitude);
    if (fl) altitude*=100.0;
  }
  return altitude;
}

void set_floor_alt()  // sets agl's artificially to zero 
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
    dlongp1 = strtof((const char *)&swgs_dlong1,NULL);
    dlatp1 = strtof((const char *)&swgs_dlat1,NULL);
    dlongp2 = strtof((const char *)&swgs_dlong2,NULL);
    dlatp2 = strtof((const char *)&swgs_dlat2,NULL);
          
  geo_inverse_wgs_84(0.0, dlatp1, dlongp1, dlatp2,dlongp2, &start_az, &az2,&start_r);
  start_r*=SG_METER_TO_NM;
  do_decode_radius_from_start_r();
}

void decode_arc_parameters()
{
  dlongc = strtof((const char *)&swgs_dlong0,NULL);
  dlatc = strtof((const char *)&swgs_dlat0,NULL);
  if (strncmp(sshap,"R",1)==0) { // R - CLOCKWISE ARC 
    dlongp1 = strtof((const char *)&swgs_dlong1,NULL);
    dlatp1 = strtof((const char *)&swgs_dlat1,NULL);
    dlongp2 = strtof((const char *)&swgs_dlong2,NULL);
    dlatp2 = strtof((const char *)&swgs_dlat2,NULL);
  }
  if (strncmp(sshap,"L",1)==0) { // R - CLOCKWISE ARC 
    dlongp2 = strtof((const char *)&swgs_dlong1,NULL);
    dlatp2 = strtof((const char *)&swgs_dlat1,NULL);
    dlongp1 = strtof((const char *)&swgs_dlong2,NULL);
    dlatp1 = strtof((const char *)&swgs_dlat2,NULL);
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

void do_tile_list()
{
  fprintf(fp_tiles,"(%4s) %85s <-> %s",picao,tile_filename, tile_addline);
}

void do_tile_update()
{
  if (process_class(iclass)) {
    do_tile_list();
    if (!cleanfiles) {
      string command = "mkdir -p "; command += SGPath(tile_filename).dir();
      cout << command << endl;
      system(command.c_str());
      FILE * ft = fopen(tile_filename,"a");
      if (ft!=NULL) {
        printf("open for append: [%s]\n",tile_filename);
        fprintf(ft,"\n##### THE FOLLOWING LINE WAS AUTOMATICALLY ADDED TO THIS FILE #####\n%s\n",tile_addline);
        fclose(ft);
      } 
      else { 
        cout << "WARNING! COULD NOT APPEND TO FILE [" << tile_filename << "]\n";
      }
    }
    else {
      string command = "mkdir -p "; command += SGPath(tile_filename).dir();
      cout << command << endl;
      system(command.c_str());
      FILE * ft = fopen(tile_filename,"r");
      if (ft!=NULL) {
        struct stat stbuf;
        stat(tile_filename,&stbuf);
        char * ts;
        long tsl = stbuf.st_size;
        ts = (char *) calloc(tsl,sizeof(char));
        long numread =fread((void*)ts,sizeof(char),tsl,ft);
        tsl=numread;
        fclose(ft);
        sprintf(tile_filename_bak,"%s.bak",tile_filename);
        FILE * ft = fopen(tile_filename_bak,"w");
        if (ft!=NULL) {
          fwrite(ts,sizeof(char),tsl,ft);
          fclose(ft);
        }
        char * al = strstr((const char *) ts,"##### THE FOLLOWING LINE WAS AUTOMATICALLY ADDED TO THIS FILE #####");                                                 
        if (al != NULL) { 
          *al= '0';
          long tsrl =0;
          while (&ts[tsrl]!=al) tsrl++; 
          FILE * ft = fopen(tile_filename,"w");
          if (ft!=NULL) {
            fwrite(ts,sizeof(char),tsrl,ft);
            fclose(ft);
          }
        }
      }
    }
  }
}

void do_floor_circle_segment()
{
  if (process_class(iclass)) {
    floor_alt = -(class_height[iclass]+CLASS_FLOOR_ADD);
    sprintf(class_x_filename,"%s%s%d-%d.ac",class_path[iclass],class_circle[iclass],ir,irr);
    do_circle(class_x_filename,r,floor_alt,0.0,iclass,class_dash[iclass]);
    fprintf(fp_tiles_b,"%80s %5s(r)%8.0f (%s) - %8.0f (%s)\n",tile_filename,sradius1,altitude_low, alt_low_type, altitude_high, alt_high_type);
    sprintf(tile_addline,"OBJECT_SHARED Models/Airspace/%s %s %s %f 0.00\n",class_x_filename,swgs_dlong0,swgs_dlat0, altitude_high*SG_FEET_TO_METER);
   do_tile_update();
  }
}
void do_circle_segment()
{
  cout << " C - CIRCLE: " << swgs_dlat0 << " / " << swgs_dlong0 << " radius: " << sradius1 << " Nautical Miles\n";  
  dlong = strtof((const char *)&swgs_dlong0,NULL);
  dlat = strtof((const char *)&swgs_dlat0,NULL);
  set_subpath();
  set_bucket(dlong,dlat);
  tilenum = gen_index();
  sprintf( tile_filename, "%s/Objects/%s%d.stg",
           output_base.c_str(), subpath, tilenum );
  do_decode_altitudes();
  r=strtof((const char *)&sradius1,NULL);
  do_decode_radius();
  if (process_class(iclass)) {
    if (strncmp(plower_alt,"SURFACE",7)==0) {
      do_floor_circle_segment();
    }
    else { // does not start at surface need to calculate floor and generate a file..
      set_floor_alt();
      if (strncmp(alt_high_type,"AGL",3) !=0 ){
        sprintf(class_x_filename,"%s%s%d-%d-%1.0f-%1.0f.ac",class_path[iclass],class_circle[iclass],ir,irr,altitude_low,altitude_high);
        do_circle(class_x_filename,r,floor_alt,0.0,iclass,class_dash[iclass]);
        sprintf(tile_addline,"OBJECT_SHARED Models/Airspace/%s %s %s %f 0.00\n",class_x_filename,swgs_dlong0,swgs_dlat0, altitude_high*SG_FEET_TO_METER);
        do_tile_update();
      }
      else cout << "..no output, upper altitude is AGL\n";
    }
  }
  npoints++;
}

void do_surface_rhumb_line_segment()
{
  if (process_class(iclass)) {
    if (strncmp(alt_high_type,"AGL",3) !=0 ){
      fprintf(fp_lines_amsl,"ICAO: %s class %d shape %s derivations %s",picao, iclass, sshap, sderivation);
      fprintf(fp_lines_amsl," from  %12s %12s to %12s %12s alt range %6.0f (%9s) to %6.0f (%9s) \n", 
            swgs_dlat1, swgs_dlong1, swgs_dlat2, swgs_dlong2,altitude_low,alt_low_type,altitude_high,alt_high_type); 
      decode_line_parameters();
      fprintf(fp_lines_amsl,"start %9.4f r %9.4f\n", start_az, start_r);
      floor_alt = -(class_height[iclass]+CLASS_FLOOR_ADD);
      sprintf(class_x_filename,"%s%s%d-%d-%1.0f-%1.0f-%1.0f.ac",class_path[iclass],class_line[iclass],ir,irr,start_az,0.0,altitude_high);
      do_line(class_x_filename,start_az,start_r,floor_alt,0.0,iclass,class_dash[iclass]);
      sprintf(tile_addline,"OBJECT_SHARED Models/Airspace/%s %s %s %f 0.00\n",class_x_filename,swgs_dlong1,swgs_dlat1, altitude_high*SG_FEET_TO_METER);
      do_tile_update();
    }
    else cout << "...no output, upper altitude is AGL\n";
  }
}

void do_rhumb_line_segment()
{
  cout << " H - RHUMB LINE: " << swgs_dlat1 << " / " << swgs_dlong1 << " - " << swgs_dlat2 << " / " << swgs_dlong2 << "\n"; 
  if (process_class(iclass)) {
    dlong = strtof((const char *)&swgs_dlong1,NULL);
    dlat = strtof((const char *)&swgs_dlat1,NULL);
    set_subpath();
    set_bucket(dlong,dlat);
    tilenum = gen_index();
    sprintf( tile_filename, "%s/Objects/%s%d.stg",
           output_base.c_str(), subpath, tilenum );
    do_decode_altitudes();
    if (strncmp(plower_alt,"SURFACE",7)==0) {
      do_surface_rhumb_line_segment();
    }
    else {
      set_floor_alt();
      if (strncmp(alt_high_type,"AGL",3) !=0 ){
        fprintf(fp_lines_amsl,"ICAO: %s class %d shape %s derivations %s",picao, iclass, sshap, sderivation);
        fprintf(fp_lines_amsl," from  %12s %12s to %12s %12s alt range %6.0f (%9s) to %6.0f (%9s) \n", 
        swgs_dlat1, swgs_dlong1, swgs_dlat2, swgs_dlong2,altitude_low,alt_low_type,altitude_high,alt_high_type); 
        decode_line_parameters();
        fprintf(fp_lines_amsl,"start %9.4f r %9.4f\n", start_az, start_r);
        sprintf(class_x_filename,"%s%s%d-%d-%1.0f-%1.0f-%1.0f.ac",
                          class_path[iclass],class_line[iclass],ir,irr,start_az,altitude_low,altitude_high);
        do_line(class_x_filename,start_az,start_r,floor_alt,0.0,iclass,class_dash[iclass]);
        sprintf(tile_addline,"OBJECT_SHARED Models/Airspace/%s %s %s %f 0.00\n",class_x_filename,swgs_dlong1,swgs_dlat1, altitude_high*SG_FEET_TO_METER);
        do_tile_update();
      }
      else cout << "...no output, upper altitude is AGL\n";
    } 
  }
}

void do_surface_arc_segment()
{
  if (process_class(iclass)) {
    if (strncmp(alt_high_type,"AGL",3) !=0 ){
      if (strncmp(alt_high_type,"FL",2) !=0 ){
        fprintf(fp_arcs_amsl,"class %d shape %s derivations %s",iclass, sshap, sderivation);
        fprintf(fp_arcs_amsl,"from  %12s %12s to %12s %12s centered at %12s %12s alt range %6.0f (%9s) to %6.0f (%9s) \n", 
        swgs_dlat1, swgs_dlong1, swgs_dlat2, swgs_dlong2,  
        swgs_dlat0, swgs_dlong0,altitude_low,alt_low_type,altitude_high,alt_high_type); 
        decode_arc_parameters();
        fprintf(fp_arcs_amsl,"start %9.4f end %9.4f xave %9.4f r %9.4f or r %9.4f diff %6.4f\n", start_az, end_az, ave_r, start_r, end_r, r_diff);
        floor_alt = -(class_height[iclass]+CLASS_FLOOR_ADD);
        sprintf(class_x_filename,"%s%s%d-%d-%1.0f-%1.0f-%1.0f-%1.0f.ac",
                                class_path[iclass],class_arc[iclass],ir,irr,start_az,end_az,0.0,altitude_high);
        do_arc(class_x_filename,start_az,end_az,ave_r,floor_alt,0.0,iclass,class_dash[iclass]);
        sprintf(tile_addline,"OBJECT_SHARED Models/Airspace/%s %s %s %f 0.00\n",class_x_filename,swgs_dlong0,swgs_dlat0, altitude_high*SG_FEET_TO_METER);
        do_tile_update();
      }
    }      
  }
}

void do_arc_segment()
{
  if (strncmp(sshap,"L",1)==0) cout << " L - COUNTERCLOCKWISE ARC -- ";
  if (strncmp(sshap,"R",1)==0) cout << " R - CLOCKWISE ARC -- ";
  if (strncmp(sderivation,"B",1)==0) cout << "   B - DISTANCE AND BEARING: \n";
  if (strncmp(sderivation,"E",1)==0) cout << "   E - END COORDINATES: \n";
  if (strncmp(sderivation,"R",1)==0) {
    cout <<"from " << swgs_dlat1 << " / " << swgs_dlong1 
         << " to " << swgs_dlat2 << " / " << swgs_dlong2 
         << " centered at " << swgs_dlat0 << " / " << swgs_dlong0 <<"\n"; 
    if (process_class(iclass)) {
      dlong = strtof((const char *)&swgs_dlong0,NULL);
      dlat = strtof((const char *)&swgs_dlat0,NULL);
      set_subpath();
      set_bucket(dlong,dlat);
      tilenum = gen_index();
      sprintf( tile_filename, "%s/Objects/%s%d.stg",
               output_base.c_str(), subpath, tilenum );
      do_decode_altitudes();
      fprintf(fp_arcs,"class %d shape %s derivations %s",iclass,sshap,sderivation);
      fprintf(fp_arcs,"from  %s %s to %s %s centered at %s %s\n",swgs_dlat1,swgs_dlong1,swgs_dlat2,swgs_dlong2,swgs_dlat0,swgs_dlong0); 
      if (strncmp(plower_alt,"SURFACE",7)==0) {
        do_surface_arc_segment();
      }
      else {
        set_floor_alt();
        if (strncmp(alt_high_type,"AGL",3) !=0 ){
          fprintf(fp_arcs_amsl,"class %d shape %s derivations %s",iclass, sshap, sderivation);
          fprintf(fp_arcs_amsl,"from  %12s %12s to %12s %12s centered at %12s %12s alt range %6.0f (%9s) to %6.0f (%9s) \n", 
          swgs_dlat1, swgs_dlong1, swgs_dlat2, swgs_dlong2,  
          swgs_dlat0, swgs_dlong0,altitude_low,alt_low_type,altitude_high,alt_high_type); 

          decode_arc_parameters();
          fprintf(fp_arcs_amsl,"start %9.4f end %9.4f xave %9.4f r %9.4f or r %9.4f diff %6.4f\n", start_az, end_az, ave_r, start_r, end_r, r_diff);
          sprintf(class_x_filename,"%s%s%d-%d-%1.0f-%1.0f-%1.0f-%1.0f.ac",
                            class_path[iclass],class_arc[iclass],ir,irr,start_az,end_az,altitude_low,altitude_high);
          do_arc(class_x_filename,start_az,end_az,ave_r,floor_alt,0.0,iclass,class_dash[iclass]);
          sprintf(tile_addline,"OBJECT_SHARED Models/Airspace/%s %s %s %f 0.00\n",class_x_filename,swgs_dlong0,swgs_dlat0, altitude_high*SG_FEET_TO_METER);
          do_tile_update();
        }
        else cout << "...no output, upper altitude is AGL\n";
      } 
    }    
  }  
}

void do_generalized_segment()
{
  cout << " G - GENERALIZED:" << swgs_dlat1 << " / " << swgs_dlong1 << " - " << swgs_dlat2 << " / " << swgs_dlong2 << "\n...handled as rhumb line...\n"; 
  do_rhumb_line_segment();
}

void do_greatcircle_segment()
{
  cout << " B - GREAT CIRCLE\n...ignored...\n";
}

void do_point_segment()
{
  cout << " A - POINT (WITHOUT RADIUS OR BEARING)\n...ignored...\n";
}

//************************************************ DAFIF record reading routines ****************************************


bool nextok(FILE* t)
{
  char tt;
  tt=fgetc(t);
  bool status=!feof(t);
  tt=ungetc(tt,t);
  return status;
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
  char tc='\t';
  char tc2='\n';
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

  cout << " segment number " << sseg_nbr; 

  if (strncmp(sshap,"A",1)==0) do_point_segment();
  if (strncmp(sshap,"B",1)==0) do_greatcircle_segment();
  if (strncmp(sshap,"C",1)==0) do_circle_segment();
  if (strncmp(sshap,"G",1)==0) do_generalized_segment();
  if (strncmp(sshap,"H",1)==0) do_rhumb_line_segment();
  if ((strncmp(sshap,"L",1)==0)||((strncmp(sshap,"R",1)==0))) do_arc_segment();
}
void read_suas_segment()
{
  char tc='\t';
  char tc2='\n';
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

  cout << " segment number " << sseg_nbr; 

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
  if (strncmp(he_elev,"U",1)==0) good_elevations=false;
  if (strncmp(le_elev,"U",1)==0) good_elevations=false;
  if (good_elevations) {
    double dhe_elev = strtof((const char *)&he_elev,NULL);
    double dle_elev = strtof((const char *)&le_elev,NULL);  

    dlong = strtof((const char *)&wgs_dlong,NULL);
    dlat = strtof((const char *)&wgs_dlat,NULL);
  
    set_subpath();
    set_bucket(dlong,dlat);
    tilenum = gen_index();
    sprintf( tile_filename, "%s/Objects/%s%d.stg",
             output_base.c_str(), subpath, tilenum );
//    cout << "tile :" << tile_filename << "\n";
    sprintf(tile_addline,"OBJECT_SHARED Models/Airport/glide-slope-1nm-by-3d.ac %s %s %6.1f -%s\nOBJECT_SHARED Models/Airport/glide-slope-1nm-by-3d.ac %s %s %6.1f -%s\n",
                          he_wgs_dlong,he_wgs_dlat, dhe_elev*SG_FEET_TO_METER,he_true_hdg,
                          le_wgs_dlong,le_wgs_dlat, dle_elev*SG_FEET_TO_METER,le_true_hdg
    );
//  OBJECT_SHARED Models/Airport/glide-slope-1nm-by-3d.ac -72.307364 43.629308 171.6  -168.00
//    cout << "add line(s): " << tile_addline << "\n";
    iclass=3;  //kludge
    do_tile_update();
  }
  else {
    cout <<" elevations for runway ends undefined\n";
  }
 
}

void read_runway()
{
  char tc='\t';
  char tc2='\n';

  read_field(fp_runway,(char *)&high_ident,sizeof(high_ident),tc);	//	HIGH_IDENT
  read_field(fp_runway,(char *)&low_ident,sizeof(low_ident),tc);	//	LOW_IDENT
  read_field(fp_runway,(char *)&high_hdg,sizeof(high_hdg),tc);		//	HIGH_HDG
  read_field(fp_runway,(char *)&low_hdg,sizeof(low_hdg),tc);		//	LOW_HDG
  read_field(fp_runway,(char *)&length,sizeof(length),tc);		//	LENGTH
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




  cout << " runway:    " << high_ident << " - " << low_ident << "\n";
  cout << " type:      [" << type <<"]\n";
  cout << "    true h: " << he_true_hdg << " - " << le_true_hdg << "\n";
  cout << "    lat   : " << he_wgs_dlat << " - " << le_wgs_dlat << "\n"; 
  cout << "    long  : " << he_wgs_dlong << " - " << le_wgs_dlong << "\n";
  cout << "    elev  : " << he_elev << " - " << le_elev << "\n";

  if (strncmp(cld_rwy,"C",1) !=0) {

    do_runway();
  }
  else {
    cout << "runway is closed, skipping \n";
  }

}







char fsegment[13];
char frunway[8];
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
}

void find_runways ( char *f, int fs)
{
  // char tc='\t';
  // char tc2='\n';
  bool found=false;
  int i=0; 
  // int n;
  char ch;
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
  char tc='\t';
  char tc2='\n';
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

      cout << "     Airspace type: " << ptype;
      if (strncmp(ptype,"01",2)==0) cout << " - ADVISORY AREA (ADA) OR (UDA)\n";
      if (strncmp(ptype,"02",2)==0) cout << " - AIR DEFENSE IDENTIFICATION ZONE (ADIZ)\n";
      if (strncmp(ptype,"03",2)==0) cout << " - AIR ROUTE TRAFFIC CONTROL CENTER (ARTCC)\n";
      if (strncmp(ptype,"04",2)==0) cout << " - AREA CONTROL CENTER (ACC)\n";
      if (strncmp(ptype,"05",2)==0) cout << " - BUFFER ZONE (BZ)\n";
      if (strncmp(ptype,"06",2)==0) cout << " - CONTROL AREA (CTA) (UTA) SPECIAL RULES AREA (SRA, U.K. ONLY)\n";
      if (strncmp(ptype,"07",2)==0) cout << " - CONTROL ZONE (CTLZ)  SPECIAL RULES ZONE (SRZ, U.K.  ONLY) MILITARY AERODROME TRAFFIC ZONE (MATZ, U.K. ONLY)\n";
      if (strncmp(ptype,"08",2)==0) cout << " - FLIGHT INFORMATION REGION (FIR)\n";
      if (strncmp(ptype,"09",2)==0) cout << " - OCEAN CONTROL AREA (OCA)\n";
      if (strncmp(ptype,"10",2)==0) cout << " - RADAR AREA\n";
      if (strncmp(ptype,"11",2)==0) cout << " - TERMINAL CONTROL AREA (TCA) OR (MTCA)\n";
      if (strncmp(ptype,"12",2)==0) cout << " - UPPER FLIGHT INFORMATION REGION (UIR)\n";

      cout << "     Name: " << pname << "\n";
      cout << "     ICAO ID: " << picao;
      cout << "     OFFICE CONTROLLING AIRSPACE: " << pcon_auth;        
      cout << "     CALL SIGN : " << pcomm_name;
      cout << "     FREQ.: " << pcomm_freq1 << "  " << pcomm_freq2;
      cout << "     Class: " << pclass << '\n';
      iclass = -1;
      if (strncmp(pclass,"A",1)==0) iclass=0;
      if (strncmp(pclass,"B",1)==0) iclass=1;
      if (strncmp(pclass,"C",1)==0) iclass=2;
      if (strncmp(pclass,"D",1)==0) iclass=3;
      if (strncmp(pclass,"E",1)==0) iclass=4;
      if (iclass != -1) {
        cum_class[iclass]++;
      }  
      else cout << "WHAT IS AIRSPACE [" << pclass << "]\n";
       if (strncmp(pclass_exc,"Y",1)==0) {
         cout << "     Class exception flag: " << pclass_exc << '\n';  
         cout << "     Class exception remarks: " << pclass_ex_rmk << '\n';
       } 
       cout << "     Level: ";
       if (strncmp(plevel,"B",1)==0) cout << "B - HIGH AND LOW LEVEL\n";
       if (strncmp(plevel,"H",1)==0) cout << "H - HIGH LEVEL\n";
       if (strncmp(plevel,"L",1)==0) cout << "L - LOW LEVEL\n";

       cout << "     Upper Altitude limit: " << pupper_alt << "\n";
       cout << "     Lower ALtitude limit: " << plower_alt << "\n";
      if (iclass != -1) {
        if (strncmp(plower_alt,"SURFACE",7)==0) cum_class_surface[iclass]++;
        if (iclass == 3) {
          if (strncmp(plower_alt,"SURFACE",7)!=0) printf("CLASS D WITH NON SURFACE FLOOR?\n");
        }
      }  
      if (prnp[0]!=0) cout << "     Required Navigation Performance: [" << prnp << "] NAUTICAL MILES\n";
      find_segments((char *) &bdry_ident,sizeof(bdry_ident));
    }
  } 
}

void find_suas_parent( char * f, int fs)
{
  char tc='\t';
  char tc2='\n';
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

      cout << "     (Special Use) Airspace type: " << ptype;

      if (strncmp(ptype,"A",1)==0) cout << "		A - ALERT\n";
      if (strncmp(ptype,"D",1)==0) cout << "		D - DANGER\n";
      if (strncmp(ptype,"M",1)==0) cout << "		M - MILITARY OPERATIONS AREA\n";
      if (strncmp(ptype,"P",1)==0) cout << "		P - PROHIBITED\n";
      if (strncmp(ptype,"R",1)==0) cout << "		R - RESTRICTED\n";
      if (strncmp(ptype,"T",1)==0) cout << "		T - TEMPORARY RESERVED AIRSPACE\n";
      if (strncmp(ptype,"W",1)==0) cout << "		W - WARNING\n"; 

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
      cout << "     Name: " << pname; // << "\n";
      cout << "     ICAO ID: " << picao;
      cout << "     CONTROLLING AGENCY: " << suas_con_agcy << "\n";        
      cout << "     CALL SIGN : " << pcomm_name;
      cout << "     FREQ.: " << pcomm_freq1 << "  " << pcomm_freq2 << "\n";
//      cout << "     Class: " << pclass << '\n';
     

      if (iclass != -1) {
        cum_class[iclass]++;
      }  
      else cout << "WHAT IS SPECIAL AIRSPACE [" << ptype << "] .... EXPECT A SEGMENTATION FAULT....\n";

       cout << "     Level: ";
       if (strncmp(plevel,"B",1)==0) cout << "B - HIGH AND LOW LEVEL\n";
       if (strncmp(plevel,"H",1)==0) cout << "H - HIGH LEVEL\n";
       if (strncmp(plevel,"L",1)==0) cout << "L - LOW LEVEL\n";

       cout << "     Upper Altitude limit: " << pupper_alt << "\n";
       cout << "     Lower ALtitude limit: " << plower_alt << "\n";

       cout << "     Effective Times:      " << suas_eff_times << "\n";
       cout << "     Weather:              " << suas_wx << "\n";
       cout << "     Effective Date:       " << suas_eff_date << "\n";

      find_suas_segments((char *) &bdry_ident,sizeof(bdry_ident));
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
  read_field(fp_airport,(char *)&acycle_date,sizeof(acycle_date),tc);
  read_field(fp_airport,(char *)&terrain,sizeof(terrain),tc);
  read_field(fp_airport,(char *)&hydro,sizeof(hydro),'\n');

}

void generate_circles()  //a kludgy dynasaur... it should be dead code...
{
  int i;
  int j;
  float r,rr, radius;

  ceiling_alt=0.0;
  floor_alt = -(class_height[CLASS_D]+CLASS_FLOOR_ADD);
  for (i=3; i<8; i++) {
    for (j=0; j<10; j++) {
       r=(float) i;
       rr=(float) j;
       radius=r+(rr/10.0);
       sprintf(class_x_filename,"%s%s%d-%d.ac",class_path[CLASS_D],class_circle[CLASS_D],i,j);
       do_circle(class_x_filename,radius,floor_alt,0.0,CLASS_D,class_dash[CLASS_D]);      
    }
  }
  floor_alt = -(class_height[CLASS_C]+CLASS_FLOOR_ADD);
  i=5;
  j=0;
  r=(float) i;
  rr=(float) j;
  radius=r+(rr/10.0);

  sprintf(class_x_filename,"%s%s%d-%d.ac",class_path[CLASS_C],class_circle[CLASS_C],i,j);
 
  do_circle(class_x_filename,radius,floor_alt,0.0,CLASS_C,class_dash[CLASS_C]);
  i=10;
  j=0;
  r=(float) i;
  rr=(float) j;
  radius=r+(rr/10.0);

  sprintf(class_x_filename,"%s%s%d-%d.ac",class_path[CLASS_C],class_circle[CLASS_C],i,j);
 
  do_circle(class_x_filename,radius,floor_alt,0.0,CLASS_C,class_dash[CLASS_C]);
  int b_r[12][2] = {
         {4,0},
         {5,0},
         {6,0},
         {7,0},
         {8,0},
         {8,5},
         {10,0},
         {10,5},
         {11,0},
         {12,0},
         {15,0},
         {20,0}
  };
  floor_alt = -(class_height[CLASS_B]+CLASS_FLOOR_ADD);
  for (i=0; i<12; i++) {
    r=(float) b_r[i][0];
    rr=(float) b_r[i][1];
    radius=r+(rr/10.0);
    sprintf(class_x_filename,"%s%s%d-%d.ac",class_path[CLASS_B],class_circle[CLASS_B],b_r[i][0],b_r[i][1]);
    printf("Class b file name: %s radius %f, floor_alt height %f\n",class_x_filename,radius,floor_alt);
    do_circle(class_x_filename,radius,floor_alt,0.0,CLASS_B,class_dash[CLASS_B]);
  }

}

void open_files()
{
  fp_class_e = fopen("circle_radiu_class_e.txt","w+");
  fp_class_d = fopen("circle_radiu_class_d.txt","w+");
  fp_class_b = fopen("circle_radiu_class_b.txt","w+");
  fp_class_c = fopen("circle_radiu_class_c.txt","w+");

  fp_arcs    = fopen("arcs.txt","w+");
  fp_arcs_surface = fopen("arcs_surface.txt","w+");
  fp_arcs_amsl = fopen("arcs_amsl.txt","w+");

  fp_lines    = fopen("lines.txt","w+");
  fp_lines_surface = fopen("lines_surface.txt","w+");
  fp_lines_amsl = fopen("lines_amsl.txt","w+");

  fp_tile_numbers=fopen("tile_numbers.txt","w+");
  fp_tiles = fopen("tile_pathlist.txt","w+");
  fp_tiles_b = fopen("tile_pathlist_b.txt","w+");
  fp_tiles_c = fopen("tile_pathlist_c.txt","w+");
  fp_tiles_d = fopen("tile_pathlist_d.txt","w+");
  fp_tiles_e = fopen("tile_pathlist_e.txt","w+");
}

void close_files()
{
  fclose(fp_class_e);
  fclose(fp_class_d);
  fclose(fp_class_b);
  fclose(fp_class_c);

  fclose(fp_arcs);
  fclose(fp_arcs_surface);
  fclose(fp_arcs_amsl);

  fclose(fp_lines);
  fclose(fp_lines_surface);
  fclose(fp_lines_amsl);

  fclose(fp_tile_numbers);
  fclose(fp_tiles);
  fclose(fp_tiles_b);
  fclose(fp_tiles_c);
  fclose(fp_tiles_d);
  fclose(fp_tiles_e);

}

int main(int argc, char **argv)
{
  int narg;
  int jarg;
  unsigned int i;
  char country[2];
 
  bool no_special=false;
  bool no_class_bcd=false;

  if (argc<2) {
    cout << "form: airspace code options \nwhere code is a two character mnemomic\n";
    cout << "or\n";
    cout << "      airspace code clean\nwhere code is a two character mnemomic and clean (case sensitive) requests to clean auto generated lines from tile files\n";
    cout << "      options are --no-special  --no-class-bcd \n";
    return 1;
  }
  if (argc==3) {
     if (strncmp("clean",argv[2],5)==0) {
       cleanfiles=true;
     }
  }
  if (argc>=3){
    narg=argc-2;
    for (jarg=0; jarg<narg; jarg++) {
      if (strncmp("--no-special",argv[jarg+2],12)==0) no_special=true;
      else {
        if (strncmp("--no-class-bcd",argv[jarg+2],14)==0) no_class_bcd=true;
          else {
            if (strncmp("clean",argv[jarg+2],5)==0) {
              cleanfiles=true;
            }
            else {
               cout << "unrecognized options " << jarg+1 << " " << argv[jarg+2] << "\n"; 
               exit(1);
            }
         }
      }
    } 
  }  

//  generate_circles();

  open_files();

  sprintf(country,"%s",argv[1]);
      
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
        cout << "\nAirspace Identifier: " << bdry_ident <<  " cycle date: " << cycle_date << "\n";
        find_parent((char *) &bdry_ident,sizeof(bdry_ident));
      }
      for (i=0; i<sizeof(bdry_ident); i++) bdry_ident_last[i]=bdry_ident[i];
    }
  }

  printf("number of records for country %s with unique boundary identifiers is %d\n",country,ncountry);
  printf("number of points %d\n",npoints);
  printf("number of airspace elements by class number, number starting at surface\n");
  for (iclass=0; iclass < MAX_CLASSES; iclass++) {
    printf(" %d %d, %d\n",iclass, cum_class[iclass],cum_class_surface[iclass]);
  }
  printf("max_r_diff was %f\n",max_r_diff);
  
  fclose(fp_country);
  fclose(fp_parent);
  fclose(fp_segment);

  printf("closed files...\n");  
 
  }  else printf("no Class B, C, D processing \n");

  if (!no_special) {

  SGPath d_country(dafift_base);
  d_country.append("./DAFIFT/BDRY/BDRY_CTRY.TXT");
  SGPath d_parent(dafift_base);
  d_parent.append("./DAFIFT/BDRY/BDRY_PAR.TXT");
  SGPath d_segment(dafift_base);
  d_segment.append("./DAFIFT/BDRY/BDRY.TXT");

  fp_country = fopen( d_country.c_str(), "r" ); 
  fp_parent  = fopen( d_parent.c_str(), "r" );
  fp_segment = fopen( d_segment.c_str(), "r" );

  printf("opened files again\n");
  read_field(fp_country,(char *)&header,sizeof(header),'\n');
  printf("survived read field call\n");
  read_field(fp_parent,(char *)&header,sizeof(header),'\n');
  read_field(fp_segment,(char *)&header,sizeof(header),'\n');

  bdry_ident_last[0]=0;

  putbackflag=false;  //segment carry over
  printf("start while\n");
  while (nextok(fp_country)) {  
    read_suas_boundary_country();
    if (strncmp(ctry_1,country,2)==0) {
      if (strncmp(bdry_ident,bdry_ident_last,sizeof(bdry_ident)) != 0) {
        ncountry++;
        cout << "\nAirspace Identifier: [" << bdry_ident <<  "] cycle date: " << cycle_date << "\n";
        find_suas_parent((char *) &bdry_ident,sizeof(bdry_ident));
      }
      for (i=0; i<sizeof(bdry_ident); i++) bdry_ident_last[i]=bdry_ident[i];
    }
  }
  printf("number of records for country %s with unique boundary identifiers is %d\n",country,ncountry);
  printf("number of points %d\n",npoints);
  printf("number of airspace elements by class number, number starting at surface\n");
  for (iclass=0; iclass < MAX_CLASSES; iclass++) {
    printf(" %d %d, %d\n",iclass, cum_class[iclass],cum_class_surface[iclass]);
  }
  printf("max_r_diff was %f\n",max_r_diff);

  fclose(fp_country);
  fclose(fp_parent);
  fclose(fp_segment);

  } else printf("no Special Use processing \n");

  SGPath d_airport(dafift_base);
  d_airport.append("./DAFIFT/ARPT/ARPT.TXT");
  SGPath d_runway(dafift_base);
  d_runway.append("./DAFIFT/ARPT/RWY.TXT");

  fp_airport = fopen( d_airport.c_str(), "r" ); 
  fp_runway  = fopen( d_runway.c_str(), "r" );

  printf("opened files again\n");
  read_field(fp_airport,(char *)&header,sizeof(header),'\n');
  read_field(fp_runway,(char *)&header,sizeof(header),'\n');

  bdry_ident_last[0]=0;

  putbackflag=false;  //segment carry over
  printf("start while\n");
  while (nextok(fp_airport)) {  
    read_airport();
    if (strncmp(arpt_ident,country,2)==0) {
    nairports++;
  //    if (strncmp(bdry_ident,bdry_ident_last,sizeof(bdry_ident)) != 0) {
    //    ncountry++;
       cout << "\nAirport Identifier: [" << arpt_ident <<  "] name: " << name << " ICAO: "<< icao << " faa host id: " << faa_host_id 
            <<" type: " << type << "cycle date: " << acycle_date << "\n";
        if (strncmp(type,"C",1) !=0) {// C is active military...
          find_runways((char *) &arpt_ident,sizeof(arpt_ident));
        }
        else {
          cout << "active military...skipping \n";
        }
//      }
//      for (i=0; i<sizeof(bdry_ident); i++) bdry_ident_last[i]=bdry_ident[i];
    }
  }
//  printf("number of records for country %s with unique boundary identifiers is %d\n",country,ncountry);
  cout << "\nNumber of Airports " << nairports << "\n";
  fclose(fp_airport);
  fclose(fp_runway);

  close_files();

  return 0; 
  
}

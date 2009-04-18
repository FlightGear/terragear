// testarray.cxx

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <simgear/compiler.h>
#include <simgear/bucket/newbucket.hxx>
#include <sys/types.h>
#include <sys/stat.h>
#include <iostream>
#include "array.hxx"

using std::cout;
using std::endl;

#ifdef _MSC_VER
#define M_ISDIR _S_IFDIR
#else
#define M_ISDIR __S_IFDIR
#endif

static void give_help( char * name )
{
  cout << "Usage:   " << name << " longitude latitude work_dir" << endl;
  cout << "example: " << name << " 151.18 -33.94 your\\work\\dir, for YSSY (Sydney)" << endl;
}

static void check_for_help( int argc, char **argv )
{
   for(int i = 1; i < argc; i++)
   {
      char * parg = argv[i];
      if(( strcmp(parg,"-h") == 0 )||
         ( strcmp(parg,"--help") == 0 ))
      {
         give_help( argv[0] );
         exit(0);
      }
   }
}

static int is_file_or_directory( char * cur_item )
{
   struct stat buf;
   if( stat( cur_item, &buf ) == 0 ) {
      if ( buf.st_mode & M_ISDIR )
         return 1; // is directory
      else
         return 2; // is file
   }
   return 0;
}

int main( int argc, char **argv ) {
    double lon, lat;

    check_for_help(argc, argv);
    
    if ( argc != 4 ) {
        cout << "ERROR: Needs 3 arguments!" << endl;
        give_help( argv[0] );
        exit(-1);
    }

    //lon = -146.248360; lat = 61.133950;  // PAVD (Valdez, AK)
    //lon = -110.664244; lat = 33.352890;  // P13
    //lon = 150.156; lat = -38.474; // Lithgow, NSW, Australia
    //lat = -33.943392; lon = 151.179773; // YSSY Sydney Intl  tile=e150s30

    // get arguments
    lon = atof( argv[1] );
    lat = atof( argv[2] );
    string work_dir = argv[3];
    
    if ( is_file_or_directory((char *) work_dir.c_str() ) != 1 ) {
       cout << "ERROR: '" << work_dir << "' is not a valid directory!" << endl;
       exit(1);
    }
    if( ( lon > 180.0 ) || ( lon < -180.0 ) ) {
       cout << "ERROR: logitude '" << lon << "' not in world range! -180.0 to 180.0" << endl;
       exit(1);
    }
    if( ( lat > 90.0 ) || ( lat < -90.0 ) ) {
       cout << "ERROR: latitude '" << lat << "' not in world range! -90.0 to 90.0" << endl;
       exit(1);
    }

    SGBucket b( lon, lat );
    cout << "input: lon=" << lon << ", lat=" << lat <<
       ", gives Bucket = " << b << endl;

    string base = b.gen_base_path();
    string path = work_dir + "/" + base;

    if ( is_file_or_directory((char *) path.c_str() ) != 1 ) {
       cout << "ERROR: '" << path << "' is not a valid directory!" << endl;
       exit(1);
    }
    
    string arraybase = path + "/" + b.gen_index_str();
    cout << "arraybase = " << arraybase << endl;
    
    path = arraybase + ".arr";
    if ( is_file_or_directory((char *) path.c_str() ) != 2 ) {
      path += ".gz";
       if ( is_file_or_directory((char *) path.c_str() ) != 2 ) {
         cout << "WARNING: can not locate " << arraybase << ".arr, nor .arr.gz!" << endl;
         cout << "Query will probably fail, with zero result!" << endl;
       }
    }
    
    TGArray a(arraybase);
    a.parse( b );

    lon *= 3600;
    lat *= 3600;
    cout << "altitude of " << a.altitude_from_grid(lon, lat) <<
       " is indicated." << endl;

    return 0;
}

// eof - testarray.cxx

// remote_server.c -- Written by Curtis Olson
//                 -- for CSci 5502

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <simgear/compiler.h>

#include <iostream>
#include <string>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _MSC_VER
#  include <winsock2.h>
   typedef int socklen_t;
#else
#  include <sys/time.h>		// FD_ISSET(), etc.
#  include <unistd.h>
#  include <sys/socket.h>		// bind
#  include <netinet/in.h>
#  include <sys/wait.h>
#endif
#include <sys/stat.h>		// for stat()
#include <time.h>               // for time();


#include <sys/types.h>

#include <simgear/bucket/newbucket.hxx>
#include <simgear/misc/sg_path.hxx>

using std:: cout ;
using std:: cerr ;
using std:: endl ;
using std:: string ;

#if defined (__sun) || defined (__CYGWIN__) || defined(sgi)
#  define WAIT_ANY (pid_t)-1
#endif

#define MAXBUF 16384

static double start_lon, start_lat;
static double lat = 0.0;
static double lon = 0.0;
static double dy = 0.0;
static int pass = 0;
static double area_width = 10.0; // width of generated area in degrees
static double area_height = 10.0; // height of generated area in degrees

#ifdef _MSC_VER

#define numWorkerThreads 10

HANDLE	gWorkerThreads[numWorkerThreads];
HANDLE	gIoPort;
DWORD WINAPI ThreadProc(void *);

#endif


int make_socket (unsigned short int* port) {
    int sock;
    struct sockaddr_in name;
    socklen_t length;

    // Create the socket.
    sock = socket (PF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
	perror ("socket");
	exit (EXIT_FAILURE);
    }

    // Give the socket a name.
    name.sin_family = AF_INET;
    name.sin_port = 0 /* htons (port) */;
    name.sin_addr.s_addr = htonl (INADDR_ANY);
    if (bind (sock, (struct sockaddr *) &name, sizeof (name)) < 0) {
	perror ("bind");
	exit (EXIT_FAILURE);
    }

    // Find the assigned port number
    length = sizeof(struct sockaddr_in);
    if ( getsockname(sock, (struct sockaddr *) &name, &length) ) {
	perror("Cannot get socket's port number");
    }
    *port = ntohs(name.sin_port);

    return sock;
}


#if 0
// let's keep these two around for a while in case we need to revive
// them

// return true if file exists
static bool file_exists( const string& file ) {
    struct stat buf;

    if ( stat( file.c_str(), &buf ) == 0 ) {
	return true;
    } else {
	return false;
    }
}



// check if the specified tile has data defined for it [ depricated ]
static bool has_data( const string& path, const SGBucket& b ) {
    string array_file = path + ".arr" + "/" + b.gen_base_path()
	+ "/" + b.gen_index_str() + ".arr";
    if ( file_exists( array_file ) ) {
	return true;
    }

    array_file += ".gz";
    if ( file_exists( array_file ) ) {
	return true;
    }

    return false;
}
#endif


// initialize the tile counting system
void init_tile_count( const string& chunk ) {
    // pre-pass
    pass = 0;

    // initial bogus value
    lat = 100;

    // determine tile height
    SGBucket tmp1;
    dy = tmp1.get_height();

    string lons = chunk.substr(0, 4);
    string lats = chunk.substr(4, 3);
    cout << "lons = " << lons << " lats = " << lats << endl;

    string horz = lons.substr(0, 1);
    start_lon = atof( lons.substr(1,3).c_str() );
    if ( horz == "w" ) { start_lon *= -1; }

    string vert = lats.substr(0, 1);
    start_lat = atof( lats.substr(1,2).c_str() );
    if ( vert == "s" ) { start_lat *= -1; }

    cout << "start_lon = " << start_lon << "  start_lat = " << start_lat
	 << endl;
}


// return the next tile
long int get_next_tile() {
    SGBucket b;
    static double shift_over = 0.0;
    static double shift_up = 0.0;
    static bool first_time = true;
    static time_t start_seconds, seconds;
    static int counter;
    static int global_counter;

    // first time this routine is called, init counters
    if ( first_time ) {
        first_time = false;
        start_seconds = seconds = time(NULL);
        counter = global_counter = 0;
    }

    // cout << "lon = " << lon << " lat = " << lat << endl;
    // cout << "start_lat = " << start_lat << endl;

    if ( lon > start_lon + area_width ) {
        // increment to next row
        // skip every other row (to avoid two clients working on
        // adjacent tiles)
        lat += 2.0 * dy;

        SGBucket tmp( SGGeod::fromDeg(0.0, lat) );
        double dx = tmp.get_width();
        lon = start_lon + (shift_over*dx) + (dx*0.5);
    }

    if ( lat > start_lat + area_height ) {
        ++pass;
        if ( pass == 1 ) {
            shift_over = 0.0;
            shift_up = 0.0;
        } else if ( pass == 2 ) {
            shift_over = 1.0;
            shift_up = 0.0;
        } else if ( pass == 3 ) {
            shift_over = 0.0;
            shift_up = 1.0;
        } else if ( pass == 4 ) {
            shift_over = 1.0;
            shift_up = 1.0;
        } else {
            return -1;
        }

        // reset lat
        // lat = -89.0 + (shift_up*dy) - (dy*0.5);
        // lat = 27.0 + (0*dy) + (dy*0.5);
        lat = start_lat + (shift_up*dy) + (dy*0.5);

        // reset lon
        SGBucket tmp( SGGeod::fromDeg(0.0, lat) );
        double dx = tmp.get_width();
        // lon = -82 + (shift_over*dx) + (dx*0.5);
        lon = start_lon + (shift_over*dx) + (dx*0.5);

        cout << "starting pass = " << pass
            << " with lat = " << lat << " lon = " << lon << endl;
    }

    // if ( ! start_lon ) {
    // lon = -180 + dx * 0.5;
    // } else {
    //    start_lon = false;
    // }

    b = SGBucket( SGGeod::fromDeg(lon, lat) );

    // increment to next tile
    SGBucket tmp( SGGeod::fromDeg(0.0, lat) );
    double dx = tmp.get_width();

    // skip every other column (to avoid two clients working on
    // adjacent tiles)
    lon += 2.0 * dx;

    ++global_counter;
    ++counter;

    time_t tmp_time = time(NULL);
    if ( tmp_time != seconds ) {
	seconds = tmp_time;
	cout << "Current tile per second rate = " << counter << endl;
	cout << "Overall tile per second rate = "
	     << (double)global_counter / (double)(seconds - start_seconds)
	     << endl;
	cout << "Overall tile per hour rate = "
	     <<  (double)global_counter * 3600.0 /
	           (double)(seconds - start_seconds)
	     << endl;
	counter = 0;
    }

    return b.gen_index();
}


// log a pending tile (has been given out as a taks for some client)
void log_pending_tile( const string& path, long int tile ) {
    SGBucket b(tile);

    string pending_file = path + "/" + b.gen_index_str() + ".pending";

    string command = "touch " + pending_file;
    if ( system( command.c_str() ) == -1 ) {
        cout << "Could not issue command " << command << endl;
    }    
}


// a tile is finished (removed the .pending file)
void log_finished_tile( const string& path, long int tile ) {
    SGBucket b(tile);

    string finished_file = path + "/" + b.gen_index_str() + ".pending";
    // cout << "unlinking " << finished_file << endl;
    unlink( finished_file.c_str() );
}


// make note of a failed tile
void log_failed_tile( const string& path, long int tile ) {
    SGBucket b(tile);

    string failed_file = path + "/" + b.gen_index_str() + ".failed";

    string command = "touch " + failed_file;
    
    if ( system( command.c_str() ) == -1 ) {
        cout << "Could not issue command " << command << endl;
    }

    cout << "logged bad tile = " << tile << endl;
}


// display usage and exit
void usage( const string name ) {
    cout << "Usage: " << name
	 << "[--width=<width> --height=<height>] "
	 << " <work_base> <output_base> chunk1 chunk2 ..."
	 << endl;
    cout << "\twhere chunk represents the south west corner of the area"
	 << endl;
    cout << "\tto build and is of the form [we]xxx[ns]yy.  For example:"
	 << endl;
    cout << "\tw020n10 e150s70, and the width and height are supplied"
	 << endl;
    cout << "\tin degrees (default: 10x10)." << endl;
    exit(-1);
}

void alloc_new_tile( int msgsock, long int next_tile, const string &status_dir ) {
    // cout << "new process started to handle new connection for "
    //      << next_tile << endl;

    // Read client's message (which is the status of the
    // last scenery creation task.)
    char buf[MAXBUF];
    int length;
    if ( (length = recv(msgsock, buf, MAXBUF, 0)) < 0) {
        perror("Cannot read command");
        exit(-1);
    }
    buf[length] = '\0';
    long int returned_tile = atoi(buf);
    cout << "client returned = " << returned_tile << endl;

    // record status
    if ( returned_tile < 0 ) {
	// failure
	log_failed_tile( status_dir, -returned_tile );
	log_finished_tile( status_dir, -returned_tile );
    } else {
	// success
	log_finished_tile( status_dir, returned_tile );
    }

    // reply to the client
    char message[MAXBUF];
    sprintf(message, "%ld", next_tile);
    length = strlen(message);
    if ( send(msgsock, message, length, 0) < 0 ) {
	perror("Cannot write to stream socket");
    }
#ifdef _MSC_VER
    closesocket(msgsock);
#else
    close(msgsock);
#endif
    // cout << "process for " << next_tile << " ended" << endl;
}

#ifdef _MSC_VER

struct Parameters {
    int msgsock;
    long next_tile;
    string status_dir;
};

DWORD WINAPI
ThreadProc(void* p)
{
    DWORD pN1;
    ULONG_PTR pN2;
    OVERLAPPED*	pOverLapped;

    while( GetQueuedCompletionStatus(gIoPort, &pN1, &pN2, &pOverLapped, INFINITE)) {
	if (pOverLapped == (OVERLAPPED*)0xFFFFFFFF)
		break;

	Parameters *p = (Parameters*)pN1;

	if ( p ) {
	    alloc_new_tile( p->msgsock, p->next_tile, p->status_dir );
	    delete p;
	}
    }
    return 1;
}

#endif


int main( int argc, char **argv ) {
    int arg_counter;
    long int next_tile;
    int sock, msgsock;
    fd_set ready;
    short unsigned int port;

#ifdef _MSC_VER
    gIoPort = CreateIoCompletionPort((HANDLE)INVALID_HANDLE_VALUE, NULL, 0, 0);

    for (int n = 0; n < numWorkerThreads; ++n) {
	DWORD id;
	gWorkerThreads[n] = CreateThread(NULL, 0, ThreadProc, gIoPort, 0, &id);
    }
#endif
				// Get any options first
    int arg_offset = 0;
    for (int i = 1; i < argc; i++) {
      string opt = argv[i];
      if (opt.find("--width=") == 0) {
	area_width = atof(opt.substr(8).c_str());
	arg_offset++;
      } else if (opt.find("--height=") == 0) {
	area_height = atof(opt.substr(9).c_str());
	arg_offset++;
      } else if (opt == "--") {
	break;
      } else if (opt.find("-") == 0) {
	cerr << "Unrecognized argument: " << opt << endl;
	usage(argv[0]);
      }
    }

    // quick argument sanity check
    if ( (argc - arg_offset) < 4 ) {
	usage( argv[0] );
    }

    string work_base = argv[arg_offset + 1];
    string output_base = argv[arg_offset + 2];

    cout << "Work base: " << work_base << endl;
    cout << "Output base: " << output_base << endl;
    cout << "Area width: " << area_width << " degrees" << endl;
    cout << "Area height: " << area_height << " degrees" << endl;

    arg_counter = arg_offset + 3;

    // initialize tile counter / incrementer
    init_tile_count( argv[arg_counter++] );

    // temp test
    // while ( (next_tile = get_next_tile()) != -1 ) {
    //     cout << next_tile << " " << SGBucket(next_tile) << endl;
    // }
    // cout << "done" << endl;
    // exit(0);

    // create the status directory
    string status_dir = work_base + "/Status";
    SGPath sgp( status_dir );
    sgp.append( "dummy" );
    sgp.create_dir( 0755 );

    // setup socket to listen on
    sock = make_socket( &port );
    cout << "socket is connected to port = " << port << endl;

    // Specify the maximum length of the connection queue
    listen(sock, 10);

    for ( ;; ) {
	FD_ZERO(&ready);
	FD_SET(sock, &ready);

	// block until we get some input on sock
	select(sock+1, &ready, 0, 0, NULL);

	if ( FD_ISSET(sock, &ready) ) {
	    // printf("%d %d Incomming message --> ", getpid(), pid);

	    // get the next tile to work on
	    next_tile = get_next_tile();

	    if ( next_tile == -1 ) {
		// end of chunk see if there are more chunks
		if ( arg_counter < argc ) {
		    // still more chunks to process
		    init_tile_count( argv[arg_counter++] );
		    next_tile = get_next_tile();
		}
	    }

	    cout << "Bucket = " << SGBucket(next_tile)
		 << " (" << pass << ")" << endl;

	    log_pending_tile( status_dir, next_tile );
	    // cout << "next tile = " << next_tile << endl;;

	    msgsock = accept(sock, 0, 0);
	    // cout << "msgsock = " << msgsock << endl;

#ifndef _MSC_VER
	    // spawn a child
	    int pid = fork();

	    if ( pid < 0 ) {
		// error
		perror("Cannot fork child process");
		exit(-1);
	    } else if ( pid > 0 ) {
		// This is the parent
		close(msgsock);

		// clean up all of our zombie children
		int status;
		while ( (pid = waitpid( WAIT_ANY, &status, WNOHANG )) > 0 ) {
		    // cout << "waitpid(): pid = " << pid
		    //      << " status = " << status << endl;
		}
	    } else {
		// This is the child
		close(sock);

		alloc_new_tile( msgsock, next_tile, status_dir );

		exit(0);
	    }
#else

	    Parameters *p = new Parameters;
	    p->msgsock = msgsock;
	    p->next_tile = next_tile;
	    p->status_dir = status_dir;
	    PostQueuedCompletionStatus(gIoPort, (DWORD)p, 0, NULL);

#endif
	}
    }
}

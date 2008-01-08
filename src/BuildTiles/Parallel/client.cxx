/* remote_exec.c -- Written by Curtis Olson */
/*               -- for CSci 5502 */


#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#ifdef HAVE_SYS_PARAM_H
#  include <sys/param.h>	// BSD macro definitions
#endif

#include <sys/time.h>		// FD_ISSET(), etc.
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <errno.h>

#include <utmp.h>

#include <stdio.h>
#include <stdlib.h>		// atoi()
#include <string.h>		// bcopy(), sterror()
#include <strings.h>		// bcopy() on Irix

#include <simgear/compiler.h>

#include STL_IOSTREAM
#include STL_STRING
#include <vector>

#include <simgear/bucket/newbucket.hxx>

SG_USING_STD(cout);
SG_USING_STD(cerr);
SG_USING_STD(endl);


#define MAXBUF 1024
#define BUSY_WAIT_TIME 30

SG_USING_STD(string);
SG_USING_STD(vector);

string work_base = ".";
string output_base = ".";
vector<string> load_dirs;
bool do_overwrite = true;


// check if it is ok to run
void check_master_switch() {
    string file = work_base + "/Status/MASTER_ON";

    if ( access( file.c_str(), F_OK ) != 0 ) {
	cout << "MASTER_ON file, " << file << " not found ... exiting." << endl;
	exit(0);
    }
}


// check if the host system is free of interactive users
int system_free() {

#if !defined(BSD) && !defined(__CYGWIN__)
    struct utmp *uptr;

    setutent();

    while ( (uptr = getutent()) != NULL ) {
	// cout << "NULL = " << NULL << "  uptr = " << uptr << endl;
	// cout << "user =  ut_user = " << uptr->ut_user << endl;
	// cout << "user =  ut_type = " << uptr->ut_type << endl;
	if (uptr->ut_type == USER_PROCESS) {
	    // found someone
	    endutent();
	    return 0;
	}
    }

    endutent();
#else
#  warning Port me
#endif

    return 1;
}


int make_socket (char *host, unsigned short int port) {
    int sock;
    struct sockaddr_in name;
    struct hostent *hp;
     
    // Create the socket.
    sock = socket (PF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
	perror ("socket");
	exit (EXIT_FAILURE);
    }
     
    // specify address family
    name.sin_family = AF_INET;

    // get the hosts official name/info
    hp = gethostbyname(host);

    // Connect this socket to the host and the port specified on the
    // command line
    bcopy(hp->h_addr, (char*)&(name.sin_addr.s_addr), hp->h_length);
    name.sin_port = htons(port);

    if ( connect(sock, (struct sockaddr *) &name, 
		 sizeof(struct sockaddr_in)) < 0 )
    {
	close(sock);
	perror("Cannot connect to stream socket");
	return -1;
    }

    return sock;
}


// connect to the server and get the next task
long int get_next_task( const string& host, int port, long int last_tile ) {
    long int tile;
    int sock, len;
    fd_set ready;
    char message[256];

    // loop till we get a socket connection
    while ( (sock = make_socket( (char *)host.c_str(), port )) < 0 ) {
	// check if the master switch is on
	check_master_switch();

	sleep(1);
    }

    // build a command string from the argv[]'s
    sprintf(message, "%ld", last_tile);

    // send command and arguments to remote server
    if ( write(sock, message, sizeof(message)) < 0 ) {
        perror("Cannot write to stream socket");
    }

    // loop until remote program finishes
    cout << "querying server for next task ..." << endl;

    FD_ZERO(&ready);
    FD_SET(sock, &ready);

    // block until input from sock
    select(sock+1, &ready, 0, 0, NULL);
    cout << " received reply" << endl;

    if ( FD_ISSET(sock, &ready) ) {
	/* input coming from socket */
	if ( (len = read(sock, message, 1024)) > 0 ) {
	    message[len] = '\0';
	    tile = atoi(message);
	    cout << "  tile to construct = " << tile << endl;
	    close(sock);
	    return tile;
	} else {
	    close(sock);
	    return -1;
	}
    }

    close(sock);
    return -1;
}

bool endswith(const char* s, const char* suffix) {
        size_t slen=strlen(s);
        size_t sufflen=strlen(suffix);
        if (slen<sufflen)
                return false;
        return (strncmp(s+slen-sufflen,suffix,sufflen)==0);
}

// check if the tile really has to be generated
static bool must_generate( const SGBucket& b ) {
    if (do_overwrite)
            return true;
    
    string btg_file = output_base + "/" + b.gen_base_path()
	+ "/" + b.gen_index_str() + ".btg.gz";
    string stg_file = output_base + "/" + b.gen_base_path()
	+ "/" + b.gen_index_str() + ".stg";
    struct stat btg_stat, stg_stat;
    bool have_btg, have_stg;
    
    have_btg= ( stat( btg_file.c_str(), &btg_stat ) == 0 );
    have_stg = ( stat( stg_file.c_str(), &stg_stat ) == 0 );
    
    if ( !have_btg ) {
            cout << "Output file " << btg_file << " was not found\n";
    }
    
    if ( !have_stg ) {
            cout << "Output file " << stg_file << " was not found\n";
    }
    
    /* Now check the load dirs for any source data and
     * whether any of that is newer than the btg-file or the stg-file
     */
    const string& prefix=b.gen_index_str()+".";
    size_t prefix_len=prefix.size();
    for (int i = 0; i < (int)load_dirs.size(); i++) {
            string path=load_dirs[i]+"/"+b.gen_base_path();
            DIR *loaddir=opendir(path.c_str());
            if (!loaddir) {
                    if (errno!=ENOENT)
                            cout << " Could not open load directory " << path << ":" << strerror(errno) << "\n";
                    continue;
            }
            
            struct dirent* de;
            struct stat src_stat;
            
            while ((de=readdir(loaddir))) {
                    if (strncmp(de->d_name,prefix.c_str(),prefix_len))
                            continue;
                    string file=path+"/"+de->d_name;
                    if ( stat(file.c_str(), &src_stat) != 0) {
                            /* huh!? */
                            cerr << " Could not stat file " << file << ":" << strerror(errno) << "\n";
                            continue;
                    }
                    if ( have_btg && src_stat.st_mtime>btg_stat.st_mtime ) {
                            cout << " File " << file << " is newer than btg-file => rebuild\n";
			    closedir(loaddir);
                            return true;
                    }
                    if ( have_stg && src_stat.st_mtime>stg_stat.st_mtime ) {
                            cout << " File " << file << " is newer than stg-file => rebuild\n";
			    closedir(loaddir);
                            return true;
                    }
                    /* Ignore elevation data, as it is not used if we have no
                     * landmass data. So in addition to elevation data we need at
                     * least one polygon file.
                     */
                    if ( endswith( de->d_name, ".arr.gz" ) || endswith( de->d_name, ".fit.gz" ) )
                            continue;
                    if ( !(have_stg && have_btg) ) {
                            cout << " There is source-data (" << file << ") for tile " << b.gen_index_str() << " but .btg or .stg is missing => build\n";
			    closedir(loaddir);
                            return true;
                    }
            }
            
            closedir(loaddir);
    }
    
    return false;
}


// build the specified tile, return true if contruction completed
// successfully
bool construct_tile( const SGBucket& b,
		     const string& result_file,
		     const string &cover ) {

    string command = "fgfs-construct ";
    command = command + " --work-dir=" + work_base;
    command = command + " --output-dir=" + output_base;
    command = command + " --tile-id=" + b.gen_index_str();
    if ( cover.size() > 0 ) {
        command = command + " --cover=" + cover;
    }
    for (int i = 0; i < (int)load_dirs.size(); i++) {
        command = command + " " + load_dirs[i];
    }
    command = command + " > " + result_file + " 2>&1";
    cout << command << endl;
	
    system( command.c_str() );
	
    FILE *fp = fopen( result_file.c_str(), "r" );
    if ( fp == NULL) {
	cout << "Missing results file " << result_file << endl;
	return false;
    }
    char line[256];
    while ( fgets( line, 256, fp ) != NULL ) {
        string line_str = line;
        line_str = line_str.substr(0, line_str.length() - 1);
        // cout << line_str << endl;
        if ( line_str == "[Finished successfully]" ) {
            cout << "Tile " << b.gen_index_str() << " finished successfully" << endl;
            fclose(fp);
            return true;
        }

    }
    fclose(fp);
    
    // Save the log file of the failed tile
    cout << "Tile " << b.gen_index_str() << " failed" << endl;
    string savelog=work_base+"/Status/failed-"+b.gen_index_str()+".log";
    
    command="mv "+result_file+" " +savelog;
    cout << command << endl;
    
    system(command.c_str());
    
    return false;
}


void
usage (const string name)
{
  cout << "Usage: " << name << endl;
  cout << "[ --output-dir=<directory>" << endl;
  cout << "  --work-dir=<directory>" << endl;
  cout << "  --host=<address>" << endl;
  cout << "  --port=<number>" << endl;
  cout << "  --rude" << endl;
  cout << "  --no-overwrite" << endl;
  cout << "  --cover=<landcover-raster>" << endl;
  cout << "<load directory...>" << endl;
  exit(-1);
}

int main(int argc, char *argv[]) {
    long int tile, last_tile;
    bool rude = false;
    bool result;

    string cover;
    string host = "127.0.0.1";
    int port=4001;

    //
    // Parse the command-line arguments.
    //
    int arg_pos;
    for (arg_pos = 1; arg_pos < argc; arg_pos++) {
      string arg = argv[arg_pos];

      if (arg.find("--output-dir=") == 0) {
	output_base = arg.substr(13);
      } else if (arg.find("--work-dir=") == 0) {
	work_base = arg.substr(11);
      } else if (arg.find("--host=") == 0) {
	host = arg.substr(7);
      } else if (arg.find("--port=") == 0) {
	port = atoi(arg.substr(7).c_str());
      } else if (arg == "--rude") {
	rude = true;
      } else if (arg == "--no-overwrite") {
	do_overwrite = false; 
      } else if (arg.find("--cover=") == 0) {
	cover = arg.substr(8);
      } else if (arg.find("--") == 0) {
	usage(argv[0]);
      } else {
	break;
      }
    }

    cout << "Output directory is " << output_base << endl;
    cout << "Working directory is " << work_base << endl;
    cout << "Server host is " << host << endl;
    cout << "Server port is " << port << endl;
    if (rude)
      cout << "Running in rude mode" << endl;
    else
      cout << "Running in polite mode" << endl;
    if (do_overwrite)
      cout << "Will overwrite existing buckets" << endl;
    else
      cout << "Will not overwrite up-to-date existing buckets" << endl;
    for (int i = arg_pos; i < argc; i++) {
      string dir;
      dir = work_base + "/";
      dir += argv[i];
      load_dirs.push_back( dir );
      cout << "Load directory: " << dir << endl;
    }

    // get hostname and pid
    char hostname[MAXBUF];
    gethostname( hostname, MAXBUF );
    pid_t pid = getpid();

    char tmp[MAXBUF];
    sprintf(tmp, "/tmp/result.%s.%d", hostname, pid);
    string result_file = tmp;

    last_tile = 0;

    // check if the master switch is on
    check_master_switch();

    while ( (tile = get_next_task( host, port, last_tile )) >= 0 ) {
        SGBucket bucket(tile);
	if (!must_generate(bucket)) {
	    cout << "No need to build tile " << tile << "\n";
	    result=true;
	} else {
	    result=construct_tile( bucket, result_file, cover );
	}
	if ( result ) {
	    last_tile = tile;
	} else {
            cout << "Build of tile " << tile << " failed\n";
	    last_tile = -tile;
	}

	// check if the master switch is on
	check_master_switch();

	// niceness policy: This whole process should run niced.  But
	// additionally, if there is interactive use, we will sleep
	// for 60 seconds between each tile to stagger out the load
	// and impose less of an impact on the machine.
	if ( !system_free() && !rude) {
	    cout << "System has interactive use, sleeping for " 
		 << BUSY_WAIT_TIME << " seconds..." << endl;
	    sleep( BUSY_WAIT_TIME );
	}
    }

    return 0;
}

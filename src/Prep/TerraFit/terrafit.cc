/*
 * terrafit.cc - Use Terra as a faster ArrayFit
 * (see http://graphics.cs.uiuc.edu/~garland/software/terra.html)
 *
 * Written by Ralf Gerlich, started December 2007
 * Based on terrafit.py by Norman Vine
 *      and Terra by Michael Garland
 *
 * Copyright (C) 2007  Ralf Gerlich - http://www.custom-scenery.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <iostream>
#include <string>

#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <errno.h>
#include <getopt.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <zlib.h>

#include <simgear/debug/logstream.hxx>
#include <simgear/bucket/newbucket.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/structure/exception.hxx>

#include <Array/array.hxx>

#include <Prep/Terra/GreedyInsert.h>
#include <Prep/Terra/Map.h>
#include <Prep/Terra/Mask.h>

/*
 * Benchmark: Processing 800 individual buckets:
 * terrafit.cc: 52s 48s 48s
 * terrafit.py: 217s 219s 223s
 *
 * terrafit.cc takes on 20% of the time that terrafit.py took!
 */
class ArrayMap: public Terra::Map {
public:
        ArrayMap(TGArray& array): array(array) {
                width=array.get_cols();
                height=array.get_rows();
                min=max=eval(0,0);
                for (int i=0;i<width;i++) {
                        for (int j=0;j<width;j++) {
                                Terra::real v=eval(i,j);
                                if (v<min)
                                        min=v;
                                if (v>max)
                                        max=v;
                        }
                }
                depth=32;
        }
        
        virtual Terra::real eval(int i, int j) {
                return (Terra::real)array.get_array_elev(i,j);
        }
        
        /* No direct reading of .arr.gz files */
        virtual void rawRead(istream&) {
        }
        virtual void textRead(istream&) {
        }
protected:
        TGArray& array;
};

static Terra::ImportMask default_mask;
namespace Terra {
/* GreedyInsertion requires us to declare a mask, even if we
 * don't need one...
 */
static Terra::ImportMask default_mask;
Terra::ImportMask *MASK=&default_mask;
}; // namespace Terra

Terra::real error_threshold=40.0;
unsigned int min_points=50;
unsigned int point_limit=1000;

inline int goal_not_met(Terra::GreedySubdivision* mesh)
{
    return
        ( mesh->maxError() > error_threshold &&
          mesh->pointCount() < point_limit ) ||
        mesh->pointCount() < min_points;
        
}

static void announce_goal(Terra::GreedySubdivision* mesh)
{
    SG_LOG(SG_GENERAL, SG_INFO, "Goal conditions met:");
    SG_LOG(SG_GENERAL, SG_INFO, "     error=" << mesh->maxError() << " [thresh="<< error_threshold << "]");
    SG_LOG(SG_GENERAL, SG_INFO, "     points=" << mesh->pointCount() << " [limit=" << point_limit << "]");
}

void greedy_insertion(Terra::GreedySubdivision* mesh)
{

    while( goal_not_met(mesh) )
    {
	if( !mesh->greedyInsert() )
	    break;
    }

    announce_goal(mesh);
}

bool endswith(const std::string& s1, const std::string& suffix) {
        size_t s1len=s1.size();
        size_t sufflen=suffix.size();
        if (s1len<sufflen)
                return false;
        return s1.compare(s1len-sufflen,sufflen,suffix)==0;
}

void fit_file(const std::string& path) {
        SG_LOG(SG_GENERAL, SG_INFO,"Working on file '" << path << "'");
        
        std::string infile,outfile;
        if (endswith(path,".arr.gz")) {
                infile=path.substr(0,path.size()-7);
        } else if (endswith(path,".arr")) {
                infile=path.substr(0,path.size()-4);
        } else {
                /* actually should not happen */
                SG_LOG(SG_GENERAL, SG_ALERT, "unknown input file extension = " << path);
                throw sg_exception("unknown input file extension!");
        }
        
        outfile=infile+".fit.gz";
        
        struct stat statbuf;
        if (stat(path.c_str(),&statbuf)!=0) {
                SG_LOG(SG_GENERAL, SG_ALERT ,"ERROR: Unable to stat '" << path << "':" << strerror(errno));
                return;
        }
        
        time_t src_mtime=statbuf.st_mtime;
        
        if (stat(outfile.c_str(),&statbuf)==0 && statbuf.st_mtime>src_mtime) {
                SG_LOG(SG_GENERAL, SG_INFO ,"Skipping " << outfile << ", source " << path << " is older");
                return;
        }
        
        SGBucket bucket(0,0); // dummy bucket
        TGArray inarray(infile);
        inarray.parse(bucket);
        
        ArrayMap *DEM=new ArrayMap(inarray);
        
        Terra::GreedySubdivision *mesh;
        
        mesh=new Terra::GreedySubdivision(DEM);
        
        greedy_insertion(mesh);
        
        gzFile fp;
        if ( (fp = gzopen( outfile.c_str(), "wb9" )) == NULL ) {
                SG_LOG(SG_GENERAL, SG_ALERT, "ERROR: opening " << outfile << " for writing!");
                return;
        }
        
        for (int x=0;x<DEM->width;x++) {
                for (int y=0;y<DEM->height;y++) {
                        if (mesh->is_used(x,y) != DATA_POINT_USED)
                                continue;
                        double vx,vy,vz;
                        vx=(inarray.get_originx()+x*inarray.get_col_step())/3600.0;
                        vx=(inarray.get_originy()+y*inarray.get_row_step())/3600.0;
                        vz=DEM->eval(x,y);
                        gzprintf(fp,"%+03.8f %+02.8f %0.2f\n",vx,vy,vz);
                }
        }
        
        gzclose(fp);
}

void walk_path(const std::string& path) {
        struct stat statbuf;
        if (stat(path.c_str(),&statbuf)!=0) {
                SG_LOG(SG_GENERAL, SG_ALERT ,"ERROR: Unable to stat '" << path << "':" << strerror(errno));
                return;
        }
        
        if (endswith(path,".arr.gz") || endswith(path,".arr")) {
                fit_file(path);
        } else if (S_ISDIR(statbuf.st_mode)) {
                DIR* dir=opendir(path.c_str());
                if (!dir) {
                        SG_LOG(SG_GENERAL, SG_ALERT ,"ERROR: Unable to open directory '" << path << "':" << strerror(errno));
                        return;
                }
                
                struct dirent* dirent;
                
                while ((dirent=readdir(dir))) {
                        if (!strcmp(dirent->d_name,".") || !strcmp(dirent->d_name,"..")) {
                                continue; // skip . and ..
                        }
                        SGPath subpath(path);
                        subpath.append(dirent->d_name);
                        walk_path(subpath.str());
                }
                
                closedir(dir);
        }
}

void usage(char* progname, const std::string& msg) {
        if (msg.size()!=0)
                SG_LOG(SG_GENERAL,SG_ALERT, msg);
        SG_LOG(SG_GENERAL,SG_INFO, "Usage: " << progname << " [options] <file | path to walk>");
        SG_LOG(SG_GENERAL,SG_INFO, "\t -h | --help ");
        SG_LOG(SG_GENERAL,SG_INFO, "\t -m | --minnodes 50");
        SG_LOG(SG_GENERAL,SG_INFO, "\t -x | --maxnodes 1000");
        SG_LOG(SG_GENERAL,SG_INFO, "\t -e | --maxerror 40");
        SG_LOG(SG_GENERAL,SG_INFO, "\t -v | --version");
        SG_LOG(SG_GENERAL,SG_INFO, "");
        SG_LOG(SG_GENERAL,SG_INFO, "Algorithm will produce at least <minnodes> fitted nodes, but no");
        SG_LOG(SG_GENERAL,SG_INFO, "more than <maxnodes>.  Within that range, the algorithm will stop");
        SG_LOG(SG_GENERAL,SG_INFO, "if the maximum elevation error for any remaining point");
        SG_LOG(SG_GENERAL,SG_INFO, "drops below <maxerror> meters.");
        SG_LOG(SG_GENERAL,SG_INFO, "");
        SG_LOG(SG_GENERAL,SG_INFO, "Increasing the maxnodes value and/or decreasing maxerror");
        SG_LOG(SG_GENERAL,SG_INFO, "will produce a better surface approximation.");
        SG_LOG(SG_GENERAL,SG_INFO, "");
        SG_LOG(SG_GENERAL,SG_INFO, "The input file must be a .arr.gz file such as that produced");
        SG_LOG(SG_GENERAL,SG_INFO, "by demchop or hgtchop utils.");
        SG_LOG(SG_GENERAL,SG_INFO, "");
        SG_LOG(SG_GENERAL,SG_INFO, "**** NOTE ****:");
        SG_LOG(SG_GENERAL,SG_INFO, "If a directory is input all .arr.gz files in directory will be");
        SG_LOG(SG_GENERAL,SG_INFO, "processed recursively.");
        SG_LOG(SG_GENERAL,SG_INFO, "");
        SG_LOG(SG_GENERAL,SG_INFO, "The output file(s) is/are called .fit.gz and is simply a list of");
        SG_LOG(SG_GENERAL,SG_INFO, "from the resulting fitted surface nodes.  The user of the");
        SG_LOG(SG_GENERAL,SG_INFO, ".fit.gz file will need to retriangulate the surface.");
}

struct option options[]={
        {"help",no_argument,NULL,'h'},
        {"minnodes",required_argument,NULL,'m'},
        {"maxnodes",required_argument,NULL,'x'},
        {"maxerror",required_argument,NULL,'e'},
        {"version",no_argument,NULL,'v'},
        {NULL,0,NULL,0}
};

int main(int argc, char** argv) {
        sglog().setLogLevels( SG_ALL, SG_DEBUG );
        int option;
        
        while ((option=getopt_long(argc,argv,"",options,NULL))!=-1) {
                switch (option) {
                case 'h':
                        usage(argv[0],"");
                        break;
                case 'm':
                        min_points=atoi(optarg);
                        break;
                case 'x':
                        point_limit=atoi(optarg);
                        break;
                case 'e':
                        error_threshold=atof(optarg);
                        break;
                case 'v':
                        SG_LOG(SG_GENERAL,SG_INFO,argv[0] << " Version 1.0");
                        exit(0);
                        break;
                case '?':
                        usage(argv[0],std::string("Unknown option:")+(char)optopt);
                        exit(1);
                }
        }
        std::cout << min_points << " " << point_limit << " " << error_threshold << "\n";
        if (optind<argc) {
                while (optind<argc) {
                        walk_path(argv[optind++]);
                }
        } else {
                usage(argv[0],"Insufficient arguments");
                exit(1);
        }
}


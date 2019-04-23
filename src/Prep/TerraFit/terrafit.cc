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
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <chrono>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#ifndef _MSC_VER
#include <unistd.h>
#ifdef __APPLE__
#include <Prep/Terra/getopt.h>
#else
#include <getopt.h>
#endif
#else
#define S_ISDIR(a) ((a)&_S_IFDIR)
#include <Prep/Terra/getopt.h>
#include <windows.h>
#define sleep(x) Sleep(x * 1000)
#endif

#include <zlib.h>

#include <simgear/bucket/newbucket.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/misc/sg_dir.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/structure/exception.hxx>
#include <simgear/threads/SGQueue.hxx>
#include <simgear/threads/SGThread.hxx>

#include <Array/array.hxx>
#include <Include/version.h>
#include <Prep/Terra/GreedyInsert.h>
#include <Prep/Terra/Map.h>
#include <Prep/Terra/Mask.h>

using simgear::Dir;
using simgear::PathList;
using std::istream;

SGLockedQueue<SGPath> global_workQueue;

/*
 * Benchmark: Processing 800 individual buckets:
 * terrafit.cc: 52s 48s 48s
 * terrafit.py: 217s 219s 223s
 *
 * terrafit.cc takes on 20% of the time that terrafit.py took!
 */
class ArrayMap : public Terra::Map {
public:
    explicit ArrayMap(TGArray& array)
        : array(array)
    {
        depth = 32;
        width = array.get_cols();
        height = array.get_rows();

        min = 30000;
        max = -30000;
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                Terra::real v = eval(i, j);
                if (v < min)
                    min = v;
                if (v > max)
                    max = v;
            }
        }
    }

    virtual ~ArrayMap() {}

    virtual Terra::real eval(int i, int j) override
    {
        return (Terra::real)array.get_array_elev(i, j);
    }

    /* No direct reading of .arr.gz files */
    virtual void rawRead(istream&)
    {
    }
    virtual void textRead(istream&)
    {
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
Terra::ImportMask* MASK = &default_mask;
}; // namespace Terra

Terra::real error_threshold = 40.0;
unsigned int min_points = 50;
unsigned int point_limit = 1000;
bool force = false;
unsigned int num_threads = 1;

inline int goal_not_met(Terra::GreedySubdivision* mesh)
{
    return (mesh->maxError() > error_threshold && mesh->pointCount() < point_limit) || mesh->pointCount() < min_points;
}

static void announce_goal(Terra::GreedySubdivision* mesh)
{
    SG_LOG(SG_GENERAL, SG_INFO, "Goal conditions met:");
    SG_LOG(SG_GENERAL, SG_INFO, "     error=" << mesh->maxError() << " [thresh=" << error_threshold << "]");
    SG_LOG(SG_GENERAL, SG_INFO, "     points=" << mesh->pointCount() << " [limit=" << point_limit << "]");
}

void greedy_insertion(Terra::GreedySubdivision* mesh)
{
    while (goal_not_met(mesh)) {
        if (!mesh->greedyInsert())
            break;
    }

    announce_goal(mesh);
}

bool endswith(const std::string& s1, const std::string& suffix)
{
    size_t s1len = s1.size();
    size_t sufflen = suffix.size();

    if (s1len < sufflen) {
        return false;
    }

    return s1.compare(s1len - sufflen, sufflen, suffix) == 0;
}

void fit_file(const SGPath& path)
{
    SG_LOG(SG_GENERAL, SG_INFO, "Working on file '" << path << "'");

    SGPath outPath(path.dir());
    outPath.append(path.file_base() + ".fit.gz");
    if (outPath.exists()) {
        unlink(outPath.c_str());
    }

    SGBucket bucket; // dummy bucket
    TGArray inarray(path.dir() + "/" + path.file_base());
    inarray.parse(bucket);
    inarray.close();

    ArrayMap DEM(inarray);

    Terra::GreedySubdivision mesh(&DEM);

    greedy_insertion(&mesh);

    gzFile fp;
    if ((fp = gzopen(outPath.c_str(), "wb9")) == NULL) {
        SG_LOG(SG_GENERAL, SG_ALERT, "ERROR: opening " << outPath << " for writing!");
        return;
    }

    gzprintf(fp, "%d\n", mesh.pointCount());

    double origin_x = inarray.get_originx();
    double origin_y = inarray.get_originy();
    double col_step = inarray.get_col_step();
    double row_step = inarray.get_row_step();

    for (int x = 0; x < DEM.width; ++x) {
        double vx = (origin_x + x * col_step) / 3600.0;

        for (int y = 0; y < DEM.height; ++y) {
            if (mesh.is_used(x, y) != DATA_POINT_USED)
                continue;

            double vy = (origin_y + y * row_step) / 3600.0;
            double vz = DEM.eval(x, y);

            gzprintf(fp, "%+03.8f %+02.8f %0.2f\n", vx, vy, vz);
        }
    }

    gzclose(fp);
}

void queue_fit_file(const SGPath& path)
{
    SGPath outPath(path.dir());
    outPath.append(path.file_base() + ".fit.gz");

    if (!force) {
        if (outPath.exists() && (path.modTime() < outPath.modTime())) {
            SG_LOG(SG_GENERAL, SG_INFO, "Skipping " << outPath << ", source " << path << " is older");
            return;
        }
    }

    global_workQueue.push(path);
}

class FitThread : public SGThread {
public:
    virtual void run()
    {
        while (!global_workQueue.empty()) {
            SGPath path = global_workQueue.pop();
            if (path.exists()) {
                fit_file(path);
            }
        }
    }
};

void walk_path(const SGPath& path)
{
    if (!path.exists()) {
        SG_LOG(SG_GENERAL, SG_ALERT, "ERROR: Unable to stat '" << path.str() << "':" << strerror(errno));
        return;
    }

    if ((path.lower_extension() == "arr") || (path.complete_lower_extension() == "arr.rectified.gz") || (path.complete_lower_extension() == "arr.gz")) {
        SG_LOG(SG_GENERAL, SG_DEBUG, "will queue " << path);
        queue_fit_file(path);
    } else if (path.isDir()) {
        Dir d(path);
        int flags = Dir::TYPE_DIR | Dir::TYPE_FILE | Dir::NO_DOT_OR_DOTDOT;
        for (const SGPath& p : d.children(flags)) {
            walk_path(p);
        }
    }
}

void usage(char* progname, const std::string& msg)
{
    if (msg.size() != 0) {
        SG_LOG(SG_GENERAL, SG_ALERT, msg);
    }

    SG_LOG(SG_GENERAL, SG_INFO, "Usage: " << progname << " [options] <file | path to walk>");
    SG_LOG(SG_GENERAL, SG_INFO, "\t -h | --help");
    SG_LOG(SG_GENERAL, SG_INFO, "\t -m | --minnodes 50");
    SG_LOG(SG_GENERAL, SG_INFO, "\t -x | --maxnodes 1000");
    SG_LOG(SG_GENERAL, SG_INFO, "\t -e | --maxerror 40");
    SG_LOG(SG_GENERAL, SG_INFO, "\t -f | --force");
    SG_LOG(SG_GENERAL, SG_INFO, "\t -j | --threads <number>");
    SG_LOG(SG_GENERAL, SG_INFO, "\t -v | --version");
    SG_LOG(SG_GENERAL, SG_INFO, "");
    SG_LOG(SG_GENERAL, SG_INFO, "Algorithm will produce at least <minnodes> fitted nodes, but no");
    SG_LOG(SG_GENERAL, SG_INFO, "more than <maxnodes>.  Within that range, the algorithm will stop");
    SG_LOG(SG_GENERAL, SG_INFO, "if the maximum elevation error for any remaining point");
    SG_LOG(SG_GENERAL, SG_INFO, "drops below <maxerror> meters.");
    SG_LOG(SG_GENERAL, SG_INFO, "");
    SG_LOG(SG_GENERAL, SG_INFO, "Increasing the maxnodes value and/or decreasing maxerror");
    SG_LOG(SG_GENERAL, SG_INFO, "will produce a better surface approximation.");
    SG_LOG(SG_GENERAL, SG_INFO, "");
    SG_LOG(SG_GENERAL, SG_INFO, "The input file must be a .arr.gz file such as that produced");
    SG_LOG(SG_GENERAL, SG_INFO, "by the hgtchop utility.");
    SG_LOG(SG_GENERAL, SG_INFO, "");
    SG_LOG(SG_GENERAL, SG_INFO, "Force will overwrite existing .arr.gz files, even if the input is older");
    SG_LOG(SG_GENERAL, SG_INFO, "");
    SG_LOG(SG_GENERAL, SG_INFO, "**** NOTE ****:");
    SG_LOG(SG_GENERAL, SG_INFO, "If a directory is input all .arr.gz files in directory will be");
    SG_LOG(SG_GENERAL, SG_INFO, "processed recursively.");
    SG_LOG(SG_GENERAL, SG_INFO, "");
    SG_LOG(SG_GENERAL, SG_INFO, "The output file(s) is/are called .fit.gz and is simply a list of");
    SG_LOG(SG_GENERAL, SG_INFO, "from the resulting fitted surface nodes.  The user of the");
    SG_LOG(SG_GENERAL, SG_INFO, ".fit.gz file will need to retriangulate the surface.");
}

struct option options[] = {
    { "help", no_argument, NULL, 'h' },
    { "minnodes", required_argument, NULL, 'm' },
    { "maxnodes", required_argument, NULL, 'x' },
    { "maxerror", required_argument, NULL, 'e' },
    { "force", no_argument, NULL, 'f' },
    { "version", no_argument, NULL, 'v' },
    { "threads", required_argument, NULL, 'j' },
    { NULL, 0, NULL, 0 }
};

int main(int argc, char** argv)
{
    sglog().setLogLevels(SG_ALL, SG_INFO);
    int option;

    auto start_time = std::chrono::high_resolution_clock::now();

    while ((option = getopt_long(argc, argv, "hm:x:e:fvj:", options, NULL)) != -1) {
        switch (option) {
        case 'h':
            usage(argv[0], "");
            break;
        case 'm':
            min_points = atoi(optarg);
            break;
        case 'x':
            point_limit = atoi(optarg);
            break;
        case 'e':
            error_threshold = atof(optarg);
            break;
        case 'f':
            force = true;
            break;
        case 'v':
            SG_LOG(SG_GENERAL, SG_INFO, argv[0] << " version " << getTGVersion());
            exit(0);
            break;
        case 'j':
            num_threads = atoi(optarg);
            break;
        case '?':
            usage(argv[0], std::string("Unknown option:") + (char)optopt);
            exit(1);
        }
    }

    SG_LOG(SG_GENERAL, SG_INFO, "TerraFit version " << getTGVersion() << " using " << num_threads << " threads");
    SG_LOG(SG_GENERAL, SG_INFO, "Min points = " << min_points);
    SG_LOG(SG_GENERAL, SG_INFO, "Max points = " << point_limit);
    SG_LOG(SG_GENERAL, SG_INFO, "Max error  = " << error_threshold);

    if (optind < argc) {
        while (optind < argc) {
            SG_LOG(SG_GENERAL, SG_INFO, "walking " << SGPath(argv[optind]));
            walk_path(SGPath(argv[optind++]));
        }
    } else {
        SG_LOG(SG_GENERAL, SG_INFO, "Use 'terrafit --help' for commands");
        exit(1);
    }

    std::vector<std::shared_ptr<FitThread>> threads;
    for (unsigned int t = 0; t < num_threads; ++t) {
        std::shared_ptr<FitThread> thread(new FitThread);
        thread->start();
        threads.push_back(thread);
    }

    while (!global_workQueue.empty()) {
        sleep(1);
    }

    for (unsigned int t = 0; t < num_threads; ++t) {
        threads[t]->join();
    }

    SG_LOG(SG_GENERAL, SG_INFO, "Work queue is empty\n");

    auto finish_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish_time - start_time;
    std::cout << std::endl
              << "Elapsed time: " << elapsed.count() << " seconds" << std::endl
              << std::endl;

    return EXIT_SUCCESS;
}

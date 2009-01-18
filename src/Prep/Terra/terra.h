#ifndef TERRA_INCLUDED // -*- C++ -*-
#define TERRA_INCLUDED

#include "GreedyInsert.h"
#include "Map.h"
#include "Mask.h"

#include <iostream>

#ifdef _MSC_VER
#define rint(x) floor((x) + 0.5)
#endif

namespace Terra {

extern GreedySubdivision *mesh;
extern Map *DEM;
extern ImportMask *MASK;

extern real error_threshold;
extern int min_points;
extern int point_limit;
extern real height_scale;
enum FileFormat {NULLfile, TINfile, EPSfile, DEMfile, OBJfile, RMSfile};
extern FileFormat output_format;
extern char *output_filename;
extern char *script_filename;

extern int goal_not_met();
extern void greedy_insertion();
extern void display_greedy_insertion(void (*callback)());
extern void scripted_preinsertion(std::istream&);
extern void subsample_insertion(int target_width);

extern void generate_output(char *filename=NULL,
			    FileFormat format=NULLfile);
extern void output_tin(std::ostream&);
extern void output_eps(std::ostream&);
extern void output_obj(std::ostream&);
extern void output_dem(std::ostream&);

extern void process_cmdline(int argc, char **argv);

}; // namespace Terra

#endif

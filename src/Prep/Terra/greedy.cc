#include "terra.h"

#include <iostream>
#include <cstdio> // for EOF

using std::istream;
using std::cerr;
using std::endl;

namespace Terra {

void scripted_preinsertion(istream& script)
{
    char op[4];
    int x, y;

    while( script.peek() != EOF )
    {
	script >> op >> x >> y;

	switch( op[0] )
	{
	case 's':
	    if( !mesh->is_used(x, y) )
	    {
		mesh->select(x, y);
		mesh->is_used(x, y) = DATA_POINT_USED;
	    }
	    break;

	case 'i':
	    if( !mesh->is_used(x,y) )
		mesh->is_used(x, y) = DATA_POINT_IGNORED;
	    break;

	case 'u':
	    if( !mesh->is_used(x,y) )
		mesh->is_used(x, y) = DATA_VALUE_UNKNOWN;
	    break;

	default:
	    break;
	}
    }
}

void subsample_insertion(int target_width)
{
    int width = DEM->width;
    int height = DEM->height;

    // 'i' is the target width and 'j' is the target height

    real i = (real)target_width;
    real j = rint((i*height) / width);

    real dx = (width-1)/(i-1);
    real dy = (height-1)/(j-1);

    real u, v;
    int x, y;
    for(u=0; u<i; u++)
	for(v=0; v<j; v++)
	{
	    x =  (int)rint(u*dx);
	    y =  (int)rint(v*dy);

	    if( !mesh->is_used(x,y) )
		mesh->select(x, y);
	}
}

inline int goal_not_met()
{
    return
        ( mesh->maxError() > error_threshold &&
          (int)mesh->pointCount() < point_limit ) ||
          (int)mesh->pointCount() < min_points;
        
}

static void announce_goal()
{
    cerr << "Goal conditions met:" << endl;
    cerr << "     error=" << mesh->maxError()
	 << " [thresh="<< error_threshold << "]" << endl;
    cerr << "     points=" << mesh->pointCount()
	 << " [limit=" << point_limit << "]" << endl;
}

void greedy_insertion()
{

    while( goal_not_met() )
    {
	if( !mesh->greedyInsert() )
	    break;
    }

    announce_goal();
}

void display_greedy_insertion(void (*callback)())
{

    while( goal_not_met() )
    {
	if( !mesh->greedyInsert() )
	{
	    (*callback)();
	    break;
	}
	(*callback)();
    }

    announce_goal();
}

}; // namespace Terra

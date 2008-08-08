#include <math.h>
#include <stdlib.h>
#include <iostream>

#include "Geom.h"
#include "Mask.h"

using std::istream;
using std::cerr;
using std::endl;

namespace Terra {

RealMask *readMask(istream& in)
{
    char magicP, magicNum;

    int width, height, maxval;

    in >> magicP >> magicNum;
    in >> width >> height >> maxval;

    if( magicP != 'P' )
    {
	cerr << "readMask: This is not PGM data." << endl;
	return NULL;
    }

    RealMask *mask = new RealMask(width, height);

    if( magicNum == '2' )
    {
	for(int j=0; j<height; j++)
	    for(int i=0; i<width; i++)
	    {
		real val;
		in >> val;
		mask->ref(i, j) = val;
	    }
    }
    else if( magicNum == '5' )
    {
	for(int j=0; j<height; j++)
	    for(int i=0; i<width; i++)
	    {
		unsigned char val;
		in >> val;
		mask->ref(i, j) = (real)val;
	    }
    }
    else
    {
	cerr << "readMask: This is not PGM data." << endl;
	return NULL;
    }


    real max = (real)maxval;

    for(int i=0; i<width; i++)
	for(int j=0; j<height; j++)
	    mask->ref(i,j) /= max;

    return mask;
}

}; // namespace Terra

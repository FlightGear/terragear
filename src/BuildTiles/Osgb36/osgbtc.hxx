#ifndef OSBTEXCOORD_H
#define OSBTEXCOORD_H


#include <simgear/bucket/newbucket.hxx>
#include <simgear/math/point3d.hxx>
#include <simgear/math/sg_types.hxx>


#define FG_STANDARD_TEXTURE_DIMENSION 1000.0 // meters
#define MAX_TEX_COORD 8.0
#define HALF_MAX_TEX_COORD ( MAX_TEX_COORD / 2.0 )


// return the basic unshifted/unmoded texture coordinate for a Easting/Northing (UK) grid reference
inline Point3D UK_basic_tex_coord( const Point3D& p )
{
//    cout << "Point in dcl_basic_tex_coord is " << p << '\n';
    return Point3D( p.x() / FG_STANDARD_TEXTURE_DIMENSION ,
		    p.y() / FG_STANDARD_TEXTURE_DIMENSION ,
		    0.0 );

}


// traverse the specified fan/strip/list of vertices and attempt to
// calculate "none stretching" texture coordinates
point_list UK_calc_tex_coords( const SGBucket& b, const point_list& geod_nodes,
			    const int_list& fan, double scale );

#endif // OSBTEXCOORD_H

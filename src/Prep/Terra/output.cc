#include <simgear/compiler.h>

#include <fstream>
#include <streambuf>

#include "terra.h"

using std::cerr;
using std::cout;
using std::endl;
using std::istream;
using std::ostream;
using std::ofstream;
using std::streampos;

namespace Terra {

void generate_output(char *filename, FileFormat format)
{
    if( !filename )
	filename = output_filename;
    if( format==NULLfile )
	format = output_format;

    //
    // If the user doesn't want output, don't do it.
    if( !filename )
	return;

    ofstream out(filename);
    switch( format )
    {
    case TINfile:
	output_tin(out);
	break;

    case EPSfile:
	output_eps(out);
	break;

    case DEMfile:
	output_dem(out);
	break;

    case OBJfile:
	output_obj(out);
	break;

    case RMSfile:
	cout << mesh->pointCount() << "\t\t" << mesh->rmsError() << endl;
	break;

    default:
	// Do nothing for unknown formats
	break;
    }
    out.close();
}




static ostream *output_stream;


static void obj_face(Triangle& T, void *closure)
{
    ostream& out = *output_stream;
    array2<int>& vert_id = *(array2<int> *)closure;

    const Vec2& p1 = T.point1();
    const Vec2& p2 = T.point2();
    const Vec2& p3 = T.point3();

    out << "f ";
    out << vert_id((int)p1[X], (int)p1[Y]) << " ";
    out << vert_id((int)p2[X], (int)p2[Y]) << " ";
    out << vert_id((int)p3[X], (int)p3[Y]) << endl;
}

static void obj_vertex(ostream& out, int x,int y)
{
    out << "v " << x <<" "<< y <<" "<< DEM->eval(x,y) << endl;
}

void output_obj(ostream& out)
{
    output_stream = &out;
    int width = DEM->width;
    int height = DEM->height;

    array2<int> vert_id(width, height);
    int index = 1;

    for(int x=0;x<width;x++)
	for(int y=0;y<height;y++)
	{
	    if( mesh->is_used(x,y) == DATA_POINT_USED )
	    {
		vert_id(x,y) = index++;
		obj_vertex(out, x,y);
	    }
	    else
		vert_id(x,y) = 0;
	}

    mesh->overFaces(obj_face, &vert_id);
}




static void tin_face(Triangle& T, void *)
{
    ostream& out = *output_stream;

    const Vec2& p1 = T.point1();
    const Vec2& p2 = T.point2();
    const Vec2& p3 = T.point3();

    out << "t ";

    out << p1[X] <<" "<< p1[Y] <<" "<< DEM->eval(p1[X],p1[Y]) <<" ";
    out << p2[X] <<" "<< p2[Y] <<" "<< DEM->eval(p2[X],p2[Y]) <<" ";
    out << p3[X] <<" "<< p3[Y] <<" "<< DEM->eval(p3[X],p3[Y]) << endl;
}

void output_tin(ostream& out)
{
    output_stream = &out;

    mesh->overFaces(tin_face);
}


static void eps_prologue(ostream& out)
{
    out << "%!PS-Adobe-2.0 EPSF-2.0" << endl;
    out << "%%Creator: TERRA" << endl;
    out << "%%BoundingBox: 0 0 " << DEM->width-1 <<" "<< DEM->height-1 << endl;
    out << "%%EndComments" << endl;

    out << endl;
    out << "% -- The following transformation is a hack to change" << endl;
    out << "% -- from image coordinates to page coordinates" << endl;
    out << "% -- (in other words, flip things upside down)" << endl;
    out << "1 -1 scale" << endl;
    out << "0 -" << DEM->height-1 << " translate" << endl;
    out << endl;

    out << "/L {moveto lineto stroke} bind def" << endl;
    out << "/C {newpath 2.5 0 360 arc closepath fill} bind def" << endl;
}

static void ps_edge(Edge *e, void *)
{
    ostream& out = *output_stream;

    const Vec2& a = e->Org();
    const Vec2& b = e->Dest();

    out << a[X] << " " << a[Y] << " "
	<< b[X] << " " << b[Y]
	<< " L" << endl;
}

void output_eps(ostream& out)
{
    eps_prologue(out);

    output_stream = &out;

    out << ".2 setlinewidth" << endl;
    mesh->overEdges(ps_edge);
    out << "showpage" << endl;
}

void output_dem(ostream& out)
{
    cerr << ">> Writing approximate DEM output ... ";
    cerr << "this may take a couple minutes." << endl;

    int width = DEM->width;
    int height = DEM->height;

    out << "P2 " << width << " " << height << " " << DEM->max << endl;

    streampos start = out.tellp();

    for(int j=0;j<height;j++)
	for(int i=0;i<width;i++)
	{
	    out << rint(mesh->eval(i,j));

	    if( out.tellp() - start < 60 )
		out << " ";
	    else
	    {
		out << endl;
		start = out.tellp();
	    }
	}
    out << endl;
    cerr << ">> Done." << endl;
}

}; // namespace Terra

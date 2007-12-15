#include <iostream>
#include <fstream>

#include <GL/glut.h>
#include "glHacks.h"

#include "terra.h"
#include "gui.h"

using std::cout;
using std::endl;

namespace Terra {

int mesh_view;
int surf_view;

int will_draw_dem = False;


// Prototype for our hack below.
//
void xglutKeepAspect(float width, float height);


////////////////////////////////////////////////////////////////////////
//
// Mesh functions
//
////////////////////////////////////////////////////////////////////////
#define MESH_MENU_EPS 1000
#define MESH_MENU_TIN 1001
#define MESH_MENU_OBJ 1002
#define MESH_MENU_DEM 1003

static void mesh_main_menu(int value)
{
    switch( value )
    {
    case MESH_MENU_EPS:
	generate_output("out.eps", EPSfile);
	break;

    case MESH_MENU_TIN:
	generate_output("out.tin", TINfile);
	break;

    case MESH_MENU_OBJ:
	generate_output("out.obj", OBJfile);
	break;

    case MESH_MENU_DEM:
	generate_output("out.pgm", DEMfile);

    default:
	break;
    }

}

static void mesh_toggle_menu(int value)
{
    int *val = (int *)value;

    *val = !(*val);

    glutPostRedisplay();
}



static void create_mesh_menus()
{
    int toggle = glutCreateMenu(mesh_toggle_menu);

    glutAddMenuEntry("Draw DEM data", (int)&will_draw_dem);


    int main = glutCreateMenu(mesh_main_menu);
    glutAddSubMenu("Toggle", toggle);

    glutAddMenuEntry("Output Mesh EPS", MESH_MENU_EPS);
    glutAddMenuEntry("Output Surface TIN", MESH_MENU_TIN);
    glutAddMenuEntry("Output Surface OBJ", MESH_MENU_OBJ);
    glutAddMenuEntry("Output Approximate DEM", MESH_MENU_DEM);

    glutAttachMenu(GLUT_RIGHT_BUTTON);
}



static void draw_edge(Edge *e, void *)
{
    Vec2& org = e->Org();
    Vec2& dest = e->Dest();
    
    glV(org[X], org[Y]);
    glV(dest[X], dest[Y]);
}

static void draw_dem()
{
    Map *bits = DEM;

    glPixelZoom((float)glutGet((GLenum)GLUT_WINDOW_WIDTH) / (float)DEM->width ,
		-(float)glutGet((GLenum)GLUT_WINDOW_HEIGHT) / (float)DEM->height);
    glRasterPos3f(0, 0, 0);


    static GLenum type_map[] = {GL_UNSIGNED_BYTE,
				GL_UNSIGNED_SHORT,
				GL_UNSIGNED_INT };
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    float scale = (float)((1<<bits->depth) - 1) / (DEM->max - DEM->min);
    glPixelTransferf(GL_RED_SCALE, scale);
    glPixelTransferf(GL_GREEN_SCALE, scale);
    glPixelTransferf(GL_BLUE_SCALE, scale);

    float bias  = -DEM->min/(DEM->max - DEM->min);
    glPixelTransferf(GL_RED_BIAS, bias);
    glPixelTransferf(GL_GREEN_BIAS, bias);
    glPixelTransferf(GL_BLUE_BIAS, bias);

    //!!
    // WARNING:
    // We are assuming that the map will never be a RealMap
    //
    GLenum type = type_map[((bits->depth)>>3)-1];
    glDrawPixels(bits->width, bits->height,
		 GL_LUMINANCE,
		 type,
		 bits->getBlock());
}

void mesh_display()
{
    glClear( GL_COLOR_BUFFER_BIT );

    if( will_draw_dem )
	draw_dem();


    glBegin(GL_LINES);
      glC(1.0, 0.15, 0.15);
      mesh->overEdges(draw_edge);
    glEnd();
}

static inline void redisplay_all(int other)
{
    glutPostRedisplay();
    glutSetWindow(other);
    glutPostRedisplay();
}

void mesh_mouse(int button, int state, int x, int y)
{
    if( state == GLUT_DOWN )
    {
	if( button == GLUT_LEFT_BUTTON )
	{
	    GLdouble u, v, z;
	    int win_height = glutGet((GLenum)GLUT_WINDOW_HEIGHT);

	    glUnproject(x, win_height-y, 0, &u, &v, &z);
	    if( u<0 ) u = 0;
	    if( v<0 ) v = 0;
	    if( u>(DEM->width-1) ) u = DEM->width-1;
	    if( v>(DEM->height-1) ) v = DEM->height-1;

	    Vec2 p(u,v);
	    cout << "Inserting point: " << p << endl;

	    mesh->insert(p);
	    redisplay_all(surf_view);
	}
	else if( button == GLUT_MIDDLE_BUTTON )
	{
	    display_greedy_insertion(mesh_display);
	    redisplay_all(surf_view);
	}
    }
}





////////////////////////////////////////////////////////////////////////
//
// Surface functions
//
////////////////////////////////////////////////////////////////////////
static void synthesize_normal(const Vec3& p1, const Vec3& p2,const Vec3& p3)
{
    // We explicitly declare these (rather than putting them in a
    // Vector) so that they can be allocated into registers.
    real v1x = p2[X]-p1[X],
        v1y = p2[Y]-p1[Y],
        v1z = p2[Z]-p1[Z];
    real v2x = p3[X]-p2[X],
         v2y = p3[Y]-p2[Y],
         v2z = p3[Z]-p2[Z];

    //
    // NOTE: We do not unitize the normal vectors here because
    //       they will be normalized in the graphics pipeline.
    //       We have to enable GL_NORMALIZE because of the height
    //       scaling we're doing.  So why bother normalizing now
    //       when they're just going to get munged?
    //

    glN(v1y*v2z - v1z*v2y,
	v1z*v2x - v1x*v2z,
	v1x*v2y - v1y*v2x);
}

void render_face(Triangle& T, void *)
{
    const Vec2& p1 = T.point1();
    const Vec2& p2 = T.point2();
    const Vec2& p3 = T.point3();

    Vec3 v1(p1, DEM->eval(p1[X],p1[Y]));
    Vec3 v2(p2, DEM->eval(p2[X],p2[Y]));
    Vec3 v3(p3, DEM->eval(p3[X],p3[Y]));

    synthesize_normal(v1, v2, v3);

    glV(v1[X], v1[Y], v1[Z]);
    glV(v2[X], v2[Y], v2[Z]);
    glV(v3[X], v3[Y], v3[Z]);
}



static long statex, statey;
static enum {NoDrag, Spin, Dolly, Zoom } dragging = NoDrag;
static float rotx = 0, roty = 0, rotz = 0;

void surf_display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glPushMatrix();
      float w  = (float)DEM->width;
      float h  = (float)DEM->height;
      float cx = w / 2.0f;
      float cy = h / 2.0f;
      float cz = (float)DEM->min + (float)(DEM->max - DEM->min) / 2.0f;

      glRotatef(rotx, 1.0, 0.0, 0.0);
      glRotatef(roty, 0.0, 1.0, 0.0);
      glRotatef(rotz, 0.0, 0.0, 1.0);
      glScalef(1.0f, 1.0f, (float)height_scale);
      glTranslatef(-cx, -cy, -cz);


      glScalef(1.0f, -1.0f, 1.0f);
      glTranslatef(0.0f, 1-h, 0.0f);

      glBegin(GL_TRIANGLES);
        mesh->overFaces(render_face);
      glEnd();
    glPopMatrix();

    glutSwapBuffers();
}


void surf_mouse(int button, int state, int x, int y)
{
    if( state == GLUT_UP )
    {
	dragging = NoDrag;
	return;
    }

    statex = x;
    statey = y;

    switch( button )
    {
    case GLUT_LEFT_BUTTON:
	dragging = Spin;
	break;

    case GLUT_MIDDLE_BUTTON:
	dragging = Dolly;
	break;

    case GLUT_RIGHT_BUTTON:
	dragging = Zoom;
	break;
    }
}


void surf_motion(int x, int y)
{
    switch( dragging )
    {
    case Spin:
        rotx += (float)(y - statey);
        roty += (float)(x - statex);

        statex = x;
        statey = y;

        glutPostRedisplay();
	break;

    case Dolly:
	glMatrixMode(GL_PROJECTION);
	glTranslatef((float)(x - statex), (float)(statey - y), 0.0f);
        statex = x;
        statey = y;
	glMatrixMode(GL_MODELVIEW);

        glutPostRedisplay();
	break;

    case Zoom:
	glMatrixMode(GL_PROJECTION);
	glTranslatef(0, 0, (float)(y - statey));
        statey = y;
	glMatrixMode(GL_MODELVIEW);

        glutPostRedisplay();
	break;

    default:
	break;
    }
}


void init_surf_view(GreedySubdivision& mesh)
{
    glClearColor(0.3125, 0.3125, 1.0, 0.0);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);

    //////////////////////
    //
    // Define viewing parameters
    //
    Map& map = mesh.getData();
    real width = (real)map.width;
    real height = (real)map.height;
    real depth = map.max - map.min;

    xglutKeepAspect(width, height);

    glMatrixMode(GL_PROJECTION);
    gluPerspective(45.0,
		   width/height,
		   0.1,
		   10*depth);

    gluLookAt(0, 0, depth,
	      0, 0, 0,
	      0, 1, 0);

    glMatrixMode(GL_MODELVIEW);


    //////////////////////
    //
    // Define lighting parameters
    //
    glShadeModel(GL_SMOOTH);

    GLfloat mat_ambient[] = { 0.2, 0.2, 0.2, 1.0 };
    GLfloat mat_diffuse[] = { 0.6, 0.6, 0.6, 1.0 };
    // GLfloat mat_specular[] = { 0.5, 0.5, 0.5, 1.0 };

    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
    // glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);

    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 0);

    GLfloat light_pos[] = { 1.0, 0.0, 1.0, 0.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
}



void gui_init()
{
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    surf_view = glutCreateWindow("TERRA: Surface");
    init_surf_view(*mesh);

    glutDisplayFunc(surf_display);
    glutMouseFunc(surf_mouse);
    glutMotionFunc(surf_motion);

    // ---------------------------------------------------------------------

    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
    mesh_view = glutCreateWindow("TERRA: Mesh");

    xglutKeepAspect(DEM->width, DEM->height);

    create_mesh_menus();

    glutDisplayFunc(mesh_display);
    glutMouseFunc(mesh_mouse);

    glMatrixMode(GL_PROJECTION);
    gluOrtho2D(-1, DEM->width,DEM->height, -1);
    // gluOrtho2D(0, DEM->width-1, DEM->height-1, 0);
    glMatrixMode(GL_MODELVIEW);
    glClearColor(0.0, 0.0, 0.0, 0.0);
}

void gui_interact()
{
    glutMainLoop();
}



////////////////////////////////////////////////////////////////////////
//
// WARNING: Entering hack zone!
//
// GLUT does not provide a function for setting the aspect
// ratio of a window.  We want to do this.  Hence the following.
//
////////////////////////////////////////////////////////////////////////

#if 0
extern "C" {
#include <GL/glutint.h>
}
#endif

void xglutKeepAspect(float width, float height)
{
#if 0
    Window win;
    XSizeHints hints;

    if( __glutCurrentWindow )
    {
        win = __glutCurrentWindow->win;


        hints.flags = PAspect;
        hints.min_aspect.x = hints.max_aspect.x = (int)(1000*width);
        hints.min_aspect.y = hints.max_aspect.y = (int)(1000*height);

        XSetWMNormalHints(__glutDisplay, win, &hints);
    }
#endif
}

}; // namespace Terra

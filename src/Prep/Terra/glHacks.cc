#include "glHacks.h"

void glGetViewport(int *x, int *y, int *w, int *h)
{
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    if( x ) *x = viewport[0];
    if( y ) *y = viewport[1];
    if( w ) *w = viewport[2];
    if( h ) *h = viewport[3];
}

void glUnproject(int win_x, int win_y, int win_z,
			double *x, double *y, double *z)
{
    GLdouble modelMatrix[16];
    GLdouble projMatrix[16];
    GLint viewport[4];

    glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
    glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
    glGetIntegerv(GL_VIEWPORT, viewport);

    gluUnProject(win_x,win_y,win_z,
		 modelMatrix, projMatrix, viewport,
		 x, y, z);
}

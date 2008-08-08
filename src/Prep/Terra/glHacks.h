#ifndef GLHACKS_INCLUDED // -*- C++ -*-
#define GLHACKS_INCLUDED

#include <iostream>

#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>

/*************************************************************************
 *
 * Yes, here it is.  A bunch of nice overloaded routines to make it
 * easier on the fingers to write OpenGL drawing code.
 *
 * The general stuff provided here is:
 *
 *      - glV
 *      - glN
 *      - glC
 *      - glLoadMatrix/glGetMatrix
 *
 *************************************************************************/



inline void glV(GLdouble x, GLdouble y) { glVertex2d(x,y); }
inline void glV(GLfloat x, GLfloat y) { glVertex2f(x,y); }
inline void glV(GLint x, GLint y) {  glVertex2i(x,y); }
inline void glV(GLshort x, GLshort y) { glVertex2s(x,y); }

inline void glV(GLdouble x, GLdouble y, GLdouble z) { glVertex3d(x,y,z); }
inline void glV(GLfloat x, GLfloat y, GLfloat z) { glVertex3f(x,y,z); }
inline void glV(GLint x, GLint y, GLint z) { glVertex3i(x,y,z); }
inline void glV(GLshort x, GLshort y, GLshort z) { glVertex3s(x,y,z); }


inline void glV(GLdouble x, GLdouble y, GLdouble z, GLdouble w)
            { glVertex4d(x,y,z,w); }
inline void glV(GLfloat x, GLfloat y, GLfloat z, GLfloat w)
            { glVertex4f(x,y,z,w); }
inline void glV(GLint x, GLint y, GLint z, GLint w) { glVertex4i(x,y,z,w); }
inline void glV(GLshort x, GLshort y, GLshort z, GLshort w )
            { glVertex4s(x,y,z,w); }

inline void glV2(const GLdouble *v) { glVertex2dv(v); }
inline void glV2(const GLfloat *v) { glVertex2fv(v); }
inline void glV2(const GLint *v) { glVertex2iv(v); }
inline void glV2(const GLshort *v) { glVertex2sv(v); }


inline void glV3(const GLdouble *v) { glVertex3dv(v); }
inline void glV3(const GLfloat *v) { glVertex3fv(v); }
inline void glV3(const GLint *v) { glVertex3iv(v); }
inline void glV3(const GLshort *v) { glVertex3sv(v); }


inline void glV4(const GLdouble *v) { glVertex4dv(v); }
inline void glV4(const GLfloat *v) { glVertex4fv(v); }
inline void glV4(const GLint *v) { glVertex4iv(v); }
inline void glV4(const GLshort *v) { glVertex4sv(v); }



inline void glN(GLdouble x, GLdouble y, GLdouble z) { glNormal3d(x,y,z); }
inline void glN(GLfloat x, GLfloat y, GLfloat z) { glNormal3f(x,y,z); }
inline void glN(GLint x, GLint y, GLint z) { glNormal3i(x,y,z); }
inline void glN(GLshort x, GLshort y, GLshort z) { glNormal3s(x,y,z); }

inline void glN3(const GLdouble *v) { glNormal3dv(v); }
inline void glN3(const GLfloat *v) { glNormal3fv(v); }
inline void glN3(const GLint *v) { glNormal3iv(v); }
inline void glN3(const GLshort *v) { glNormal3sv(v); }



inline void glC(GLdouble x, GLdouble y, GLdouble z) { glColor3d(x,y,z); }
inline void glC(GLfloat x, GLfloat y, GLfloat z) { glColor3f(x,y,z); }
inline void glC(GLint x, GLint y, GLint z) { glColor3i(x,y,z); }
inline void glC(GLshort x, GLshort y, GLshort z) { glColor3s(x,y,z); }

inline void glC(GLdouble x, GLdouble y, GLdouble z, GLdouble w)
            { glColor4d(x,y,z,w); }
inline void glC(GLfloat x, GLfloat y, GLfloat z, GLfloat w)
            { glColor4f(x,y,z,w); }
inline void glC(GLint x, GLint y, GLint z, GLint w) { glColor4i(x,y,z,w); }
inline void glC(GLshort x, GLshort y, GLshort z, GLshort w )
            { glColor4s(x,y,z,w); }

inline void glC3(const GLdouble *v) { glColor3dv(v); }
inline void glC3(const GLfloat *v) { glColor3fv(v); }
inline void glC3(const GLint *v) { glColor3iv(v); }
inline void glC3(const GLshort *v) { glColor3sv(v); }

inline void glC4(const GLdouble *v) { glColor4dv(v); }
inline void glC4(const GLfloat *v) { glColor4fv(v); }
inline void glC4(const GLint *v) { glColor4iv(v); }
inline void glC4(const GLshort *v) { glColor4sv(v); }



inline void glLoadMatrix(const GLdouble *m) { glLoadMatrixd(m); }
inline void glLoadMatrix(const GLfloat *m) { glLoadMatrixf(m); }

inline void glGetMatrix(GLdouble *m,GLenum src=GL_PROJECTION_MATRIX)
{ glGetDoublev(src,m); }
inline void glGetMatrix(GLfloat *m,GLenum src=GL_PROJECTION_MATRIX)
{ glGetFloatv(src,m); }



/*************************************************************************
 *
 * Here are some more generally useful things.
 *
 *************************************************************************/

extern void glGetViewport(int *x, int *y, int *w, int *h);
extern void glUnproject(int win_x, int win_y, int win_z,
			double *x, double *y, double *z);

typedef union {
    unsigned int pixel;
    struct { unsigned char r,g,b,a; } channel;
} gfxPixel;


inline gfxPixel *glSnapshot(int x, int y, int w, int h)
{
    gfxPixel *data = new gfxPixel[w*h];

    glReadBuffer(GL_FRONT);
    glReadPixels(x,y,w,h,
		 GL_RGBA,
		 GL_UNSIGNED_BYTE,
		 data);

    return data;
}

#ifdef __STDIO_H__
inline void glToPPM(FILE *out)
{
    int x,y,w,h;
    
    glGetViewport(&x, &y, &w, &h);
    gfxPixel *data = glSnapshot(x,y,w,h);

    fprintf(out,"P6 %u %u 255\n",w,h);
    int i,j;
    for(j=h-1;j>=0;j--)
	for(i=0;i<w;i++)
	{
	    int index = j*w + i;
	    fputc(data[index].channel.r, out);
	    fputc(data[index].channel.g, out);
	    fputc(data[index].channel.b, out);
	}

    delete[] data;
}
#endif

inline std::ostream& operator<<(std::ostream& out, const gfxPixel& p)
{
    return out << p.channel.r << p.channel.g << p.channel.b;
}

inline void glToPPM(std::ostream& out)
{
    int x,y,w,h;
    
    glGetViewport(&x, &y, &w, &h);
    gfxPixel *data = glSnapshot(x,y,w,h);

    out << "P6 " << w <<" "<< h << " 255" << std::endl;
    int i,j;
    for(j=h-1;j>=0;j--)
	for(i=0;i<w;i++)
	{
	    int index = j*w + i;
	    out << data[index];
	}

    delete[] data;
}


#endif

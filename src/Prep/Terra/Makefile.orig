#################################################################
#
# Configuration variables.
# You should change these to fit your system.
#

CC = cc
C++ = CC

# For compiling on SGI's with the pre-5.3 (ie. cfront-based) compiler:
# add '-ptr/tmp/terra_ptrepository' to OPTFLAGS
# add '-pte.cc' to LFLAGS

OPTFLAGS = -g -mips2 
# OPTFLAGS = -O2 -mips2

GUI_LIBS = -lglut -lGLU -lGL -lXmu -lX11
LIBS = -lmalloc -lmx

#
# This defines the location of the GLUT libraries
#
ANIM = /afs/cs/project/anim/garland
GLUT_FLAGS = 
GLUT_INCDIR = $(ANIM)/include
GLUT_LIBDIR = $(ANIM)/lib

#
# Include any other search directories you need here
#
INCDIR = -I$(GLUT_INCDIR)
LIBDIR = -L$(GLUT_LIBDIR)

#
# These are the flags for compilation (CFLAGS) and linking (LFLAGS) 
#
CFLAGS = $(INCDIR) $(OPTFLAGS) -DSAFETY
LFLAGS = $(LIBDIR) $(OPTFLAGS)


#################################################################
#
# Rules for building the Terra programs.
# You should not need to change anything here.
#

.SUFFIXES: .cc
.cc.o:
	$(C++) $(CFLAGS) -c $<

.C.o:
	$(C++) $(CFLAGS) -c $<


BASE_SRCS = Quadedge.cc Subdivision.cc Map.cc Mask.cc cmdline.cc \
            GreedyInsert.cc Heap.cc greedy.cc output.cc
GUI_SRCS  = glHacks.cc gui.cc xterra.cc

TERRA_SRCS = terra.cc $(BASE_SRCS)
XTERRA_SRCS = $(GUI_SRCS) $(BASE_SRCS)

TERRA_OBJS = $(TERRA_SRCS:.cc=.o)
XTERRA_OBJS = $(XTERRA_SRCS:.cc=.o)


TARGETS = terra xterra

all: $(TARGETS)

terra: $(TERRA_OBJS)
	$(C++) $(LFLAGS) -o terra $(TERRA_OBJS) $(LIBS)

xterra: $(XTERRA_OBJS)
	$(C++) $(LFLAGS) -o xterra $(XTERRA_OBJS) $(GUI_LIBS) $(LIBS)

clean :
	/bin/rm -f $(XTERRA_OBJS) $(TERRA_OBJS) $(TARGETS)
	/bin/rm -f -r ii_files ptrepository
	find . -name '*~'    -print -exec rm -f {} \;
	find . -name 'core' -print -exec rm -f {} \;

depend :
	touch Makefile.depend
	makedepend -fMakefile.depend $(INCDIR) -I/usr/include/CC $(BASE_SRCS) $(GUI_SRCS)
	/bin/rm -f Makefile.depend.bak

sinclude Makefile.depend

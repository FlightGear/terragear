#include <GL/glut.h>

#include "terra.h"
#include "gui.h"



main(int argc, char **argv)
{
    glutInit(&argc, argv);
    process_cmdline(argc, argv);


    gui_init();


    gui_interact();
}

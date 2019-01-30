#include <GL/glut.h>

#include "terra.h"
#include "gui.h"


int main(int argc, char **argv)
{
    glutInit(&argc, argv);
    Terra::process_cmdline(argc, argv);

    Terra::gui_init();

    Terra::gui_interact();

    return 0;
}

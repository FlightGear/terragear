#include "terra.h"

int main(int argc, char **argv)
{
    Terra::process_cmdline(argc, argv);

    Terra::greedy_insertion();

    Terra::generate_output();
}

#include "terra.h"

main(int argc, char **argv)
{
    process_cmdline(argc, argv);

    greedy_insertion();

    generate_output();
}

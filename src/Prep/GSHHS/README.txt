GSHHS - Global Self-consistant Hierarchical High-resolution Shorelines

http://www.soest.hawaii.edu/soest/gmt.html

This utility transforms the raw GSHHS data into TerraGear polygons and
chops them up according to our tiling scheme.

Usage: ./gshhs <gshhs_file> <work_dir> <level> [ area_type ]

level must be either 1 (land), 2 (lake), 3 (island), or 4 (pond)

This determines which feature of the data set to extract.  If you want
to extract all four feature types, you will need to run the utility 4
separate times.  I recommend you extract each feature into a separate
work directory.  This will ease managment issues if something get's
hosed during the run or if you want to go rerun a section.  This also
allows you to optionally build scenery with the land mass source, but
not the lakes/islands/ponds if you should decide to use a different
data souce for those.

Some of the initial large landmass processing can consume a large
amount of RAM if you are processing the highest res data set.

By default, this utility assigns the "DefaultArea" land cover type to
the land mass.  You can optionally override this if you want to use
something else.  TerraGear uses the land cover to assign textures in
the final scenery.

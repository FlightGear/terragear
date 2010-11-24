All "grass*" scripts are to be run as $GRASS_BATCH_JOB on the
respective mapset in order to prepare and clean polygon datasets for
the use with TerraGear's "construct" tool.  In the long term, the
preprocessing in GRASS is destined to replace the respective
functionality in "construct".
Note: GRASS7 is required for certain features.

The "IntersectsRemoval.sh" utility takes a single Shapefile containing
one single closed POLYGON (MULTIPOLYGON's _should_ work but might crash
in PostGIS) to cut a hole into the Custom Scenery basemap.

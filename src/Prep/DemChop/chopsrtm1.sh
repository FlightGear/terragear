#! /bin/bash

DATADIR=/home/martin/archive/GIS/GISData/SRTM/version2_1/HGT/SRTM1/
WORKDIR=$HOME/workdirs/world_scenery

for file in "$DATADIR/"Region_*/*.hgt.zip; do
	hgtchop 1 $file $WORKDIR/SRTM2-America-1
done

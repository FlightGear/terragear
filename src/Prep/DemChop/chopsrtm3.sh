#! /bin/bash

echo "Available regions: Africa Australia Eurasia Islands North_America South_America"

#DATADIR=/mnt/agami/custom_scenery/SRTM/version2/HGT
DARADIR=/home/martin/archive/GIS/GISData/SRTM/version2_1/HGT/SRTM3/
WORKDIR=$HOME/workdirs/world_scenery

for region in Africa Australia Eurasia Islands North_America South_America; do
    for file in "$DATADIR/$region/"*.hgt.zip; do
	hgtchop 3 $file $WORKDIR/SRTM2-$region-3
    done
done

#! /bin/bash

#DATADIR=/mnt/geothumper/data/custom_scenery/SRTM/srtm30/
DATADIR=/home/martin/archive/GIS/GISData/SRTM/version2_1/HGT/SRTM30
WORKDIR=$HOME/workdirs/world_scenery

for file in e020n90  e060n90  e100n90  e140n90  w020n90  w060n90  w100n90  w140n90  w180n90; do
    unzip $DATADIR/$file/$file.dem.zip
    unzip $DATADIR/$file/$file.hdr.zip
    UPPERBASE=`echo $file | tr a-z A-Z`
    OUTPUTDIR=$WORKDIR/SRTM-30-ASCII/$file
    mkdir -p $OUTPUTDIR
    raw2ascii $UPPERBASE $OUTPUTDIR
    rm $UPPERBASE.{DEM,HDR}
done

#! /bin/bash

WORKDIR=$HOME/workdirs/world_scenery

for f in $WORKDIR/SRTM-30-ASCII/*/*.dem; do
    demchop $f $WORKDIR/SRTM-30
done
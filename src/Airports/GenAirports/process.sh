#! /bin/bash

# Previously Skipped: EBPP
# Manually removed: NZSP (led to a runway around the world)
# Fixed: YSAR (was marked as land airport, but was a heliport)

WORKDIR=$HOME/workdirs/world_scenery
#APTDAT="/home/martin/GIT/fgdata/Airports/apt.dat.gz"
#APTDAT="/home/rgerlich/rawdata/apt.dat.gz"
APTDAT="/home/rgerlich/rawdata/apt.helidat.gz"
SPAT="--nudge=20"

exec /usr/bin/time genapts --input=$APTDAT --work=$WORKDIR $SPAT > genapts.log 2>&1

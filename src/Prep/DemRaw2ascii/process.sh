#!/bin/sh
#

TOOLPATH=`pwd`
SOURCE=/stage/fgfs01/curt/RawData/DEM-30-Bin
OUTPUTBASE=/stage/fgfs01/curt/RawData/DEM-30-Ascii

SCRATCH=${SOURCE}/Scratch

# clean the scratch space
echo rm -rf ${SCRATCH}/*
rm -rf ${SCRATCH}/*

cd ${SCRATCH} || exit
echo "cd to $SCRATCH successful"

tar xzvf ${SOURCE}/${1}.tar.gz

NAME=`echo *.DEM`

BASE=`basename $NAME .DEM`
LOWBASE=`echo $BASE | tr A-Z a-z`

OUTPUT=${OUTPUTBASE}/${LOWBASE}

echo mkdir -p $OUTPUT
mkdir -p $OUTPUT

${TOOLPATH}/raw2ascii ${SCRATCH}/${BASE} ${OUTPUT}

cd ${OUTPUTBASE} || exit
echo "cd to $OUTPUTBASE successful"

tar czvf ${LOWBASE}.tar.gz ${LOWBASE}

echo rm -rf ${LOWBASE}
rm -rf ${LOWBASE}



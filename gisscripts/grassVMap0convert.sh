#!/bin/bash
#
# Written by Martin Spott
#
# Copyright (C) 2010, 2011  Martin Spott - Martin (at) flightgear (dot) org
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#

DUMPDIR=${HOME}/live/vmap0shp
mkdir -p ${DUMPDIR}

# Have an old-style, 32-bit setup of GDAL and dependencies, because
# 'ogr2ogr' on Debian Squeeze segfaults on certain features.
#
export LD_LIBRARY_PATH=/opt/GRASS.32/lib
OGR2OGR=/opt/GRASS.32/bin/ogr2ogr
OGRINFO=/opt/GRASS.32/bin/ogrinfo

AREAS="v0eur_5/vmaplv0/eurnasia v0noa_5/vmaplv0/noamer v0sas_5/vmaplv0/sasaus v0soa_5/vmaplv0/soamafr"

# Proposal on how to retrieve available features:
#
GetFeatureNames () {
  for VMAP_URL in ${AREAS}; do
      ${OGRINFO} gltp:/vrf/home/martin/live/vmap0/${VMAP_URL}
  done 2>&1 | egrep \^"[0-9].*@" | awk '{print $2}' | sort | uniq
}

# Finally convert:
#
ConvertToSHapefiles () {
    for VMAP_URL in ${AREAS}; do
        ZONE=`echo ${VMAP_URL} | cut -f 1 -d \/ | sed -e 's/^v0//g' -e 's/_5\$//g'`
        for FEATURE in `${OGRINFO} gltp:/vrf/home/martin/live/vmap0/${VMAP_URL} 2>&1 | egrep \^"[0-9].*@" | awk '{print $2}' | sort | uniq`; do
            CATEGORY=`echo ${FEATURE} | cut -f 1 -d \@`
            DIR=`echo ${FEATURE} | cut -f 1 -d \( | cut -f 2 -d \@`
            TYPE=`echo ${FEATURE} | cut -f 2 -d \_`
            OUTPUT=${ZONE}_${DIR}-${CATEGORY}-${TYPE}
            echo "### ${VMAP_URL} # ${FEATURE} # ${OUTPUT} ###"
            ${OGR2OGR} -f "ESRI Shapefile" ${DUMPDIR}/${OUTPUT}.shp gltp:/vrf/home/martin/live/vmap0/${VMAP_URL} ${FEATURE}; echo ${?}
        done
    done 
}

#GetFeatureNames
ConvertToSHapefiles

# EOF

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

DUMPDIR=${HOME}/shp
mkdir -p ${DUMPDIR}

# Have an old-style, 32-bit setup of GDAL and dependencies, because
# 'ogr2ogr' on Debian Squeeze segfaults on certain features.
#
export LD_LIBRARY_PATH=/opt/GRASS.32/lib
OGR2OGR=/opt/GRASS.32/bin/ogr2ogr

for VMAP_URL in v0eur_5/vmaplv0/eurnasia v0noa_5/vmaplv0/noamer v0sas_5/vmaplv0/sasaus v0soa_5/vmaplv0/soamafr; do
    ZONE=`echo ${VMAP_URL} | cut -f 1 -d \/ | sed -e 's/^v0//g' -e 's/_5\$//g'`
    for FEATURE in 'aerofacp@trans(*)_point' 'aquecanl@hydro(*)_line' 'barrierl@bnd(*)_line' 'bndtxt@bnd(*)_text' 'builtupa@pop(*)_area' 'builtupp@pop(*)_point' 'coastl@bnd(*)_line' 'contourl@elev(*)_line' 'cropa@veg(*)_area' 'cutfill@phys(*)_line' 'dangerp@hydro(*)_point' 'depthl@bnd(*)_line' 'dqarea@dq(*)_area' 'dqline@dq(*)_line' 'dqline@trans(*)_line' 'dqline@util(*)_line' 'dqtxt@dq(*)_text' 'elevp@elev(*)_point' 'extracta@ind(*)_area' 'extractp@ind(*)_point' 'fishinda@ind(*)_area' 'grassa@veg(*)_area' 'grounda@phys(*)_area' 'hydrotxt@hydro(*)_text' 'indtxt@ind(*)_text' 'inwatera@hydro(*)_area' 'landicea@phys(*)_area' 'libref@libref(*)_line' 'libreft@libref(*)_text' 'lndfrml@phys(*)_line' 'miscl@hydro(*)_line' 'miscp@hydro(*)_point' 'misindp@ind(*)_point' 'mispopa@pop(*)_area' 'mispopp@pop(*)_point' 'mistranl@trans(*)_line' 'oceansea@bnd(*)_area' 'oasisa@veg(*)_area' 'orcharda@veg(*)_area' 'phystxt@phys(*)_text' 'pipel@util(*)_line' 'polbnda@bnd(*)_area' 'polbndl@bnd(*)_line' 'polbndp@bnd(*)_point' 'poptxt@pop(*)_text' 'railrdl@trans(*)_line' 'roadl@trans(*)_line' 'rryardp@trans(*)_point' 'seaicea@phys(*)_area' 'storagep@ind(*)_point' 'swampa@veg(*)_area' 'tileref@tileref(*)_area' 'tilereft@tileref(*)_text' 'traill@trans(*)_line' 'transtrc@trans(*)_point' 'transtrl@trans(*)_line' 'transtxt@trans(*)_text' 'treesa@veg(*)_area' 'tundraa@veg(*)_area' 'utill@util(*)_line' 'utilp@util(*)_point' 'utiltxt@util(*)_text' 'watrcrsl@hydro(*)_line'; do
        CATEGORY=`echo ${FEATURE} | cut -f 1 -d \@`
        DIR=`echo ${FEATURE} | cut -f 1 -d \( | cut -f 2 -d \@`
        TYPE=`echo ${FEATURE} | cut -f 2 -d \_`
        OUTPUT=${ZONE}_${DIR}-${CATEGORY}-${TYPE}
        echo "### ${VMAP_URL} # ${FEATURE} # ${OUTPUT} ###"
        ${OGR2OGR} -f "ESRI Shapefile" ${DUMPDIR}/${OUTPUT}.shp gltp:/vrf/home/martin/live/vmap0/${VMAP_URL} ${FEATURE}; echo ${?}
    done
done 

# EOF

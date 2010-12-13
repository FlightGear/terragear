#!/bin/sh
#
# Written by Martin Spott
#
# Copyright (C) 2010  Markus Metz @ GRASS GIS
# Copyright (C) 2010  Martin Spott - Martin (at) flightgear (dot) org
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

#MODE=${1}  # Doesn't work in $GRASS_BATCH_JOB
# Create reasonable symlinks pointing to this script to return proper
# $MODE-values for the "case" clause, like 'grass01import.sh_shp'
MODE=`basename ${0} | cut -f 2 -d \_`
#
# Self-explaining ....
DEBUG=false
#
# DB-Connection
PGHOST=geoscope.optiputer.net
PGUSER=webuser
PGDATABASE=landcover
PSQL="psql -tA -h ${PGHOST} -U ${PGUSER} -d ${PGDATABASE} -c"
#
# Note: CORINE Shapefiles are EPSG:3035, Landcover-DB is EPSG:4326, but
# _both_ projections (reference systems) are using square meters as unit for
# area size declarations in GRASS !
# Units for snapping are different in GRASS, though. Whereas EPSG:3035 is
# using meters, EPSG:4326 is using degrees.
#
case ${MODE} in
	shp)
	    # Local, pre-intersected Shapefiles
	    DSN=${HOME}/live/corine
#	    PREFIX=nl_
	    PREFIX=clc00_
	    SELECTION=${DSN}/${PREFIX}c[0-9][0-9][0-9].shp
	    SNAP=1
	    SPAT=""
	;;
	ldb)
	    # Landcover-DB
	    DSN="PG:host=${PGHOST} user=${PGUSER} dbname=${PGDATABASE}"
	    PREFIX=clc_
	    # Unsafe because "_" matches any single character in LIKE.
#	    SELECTION=`${PSQL} "SELECT f_table_name FROM geometry_columns WHERE f_table_name LIKE '${PREFIX}%' AND type LIKE 'POLYGON' ORDER BY f_table_name"`
	    SELECTION=`psql -tA -c "\dt" | awk -F\| '{print $2}' | grep \^${PREFIX}`
	    SNAP=0.00001
	    SPAT="spatial=2.8,49.8,8.2,54.2"  # spatial=-123,37,-121,38  # spatial=3,50,8,54
	;;
esac
#
MIN_AREA=1

# Strip input down to bare GRASS map name
#
Selection () {
    for FILE in ${SELECTION}; do
        basename ${FILE} | cut -f 1 -d \. | cut -f 2 -d \_
    done
}

if [ ${DEBUG} = "true" ]; then
    echo "PREFIX # GRASSMAP # DSN # LAYER-in-DSN"
fi

# Go
#
for MAP in `echo $(Selection)`; do
    LAYER=${PREFIX}${MAP}
    if [ ${DEBUG} = "true" ]; then
        echo "${PREFIX} # ${MAP} # ${DSN} # ${LAYER}"
    fi
    v.in.ogr dsn="${DSN}" layer=${LAYER} output=${MAP} snap=${SNAP} min_area=${MIN_AREA} ${SPAT} --verbose
done

# EOF

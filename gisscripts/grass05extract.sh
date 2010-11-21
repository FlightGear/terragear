#!/bin/sh
#
# Written by Martin Spott
#
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

# Create reasonable symlinks pointing to this script to return proper
# $MODE-values for the "case" clause, like 'grass05extract.sh_shp'
MODE=`basename ${0} | cut -f 2 -d \_`

DUMPDIR=${HOME}/shp
mkdir -p ${DUMPDIR}
RUNDIR=`pwd`
cd `dirname ${0}` && export BASEDIR=`pwd`
cd ${RUNDIR}
#

MAPPINGFILE=${BASEDIR}/CORINEtoCStest.txt
CLEANMAP=clc00_nl_clobber
EXTRACTPREFIX=clc00_nl_

case ${MODE} in
	shp)
	    # Re-project from EPSG:3035 - this one is _not_ to be run inside
	    # the import location !!!
	    v.proj location=corine mapset=PERMANENT input=${CLEANMAP}  # output=${CLEANMAP}
	;;
esac

# Print available (integer) categories - centroids only !!!
#
SELECTION=`v.category input=${CLEANMAP} type=centroid option=print | sort -n | uniq`

for CATEGORY in ${SELECTION}; do
    LAYER=`grep \^${CATEGORY} ${MAPPINGFILE} | awk '{print $2}' | sed -e "s/^cs_/${EXTRACTPREFIX}/g"`
    v.extract list=${CATEGORY} input=${CLEANMAP} output=${EXTRACTPREFIX}${CATEGORY} type=area
    v.dissolve input=${EXTRACTPREFIX}${CATEGORY} output=${EXTRACTPREFIX}${CATEGORY}_dissolved
    v.out.ogr input=${EXTRACTPREFIX}${CATEGORY}_dissolved dsn=${DUMPDIR}/${LAYER}.shp
done

# EOF

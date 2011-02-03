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

# Create reasonable symlinks pointing to this script to return proper
# $MODE-values for the "case" clause, like 'grass04clean.sh_shp_first'
COMMAND=`basename ${0} | cut -f 1 -d \_`
MODE=`basename ${0} | cut -f 2 -d \_`
RUN=`basename ${0} | cut -f 3 -d \_`

#PATCHMAP=clc00_nl
PATCHMAP=clc00

case ${MODE} in
	shp)
	    SNAP=1
	;;
	ldb)
	    SNAP=0.0003  # SNAP=0.00001
	;;
esac
#
MIN_AREA=10

case ${RUN} in
	first)
	    v.clean input=${PATCHMAP} output=${PATCHMAP}_clean tool=snap,bpol,rmdupl,rmline,break,rmdupl,rmdangle,rmarea thresh=${SNAP},0,0,0,0,0,0,${MIN_AREA} type=boundary --verbose
	    v.dissolve input=${PATCHMAP}_clean output=${PATCHMAP}_dissolved
	;;
	second)
	    v.clean input=${PATCHMAP}_dissolved output=${PATCHMAP}_clobber tool=break,rmdupl,rmdangle type=boundary --verbose
	;;
	loop)
	    g.remove vect=${PATCHMAP}_dissolved
	    g.rename vect=${PATCHMAP}_clobber,${PATCHMAP}_dissolved
	    ${COMMAND}_${MODE}_second
	;;
esac

# EOF

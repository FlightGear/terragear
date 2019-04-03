#!/bin/bash -x
#
# Copyright (C) - 2006  FlightGear scenery team
# Copyright (C) 2006 - 2014  Martin Spott
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

date

WORKBASE=${HOME}/workdirs
OUTPUTDIR=${HOME}/scenery/Terrain
DEBUGDIR=${HOME}/scenery/Debug
SHAREDIR=${HOME}/scenery/Shared
NUDGE="--nudge=20"

mkdir -p ${OUTPUTDIR} ${DEBUGDIR}

#SHAREDOPTS="--no-write-shared-edges --use-own-shared-edges"
#SHAREDOPTS="--no-write-shared-edges"

# Base Package
#SPAT="--min-lon=-124 --max-lon=-120 --min-lat=36 --max-lat=39"
# World
#SPAT=""  # this one doesn't work
#SPAT="--min-lon=-180 --max-lon=180 --min-lat=-90 --max-lat=90"

CONSTRUCT=${HOME}/terragear/bin/tg-construct

LOADDIRS="AirportArea \
    AirportObj \
    SRTM2-VFP-3-NE \
    SRTM2-VFP-3-NW \
    SRTM2-VFP-3-SE \
    SRTM2-VFP-3-SW \
    Poly-City \
    Poly-Commercial \
    Poly-Floodland \
    Poly-Waterbody \
    Poly-Agro \
    Poly-Forest \
    Poly-LandCover \
    Poly-Ice \
    Poly-Watercourse \
    Poly-RoadCover \
    Line-Road \
    Line-Railroad \
    Line-Stream"

# --threads=4

${CONSTRUCT} \
    --output-dir=${OUTPUTDIR} \
    --work-dir=${WORKBASE} \
    --share-dir=${SHAREDIR} \
    --priorities=${HOME}/terragear/share/TerraGear/default_priorities.txt \
    ${NUDGE} ${SPAT} --ignore-landmass \
    --debug-dir=/home/martin/scenery/Debug \
    ${LOADDIRS}

# EOF

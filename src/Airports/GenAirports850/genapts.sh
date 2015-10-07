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
NUDGE="--nudge=20"

# Base Package
#SPAT="--min-lon=-124 --max-lon=-120 --min-lat=36 --max-lat=39"
# World
SPAT=""

APTDAT=${HOME}/live/airfield/v10+/apt.dat

DEM="--dem-path=SRTM2-VFP-3-NE --dem-path=SRTM2-VFP-3-NW \
     --dem-path=SRTM2-VFP-3-SE --dem-path=SRTM2-VFP-3-SW"

GENAPTS=${HOME}/terragear/bin/genapts850

#${GENAPTS} --threads --input=${APTDAT} --work=${WORKBASE} ${DEM} ${NUDGE} ${SPAT}
${GENAPTS} --input=${APTDAT} --work=${WORKBASE} ${DEM} ${NUDGE} ${SPAT}

# EOF

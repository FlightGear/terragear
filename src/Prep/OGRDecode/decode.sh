#!/bin/bash
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

# Base Package
#SPAT="--spat -124 36 -120 39"
# World
#SPAT=""
# works better
SPAT="--spat -180 -90 180 90"

DATASOURCE="PG:dbname=${PGDATABASE} host=${PGHOST} user=${PGUSER}"

#OGRDECODE="${HOME}/terragear/bin/ogr-decode --all-threads --continue-on-errors --max-segment 400 ${SPAT}"
OGRDECODE="${HOME}/terragear/bin/ogr-decode --max-segment 400 ${SPAT}"
SHAPEDECODE="${HOME}/terragear/bin/shape-decode --max-segment 400"
COMMAND=${OGRDECODE}

ProcessLandmass() {
    date
    # World land mass
    ${COMMAND} --area-type Default ${WORKBASE}/Poly-LandMass "${DATASOURCE}" cs_landmass
}  # ProcessLandmass

ProcessWaterbody() {
    date
    # Inland still water: lakes, intermittent lakes, and flood land
    ${COMMAND} --area-type Lake ${WORKBASE}/Poly-Waterbody "${DATASOURCE}" cs_lake
    ${COMMAND} --area-type IntermittentLake ${WORKBASE}/Poly-Waterbody "${DATASOURCE}" cs_intermittentlake
    ${COMMAND} --area-type Lagoon ${WORKBASE}/Poly-Waterbody "${DATASOURCE}" cs_lagoon
    ${COMMAND} --area-type Estuary ${WORKBASE}/Poly-Waterbody "${DATASOURCE}" cs_estuary
    ${COMMAND} --area-type Watercourse ${WORKBASE}/Poly-Watercourse "${DATASOURCE}" cs_watercourse
}  # ProcessWaterbody

ProcessCity() {
    date
    # Population areas: cities and towns
    ${COMMAND} --area-type SubUrban ${WORKBASE}/Poly-City "${DATASOURCE}" cs_suburban
    ${COMMAND} --area-type Urban ${WORKBASE}/Poly-City "${DATASOURCE}" cs_urban
    ${COMMAND} --area-type Town ${WORKBASE}/Poly-City "${DATASOURCE}" cs_town
#    ${COMMAND} --area-type Town --point-width 400 ${WORKBASE}/Poly-City "${DATASOURCE}" cs_town
}  # ProcessCity

ProcessCommercial() {
    date
    ${COMMAND} --area-type Industrial ${WORKBASE}/Poly-Commercial "${DATASOURCE}" cs_industrial
    ${COMMAND} --area-type Construction ${WORKBASE}/Poly-Commercial "${DATASOURCE}" cs_construction
    ${COMMAND} --area-type Transport ${WORKBASE}/Poly-Commercial "${DATASOURCE}" cs_transport
    ${COMMAND} --area-type Port ${WORKBASE}/Poly-Commercial "${DATASOURCE}" cs_port
    ${COMMAND} --area-type Fishing ${WORKBASE}/Poly-Commercial "${DATASOURCE}" cs_fishing
}  # ProcessCommercial

ProcessForest() {
    date
    # Forest: deciduous broad, evergreen broad, mixed
    ${COMMAND} --area-type DeciduousForest ${WORKBASE}/Poly-Forest "${DATASOURCE}" cs_deciduousforest
    ${COMMAND} --area-type EvergreenForest ${WORKBASE}/Poly-Forest "${DATASOURCE}" cs_evergreenforest
    ${COMMAND} --area-type MixedForest ${WORKBASE}/Poly-Forest "${DATASOURCE}" cs_mixedforest
    ${COMMAND} --area-type RainForest ${WORKBASE}/Poly-Forest "${DATASOURCE}" cs_rainforest
    ${COMMAND} --area-type AgroForest ${WORKBASE}/Poly-Forest "${DATASOURCE}" cs_agroforest
}  # ProcessForest

ProcessLandCover() {
    date
    # Ground cover: sand, tidal, lava, barren, grass, scrub, herb-tundra
    ${COMMAND} --area-type Airport ${WORKBASE}/Poly-LandCover "${DATASOURCE}" cs_airport
    ${COMMAND} --area-type Rock ${WORKBASE}/Poly-LandCover "${DATASOURCE}" cs_rock
    ${COMMAND} --area-type Dirt ${WORKBASE}/Poly-LandCover "${DATASOURCE}" cs_dirt
    ${COMMAND} --area-type BarrenCover ${WORKBASE}/Poly-LandCover "${DATASOURCE}" cs_barrencover
    ${COMMAND} --area-type GolfCourse ${WORKBASE}/Poly-LandCover "${DATASOURCE}" cs_golfcourse
    ${COMMAND} --area-type Cemetery ${WORKBASE}/Poly-LandCover "${DATASOURCE}" cs_cemetery
    ${COMMAND} --area-type Grassland ${WORKBASE}/Poly-LandCover "${DATASOURCE}" cs_grassland
    ${COMMAND} --area-type Greenspace ${WORKBASE}/Poly-LandCover "${DATASOURCE}" cs_greenspace
    ${COMMAND} --area-type OpenMining ${WORKBASE}/Poly-LandCover "${DATASOURCE}" cs_openmining
    ${COMMAND} --area-type Dump ${WORKBASE}/Poly-LandCover "${DATASOURCE}" cs_dump
    ${COMMAND} --area-type Lava ${WORKBASE}/Poly-LandCover "${DATASOURCE}" cs_lava
    ${COMMAND} --area-type Sand ${WORKBASE}/Poly-LandCover "${DATASOURCE}" cs_sand
    ${COMMAND} --area-type Saline ${WORKBASE}/Poly-LandCover "${DATASOURCE}" cs_saline
    ${COMMAND} --area-type Scrub ${WORKBASE}/Poly-LandCover "${DATASOURCE}" cs_scrub
    ${COMMAND} --area-type Heath ${WORKBASE}/Poly-LandCover "${DATASOURCE}" cs_heath
    ${COMMAND} --area-type Sclerophyllous ${WORKBASE}/Poly-LandCover "${DATASOURCE}" cs_sclerophyllous
    ${COMMAND} --area-type Burnt ${WORKBASE}/Poly-LandCover "${DATASOURCE}" cs_burnt
    ${COMMAND} --area-type HerbTundra ${WORKBASE}/Poly-LandCover "${DATASOURCE}" cs_herbtundra

    # Concrete walls
    ${COMMAND} --area-type Asphalt ${WORKBASE}/Poly-LandCover "${DATASOURCE}" cs_asphalt

    # Uncovered areas
    ${COMMAND} --area-type Scrub ${WORKBASE}/Poly-LandCover "${DATASOURCE}" cs_void
}  # ProcessLandCover

ProcessIce() {
    date
    # Ice cover: glaciers, pack ice, and sea ice
    ${COMMAND} --area-type Glacier ${WORKBASE}/Poly-Ice "${DATASOURCE}" cs_glacier
    ${COMMAND} --area-type PackIce ${WORKBASE}/Poly-Ice "${DATASOURCE}" cs_packice
    ${COMMAND} --area-type PolarIce ${WORKBASE}/Poly-Ice "${DATASOURCE}" cs_polarice
}  # ProcessIce

ProcessFloodland() {
    date
    # Marshes: marsh and bog
    ${COMMAND} --area-type Bog ${WORKBASE}/Poly-Floodland "${DATASOURCE}" cs_bog
    ${COMMAND} --area-type Marsh ${WORKBASE}/Poly-Floodland "${DATASOURCE}" cs_marsh
    ${COMMAND} --area-type FloodLand ${WORKBASE}/Poly-Floodland "${DATASOURCE}" cs_floodland
    ${COMMAND} --area-type Littoral ${WORKBASE}/Poly-Floodland "${DATASOURCE}" cs_littoral
    ${COMMAND} --area-type SaltMarsh ${WORKBASE}/Poly-Floodland "${DATASOURCE}" cs_saltmarsh
}  # ProcessFloodland

ProcessAgro() {
    date
    # Crops: mixed pasture, dry crop, irrigated crop
    ${COMMAND} --area-type CropGrass ${WORKBASE}/Poly-Agro "${DATASOURCE}" cs_cropgrass
    ${COMMAND} --area-type DryCrop ${WORKBASE}/Poly-Agro "${DATASOURCE}" cs_drycrop
    ${COMMAND} --area-type IrrCrop ${WORKBASE}/Poly-Agro "${DATASOURCE}" cs_irrcrop
    ${COMMAND} --area-type Rice ${WORKBASE}/Poly-Agro "${DATASOURCE}" cs_rice
    ${COMMAND} --area-type MixedCrop ${WORKBASE}/Poly-Agro "${DATASOURCE}" cs_mixedcrop
    ${COMMAND} --area-type ComplexCrop ${WORKBASE}/Poly-Agro "${DATASOURCE}" cs_complexcrop
    ${COMMAND} --area-type NaturalCrop ${WORKBASE}/Poly-Agro "${DATASOURCE}" cs_naturalcrop
    ${COMMAND} --area-type Vineyard ${WORKBASE}/Poly-Agro "${DATASOURCE}" cs_vineyard
    ${COMMAND} --area-type Orchard ${WORKBASE}/Poly-Agro "${DATASOURCE}" cs_orchard
    ${COMMAND} --area-type Olives ${WORKBASE}/Poly-Agro "${DATASOURCE}" cs_olives
}  # ProcessAgro

# OSM lines expect "SET tunnel = NULL WHERE tunnel LIKE '' OR tunnel ILIKE 'no' OR tunnel ILIKE 'false'"
#              and "SET bridge = NULL WHERE bridge LIKE '' OR bridge ILIKE 'no' OR bridge ILIKE 'false'"
# Get linear textures with "--texture-lines"
# Fetch line width from OSM tables via "--line-width-column width", but
#     requires pre-formatting width columns, as they may contain random
#     garbage
ProcessRoad() {
    date
    # Roads: highway, freeway, trail
    ${COMMAND} --texture-lines --where "tunnel IS NULL AND bridge IS NULL" --area-type Freeway --line-width 11 ${WORKBASE}/Line-Road "${DATASOURCE}" osm_motorway || echo "ERROR on osm_motorway: ${?}"
    ${COMMAND} --texture-lines --where "tunnel IS NULL AND bridge IS NULL" --area-type Freeway --line-width 7 ${WORKBASE}/Line-Road "${DATASOURCE}" osm_trunk || echo "ERROR on osm_trunk: ${?}"
    ${COMMAND} --texture-lines --where "tunnel IS NULL AND bridge IS NULL" --area-type Freeway --line-width 9 ${WORKBASE}/Line-Road "${DATASOURCE}" osm_primary || echo "ERROR on osm_primary: ${?}"
    ${COMMAND} --texture-lines --where "tunnel IS NULL AND bridge IS NULL" --area-type Road --line-width 7 ${WORKBASE}/Line-Road "${DATASOURCE}" osm_secondary || echo "ERROR on osm_secondary: ${?}"
#    ${COMMAND} --texture-lines --where "tunnel IS NULL AND bridge IS NULL" --area-type Road --line-width 5 ${WORKBASE}/Line-Road "${DATASOURCE}" osm_tertiary || echo "ERROR on osm_tertiary: ${?}"
#    ${COMMAND} --texture-lines --where "tunnel IS NULL AND bridge IS NULL" --area-type Road --line-width 4 ${WORKBASE}/Line-Road "${DATASOURCE}" osm_residential || echo "ERROR on osm_residential: ${?}"
#    ${COMMAND} --texture-lines --where "tunnel IS NULL AND bridge IS NULL" --area-type Road --line-width 4 ${WORKBASE}/Line-Road "${DATASOURCE}" osm_unclassified || echo "ERROR on osm_unclassified: ${?}"
#    ${COMMAND} --texture-lines --where "tunnel IS NULL AND bridge IS NULL" --area-type Road --line-width 3 ${WORKBASE}/Line-Road "${DATASOURCE}" osm_service || echo "ERROR on osm_service: ${?}"
    ${COMMAND} --texture-lines --where "tunnel IS NULL AND bridge IS NULL" --area-type Freeway --line-width 9 ${WORKBASE}/Line-Road "${DATASOURCE}" osm_raceway || echo "ERROR on osm_raceway: ${?}"
}  # ProcessRoad

ProcessRailroad() {
    date
    # Railroads: single and multiple tracks
    ${COMMAND} --texture-lines --where "tunnel IS NULL AND bridge IS NULL" --area-type Railroad --line-width 5 ${WORKBASE}/Line-Railroad "${DATASOURCE}" osm_rail || echo "ERROR on osm_rail: ${?}"
#    ${COMMAND} --texture-lines --where "tunnel IS NULL AND bridge IS NULL" --area-type Railroad --line-width 2 ${WORKBASE}/Line-Railroad "${DATASOURCE}" osm_tram || echo "ERROR on osm_tram: ${?}"
}  # ProcessRailroad

ProcessStream() {
    date
    # Inland moving water: rivers/streams, intermittent streams, and canals
    ${COMMAND} --texture-lines --where "tunnel IS NULL AND bridge IS NULL" --area-type Stream --line-width 3 ${WORKBASE}/Line-Stream "${DATASOURCE}" osm_drain || echo "ERROR on osm_drain: ${?}"
    ${COMMAND} --texture-lines --where "tunnel IS NULL AND bridge IS NULL" --area-type Stream --line-width 12 ${WORKBASE}/Line-Stream "${DATASOURCE}" osm_river || echo "ERROR on osm_river: ${?}"
    ${COMMAND} --texture-lines --where "tunnel IS NULL AND bridge IS NULL" --area-type Canal --line-width 12 ${WORKBASE}/Line-Stream "${DATASOURCE}" osm_canal || echo "ERROR on osm_canal: ${?}"
#
#    ${COMMAND} --area-type Freeway ${WORKBASE}/Poly-RoadCover "${DATASOURCE}" osm_road_cover
}  # ProcessStream

# ProcessLandmass

ProcessWaterbody
ProcessCity
ProcessCommercial
ProcessForest
#
ProcessLandCover
ProcessIce
ProcessFloodland
ProcessAgro
#
ProcessRoad
ProcessRailroad
ProcessStream

#CALLNAME=`basename ${0}`
#${CALLNAME}

# EOF

#!/bin/bash

WORKBASE=${HOME}/workdirs

# Base Package
#SPAT="--spat -124 36 -120 39"
# World
SPAT=""

DATASOURCE="PG:dbname=${PGDATABASE} host=${PGHOST} user=${PGUSER}"
COMMAND="${HOME}/terragear/bin/ogr-decode --continue-on-errors --max-segment 400 ${SPAT}"

ProcessLandmass () {
    # World land mass
    ${COMMAND} --area-type Default ${WORKBASE}/Shape-LandMass "${DATASOURCE}" cs_landmass
}  # ProcessLandmass

ProcessWaterbody () {
    # Inland still water: lakes, intermittent lakes, and flood land
    ${COMMAND} --area-type Lake ${WORKBASE}/Shape-Waterbody "${DATASOURCE}" cs_lake
    ${COMMAND} --area-type IntermittentLake ${WORKBASE}/Shape-Waterbody "${DATASOURCE}" cs_intermittentlake
    ${COMMAND} --area-type FloodLand ${WORKBASE}/Shape-Waterbody "${DATASOURCE}" cs_floodland
    ${COMMAND} --area-type Lagoon ${WORKBASE}/Shape-Waterbody "${DATASOURCE}" cs_lagoon
    ${COMMAND} --area-type Estuary ${WORKBASE}/Shape-Waterbody "${DATASOURCE}" cs_estuary
    ${COMMAND} --area-type Watercourse ${WORKBASE}/Shape-Watercourse "${DATASOURCE}" cs_watercourse
}  # ProcessWaterbody

ProcessCity () {
    # Population areas: cities and towns
    ${COMMAND} --area-type SubUrban ${WORKBASE}/Shape-City "${DATASOURCE}" cs_suburban
    ${COMMAND} --area-type Urban ${WORKBASE}/Shape-City "${DATASOURCE}" cs_urban
    ${COMMAND} --area-type Town ${WORKBASE}/Shape-City "${DATASOURCE}" cs_town
#    ${COMMAND} --area-type Town --point-width 400 ${WORKBASE}/Shape-City "${DATASOURCE}" cs_town
}  # ProcessCity

ProcessCommercial () {
    ${COMMAND} --area-type Industrial ${WORKBASE}/Shape-Commercial "${DATASOURCE}" cs_industrial
    ${COMMAND} --area-type Construction ${WORKBASE}/Shape-Commercial "${DATASOURCE}" cs_construction
    ${COMMAND} --area-type Transport ${WORKBASE}/Shape-Commercial "${DATASOURCE}" cs_transport
    ${COMMAND} --area-type Port ${WORKBASE}/Shape-Commercial "${DATASOURCE}" cs_port
    ${COMMAND} --area-type Fishing ${WORKBASE}/Shape-Commercial "${DATASOURCE}" cs_fishing
}  # ProcessCommercial

ProcessForest () {
    # Forest: deciduous broad, evergreen broad, mixed
    ${COMMAND} --area-type DeciduousForest ${WORKBASE}/Shape-Forest "${DATASOURCE}" cs_deciduousforest
    ${COMMAND} --area-type EvergreenForest ${WORKBASE}/Shape-Forest "${DATASOURCE}" cs_evergreenforest
    ${COMMAND} --area-type MixedForest ${WORKBASE}/Shape-Forest "${DATASOURCE}" cs_mixedforest
    ${COMMAND} --area-type RainForest ${WORKBASE}/Shape-Forest "${DATASOURCE}" cs_rainforest
    ${COMMAND} --area-type AgroForest ${WORKBASE}/Shape-Forest "${DATASOURCE}" cs_agroforest
}  # ProcessForest

ProcessLandCover () {
    # Ground cover: sand, tidal, lava, barren, grass, scrub, herb-tundra
    ${COMMAND} --area-type Airport ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_airport
    ${COMMAND} --area-type Rock ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_rock
    ${COMMAND} --area-type Dirt ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_dirt
    ${COMMAND} --area-type BarrenCover ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_barrencover
    ${COMMAND} --area-type GolfCourse ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_golfcourse
    ${COMMAND} --area-type Cemetery ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_cemetery
    ${COMMAND} --area-type Grassland ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_grassland
    ${COMMAND} --area-type Greenspace ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_greenspace
    ${COMMAND} --area-type OpenMining ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_openmining
    ${COMMAND} --area-type Dump ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_dump
    ${COMMAND} --area-type Lava ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_lava
    ${COMMAND} --area-type Sand ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_sand
    ${COMMAND} --area-type Saline ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_saline
    ${COMMAND} --area-type Scrub ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_scrub
    ${COMMAND} --area-type Heath ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_heath
    ${COMMAND} --area-type Sclerophyllous ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_sclerophyllous
    ${COMMAND} --area-type Burnt ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_burnt
    ${COMMAND} --area-type HerbTundra ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_herbtundra

    # Concrete walls
    ${COMMAND} --area-type Asphalt ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_asphalt

    # Uncovered areas
    ${COMMAND} --area-type Scrub ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_void
}  # ProcessLandCover

ProcessIce () {
    # Ice cover: glaciers, pack ice, and sea ice
    ${COMMAND} --area-type Glacier ${WORKBASE}/Shape-Ice "${DATASOURCE}" cs_glacier
    ${COMMAND} --area-type PackIce ${WORKBASE}/Shape-Ice "${DATASOURCE}" cs_packice
    ${COMMAND} --area-type PolarIce ${WORKBASE}/Shape-Ice "${DATASOURCE}" cs_polarice
}  # ProcessIce

ProcessFloodland () {
    # Marshes: marsh and bog
    ${COMMAND} --area-type Bog ${WORKBASE}/Shape-Floodland "${DATASOURCE}" cs_bog
    ${COMMAND} --area-type Marsh ${WORKBASE}/Shape-Floodland "${DATASOURCE}" cs_marsh
    ${COMMAND} --area-type Littoral ${WORKBASE}/Shape-Floodland "${DATASOURCE}" cs_littoral
    ${COMMAND} --area-type SaltMarsh ${WORKBASE}/Shape-Floodland "${DATASOURCE}" cs_saltmarsh
}  # ProcessFloodland

ProcessAgro () {
    # Crops: mixed pasture, dry crop, irrigated crop
    ${COMMAND} --area-type CropGrass ${WORKBASE}/Shape-Agro "${DATASOURCE}" cs_cropgrass
    ${COMMAND} --area-type DryCrop ${WORKBASE}/Shape-Agro "${DATASOURCE}" cs_drycrop
    ${COMMAND} --area-type IrrCrop ${WORKBASE}/Shape-Agro "${DATASOURCE}" cs_irrcrop
    ${COMMAND} --area-type Rice ${WORKBASE}/Shape-Agro "${DATASOURCE}" cs_rice
    ${COMMAND} --area-type MixedCrop ${WORKBASE}/Shape-Agro "${DATASOURCE}" cs_mixedcrop
    ${COMMAND} --area-type ComplexCrop ${WORKBASE}/Shape-Agro "${DATASOURCE}" cs_complexcrop
    ${COMMAND} --area-type NaturalCrop ${WORKBASE}/Shape-Agro "${DATASOURCE}" cs_naturalcrop
    ${COMMAND} --area-type Vineyard ${WORKBASE}/Shape-Agro "${DATASOURCE}" cs_vineyard
    ${COMMAND} --area-type Orchard ${WORKBASE}/Shape-Agro "${DATASOURCE}" cs_orchard
    ${COMMAND} --area-type Olives ${WORKBASE}/Shape-Agro "${DATASOURCE}" cs_olives
}  # ProcessAgro

# OSM lines expect "SET tunnel = NULL WHERE tunnel LIKE '' OR tunnel ILIKE 'no' OR tunnel ILIKE 'false'"
#              and "SET bridge = NULL WHERE bridge LIKE '' OR bridge ILIKE 'no' OR bridge ILIKE 'false'"
# Get linear textures with "--texture-lines"
# Fetch line width from OSM tables via "--line-width-column width", but
#     requires pre-formatting width columns, as they may contain random
#     garbage
ProcessRoad () {
    # Roads: highway, freeway, trail
    ${COMMAND} --where "tunnel IS NULL AND bridge IS NULL" --area-type Freeway --line-width 11m ${WORKBASE}/Shape-Road "${DATASOURCE}" osm_motorway || echo "ERROR on osm_motorway: ${?}"
    ${COMMAND} --where "tunnel IS NULL AND bridge IS NULL" --area-type Freeway --line-width 7m ${WORKBASE}/Shape-Road "${DATASOURCE}" osm_trunk || echo "ERROR on osm_trunk: ${?}"
    ${COMMAND} --where "tunnel IS NULL AND bridge IS NULL" --area-type Freeway --line-width 9m ${WORKBASE}/Shape-Road "${DATASOURCE}" osm_primary || echo "ERROR on osm_primary: ${?}"
    ${COMMAND} --where "tunnel IS NULL AND bridge IS NULL" --area-type Road --line-width 7m ${WORKBASE}/Shape-Road "${DATASOURCE}" osm_secondary || echo "ERROR on osm_secondary: ${?}"
#    ${COMMAND} --where "tunnel IS NULL AND bridge IS NULL" --area-type Road --line-width 5m ${WORKBASE}/Shape-Road "${DATASOURCE}" osm_tertiary || echo "ERROR on osm_tertiary: ${?}"
#    ${COMMAND} --where "tunnel IS NULL AND bridge IS NULL" --area-type Road --line-width 4m ${WORKBASE}/Shape-Road "${DATASOURCE}" osm_residential || echo "ERROR on osm_residential: ${?}"
#    ${COMMAND} --where "tunnel IS NULL AND bridge IS NULL" --area-type Road --line-width 4m ${WORKBASE}/Shape-Road "${DATASOURCE}" osm_unclassified || echo "ERROR on osm_unclassified: ${?}"
#    ${COMMAND} --where "tunnel IS NULL AND bridge IS NULL" --area-type Road --line-width 3m ${WORKBASE}/Shape-Road "${DATASOURCE}" osm_service || echo "ERROR on osm_service: ${?}"
    ${COMMAND} --where "tunnel IS NULL AND bridge IS NULL" --area-type Freeway --line-width 9m ${WORKBASE}/Shape-Road "${DATASOURCE}" osm_raceway || echo "ERROR on osm_raceway: ${?}"
}  # ProcessRoad

ProcessRailroad () {
    # Railroads: single and multiple tracks
    ${COMMAND} --where "tunnel IS NULL AND bridge IS NULL" --area-type Railroad --line-width 5m ${WORKBASE}/Shape-Railroad "${DATASOURCE}" osm_rail || echo "ERROR on osm_rail: ${?}"
#    ${COMMAND} --where "tunnel IS NULL AND bridge IS NULL" --area-type Railroad --line-width 2m ${WORKBASE}/Shape-Railroad "${DATASOURCE}" osm_tram || echo "ERROR on osm_tram: ${?}"
}  # ProcessRailroad

ProcessStream () {
    # Inland moving water: rivers/streams, intermittent streams, and canals
    ${COMMAND} --where "tunnel IS NULL AND bridge IS NULL" --area-type Stream --line-width 3 ${WORKBASE}/Shape-Stream "${DATASOURCE}" osm_drain || echo "ERROR on osm_drain: ${?}"
    ${COMMAND} --where "tunnel IS NULL AND bridge IS NULL" --area-type Stream --line-width 12 ${WORKBASE}/Shape-Stream "${DATASOURCE}" osm_river || echo "ERROR on osm_river: ${?}"
    ${COMMAND} --where "tunnel IS NULL AND bridge IS NULL" --area-type Canal --line-width 12 ${WORKBASE}/Shape-Stream "${DATASOURCE}" osm_canal || echo "ERROR on osm_canal: ${?}"
#
#    ${COMMAND} --area-type Freeway ${WORKBASE}/Shape-RoadCover "${DATASOURCE}" osm_road_cover
}  # ProcessStream

#ProcessLandmass

# ProcessWaterbody
# ProcessCity
# ProcessCommercial
# ProcessForest
#
# ProcessLandCover
# ProcessIce
# ProcessFloodland
# ProcessAgro
#
# ProcessRoad
# ProcessRailroad
# ProcessStream
# ProcessPolygon
# ProcessLine

CALLNAME=`basename ${0}`
${CALLNAME}

# EOF

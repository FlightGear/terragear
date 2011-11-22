#!/bin/bash

ROAD_TYPE="GPL"
WORKBASE=${HOME}/workdirs

# Base Package
#SPAT="--spat -124 36 -120 39"
# World
SPAT=""

DATASOURCE="PG:dbname=${PGDATABASE} host=${PGHOST} user=${PGUSER}"
OGRDECODE="${HOME}/terragear/bin/ogr-decode --continue-on-errors --max-segment 400 ${SPAT}"

ProcessLandmass () {
    # World land mass
    ${OGRDECODE} --area-type Default ${WORKBASE}/Shape-LandMass "${DATASOURCE}" v0_landmass
    
    # Void-filling
#    ${OGRDECODE} --area-type Default ${WORKBASE}/Shape-LandMass "${DATASOURCE}" cs_default
}  # ProcessLandmass

ProcessPolygon () {
    # Inland still water: lakes, intermittent lakes, and flood land
    ${OGRDECODE} --area-type Lake ${WORKBASE}/Shape-Lakes "${DATASOURCE}" cs_lake
    ${OGRDECODE} --area-type IntermittentLake ${WORKBASE}/Shape-Lakes "${DATASOURCE}" cs_intermittentlake
    ${OGRDECODE} --area-type FloodLand ${WORKBASE}/Shape-Lakes "${DATASOURCE}" cs_floodland
    ${OGRDECODE} --area-type Lagoon ${WORKBASE}/Shape-Lakes "${DATASOURCE}" cs_lagoon
    ${OGRDECODE} --area-type Estuary ${WORKBASE}/Shape-Lakes "${DATASOURCE}" cs_estuary
    ${OGRDECODE} --area-type Watercourse ${WORKBASE}/Shape-Rivers "${DATASOURCE}" cs_watercourse

    # Population areas: cities and towns
    ${OGRDECODE} --area-type SubUrban ${WORKBASE}/Shape-Cities "${DATASOURCE}" cs_suburban
    ${OGRDECODE} --area-type Urban ${WORKBASE}/Shape-Cities "${DATASOURCE}" cs_urban
    ${OGRDECODE} --area-type Town ${WORKBASE}/Shape-Cities "${DATASOURCE}" cs_town
    ${OGRDECODE} --area-type Town --point-width 400 ${WORKBASE}/Shape-Cities "${DATASOURCE}" v0_town
    ${OGRDECODE} --area-type Industrial ${WORKBASE}/Shape-Cities "${DATASOURCE}" cs_industrial
    ${OGRDECODE} --area-type Construction ${WORKBASE}/Shape-Cities "${DATASOURCE}" cs_construction
    ${OGRDECODE} --area-type Transport ${WORKBASE}/Shape-Cities "${DATASOURCE}" cs_transport
    ${OGRDECODE} --area-type Port ${WORKBASE}/Shape-Cities "${DATASOURCE}" cs_port
    
    # Forest: deciduous broad, evergreen broad, mixed
    ${OGRDECODE} --area-type DeciduousForest ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_deciduousforest
    ${OGRDECODE} --area-type EvergreenForest ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_evergreenforest
    ${OGRDECODE} --area-type MixedForest ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_mixedforest
    ${OGRDECODE} --area-type RainForest ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_rainforest
    
    # Ground cover: sand, tidal, lava, barren, grass, scrub, herb-tundra
    ${OGRDECODE} --area-type Airport ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_airport
    ${OGRDECODE} --area-type Rock ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_rock
    ${OGRDECODE} --area-type Dirt ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_dirt
    ${OGRDECODE} --area-type BarrenCover ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_barrencover
    ${OGRDECODE} --area-type GolfCourse ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_golfcourse
    ${OGRDECODE} --area-type Cemetery ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_cemetery
    ${OGRDECODE} --area-type Grassland ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_grassland
    ${OGRDECODE} --area-type Greenspace ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_greenspace
    ${OGRDECODE} --area-type OpenMining ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_openmining
    ${OGRDECODE} --area-type Dump ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_dump
    ${OGRDECODE} --area-type Lava ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_lava
    ${OGRDECODE} --area-type Sand ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_sand
    ${OGRDECODE} --area-type Saline ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_saline
    ${OGRDECODE} --area-type Scrub ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_scrub
    ${OGRDECODE} --area-type Heath ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_heath
    ${OGRDECODE} --area-type Sclerophyllous ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_sclerophyllous
    ${OGRDECODE} --area-type Burnt ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_burnt
    ${OGRDECODE} --area-type HerbTundra ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_herbtundra
    ${OGRDECODE} --area-type AgroForest ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_agroforest

    # Marshes: marsh and bog
    ${OGRDECODE} --area-type Bog ${WORKBASE}/Shape-Floodland "${DATASOURCE}" cs_bog
    ${OGRDECODE} --area-type Marsh ${WORKBASE}/Shape-Floodland "${DATASOURCE}" cs_marsh
    ${OGRDECODE} --area-type Littoral ${WORKBASE}/Shape-Floodland "${DATASOURCE}" cs_littoral
    ${OGRDECODE} --area-type SaltMarsh ${WORKBASE}/Shape-Floodland "${DATASOURCE}" cs_saltmarsh
    
    # Ice cover: glaciers, pack ice, and sea ice
    ${OGRDECODE} --area-type Glacier ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_glacier
    ${OGRDECODE} --area-type PackIce ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_packice
    ${OGRDECODE} --area-type PolarIce ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_polarice
    
    # Crops: mixed pasture, dry crop, irrigated crop
    ${OGRDECODE} --area-type CropGrass ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_cropgrass
    ${OGRDECODE} --area-type DryCrop ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_drycrop
    ${OGRDECODE} --area-type IrrCrop ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_irrcrop
    ${OGRDECODE} --area-type Rice ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_rice
    ${OGRDECODE} --area-type MixedCrop ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_mixedcrop
    ${OGRDECODE} --area-type ComplexCrop ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_complexcrop
    ${OGRDECODE} --area-type DryCrop ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_naturalcrop
    ${OGRDECODE} --area-type Vineyard ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_vineyard
    ${OGRDECODE} --area-type Orchard ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_orchard
    ${OGRDECODE} --area-type Olives ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_olives
    
    # Concrete walls
    ${OGRDECODE} --area-type Asphalt ${WORKBASE}/Shape-LandCover "${DATASOURCE}" cs_asphalt
}  # ProcessPolygon

ProcessLine () {
    # Roads: highway, freeway, trail
    # Railroads: single and multiple tracks
    # Inland moving water: rivers/streams, intermittent streams, and canals
    if [ ${ROAD_TYPE} = "GPL" ]; then
      ${OGRDECODE} --area-type Freeway --line-width 50m ${WORKBASE}/Shape-Roads "${DATASOURCE}" cs_freeway
      ${OGRDECODE} --area-type Road --line-width 20m ${WORKBASE}/Shape-Roads "${DATASOURCE}" cs_road
      #
      ${OGRDECODE} --area-type Railroad --line-width 20m ${WORKBASE}/Shape-Railroads "${DATASOURCE}" cs_railroad2
      ${OGRDECODE} --area-type Railroad --line-width 10m ${WORKBASE}/Shape-Railroads "${DATASOURCE}" cs_railroad1
      #
      ${OGRDECODE} --area-type Stream --line-width 40 ${WORKBASE}/Shape-Rivers "${DATASOURCE}" cs_stream
      ${OGRDECODE} --area-type IntermittentStream --line-width 40 ${WORKBASE}/Shape-Rivers "${DATASOURCE}" cs_intermittentstream
      ${OGRDECODE} --area-type Canal --line-width 50 ${WORKBASE}/Shape-Canals "${DATASOURCE}" cs_canal
    fi
    if [ ${ROAD_TYPE} = "OSM" ]; then
      ${OGRDECODE} --area-type Freeway --line-width 11m ${WORKBASE}/Shape-Roads "${DATASOURCE}" osm_motorway || echo "ERROR on osm_motorway: ${?}"
      ${OGRDECODE} --area-type Freeway --line-width 7m ${WORKBASE}/Shape-Roads "${DATASOURCE}" osm_trunk || echo "ERROR on osm_trunk: ${?}"
      ${OGRDECODE} --area-type Freeway --line-width 9m ${WORKBASE}/Shape-Roads "${DATASOURCE}" osm_primary || echo "ERROR on osm_primary: ${?}"
      ${OGRDECODE} --area-type Road --line-width 7m ${WORKBASE}/Shape-Roads "${DATASOURCE}" osm_secondary || echo "ERROR on osm_secondary: ${?}"
      ${OGRDECODE} --area-type Road --line-width 5m ${WORKBASE}/Shape-Roads "${DATASOURCE}" osm_tertiary || echo "ERROR on osm_tertiary: ${?}"
      #
      ${OGRDECODE} --area-type Railroad --line-width 5m ${WORKBASE}/Shape-Railroads "${DATASOURCE}" osm_rail || echo "ERROR on osm_rail: ${?}"
      #
      ${OGRDECODE} --area-type Stream --line-width 3 ${WORKBASE}/Shape-Rivers "${DATASOURCE}" osm_drain || echo "ERROR on osm_drain: ${?}"
      ${OGRDECODE} --area-type Stream --line-width 12 ${WORKBASE}/Shape-Rivers "${DATASOURCE}" osm_river || echo "ERROR on osm_river: ${?}"
      ${OGRDECODE} --area-type Canal --line-width 12 ${WORKBASE}/Shape-Canals "${DATASOURCE}" osm_canal || echo "ERROR on osm_canal: ${?}"
    fi
    if [ ${ROAD_TYPE} = "OSMcover" ]; then
      ${OGRDECODE} --area-type Freeway ${WORKBASE}/Shape-LandCover "${DATASOURCE}" osm_road_cover
    fi
}  # ProcessLine

ProcessLandmass
ProcessPolygon
ProcessLine

# EOF

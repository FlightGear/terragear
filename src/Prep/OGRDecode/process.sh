#!/bin/bash

# Put your datasource here
DATASOURCE="PG:dbname=$PGDATABASE host=$PGHOST user=$PGUSER"

WORKBASE=$HOME/workdir

OGRDECODE="$HOME/install_headless/bin/ogr-decode --continue-on-errors --max-segment 400 $*"

# World land mass
${OGRDECODE} --area-type Default ${WORKBASE}/Shape-LandMass "${DATASOURCE}" v0_landmass

# Inland moving water: rivers/streams, intermittent streams, and canals
${OGRDECODE} --area-type Stream --line-width 40 ${WORKBASE}/Shape-Rivers "${DATASOURCE}" v0_stream
${OGRDECODE} --area-type IntermittentStream --line-width 30 ${WORKBASE}/Shape-Rivers "${DATASOURCE}" v0_intermittentstream
${OGRDECODE} --area-type Canal --line-width 50 ${WORKBASE}/Shape-Canals "${DATASOURCE}" v0_canal

# Inland still water: lakes, intermittent lakes, and flood land
${OGRDECODE} --area-type Lake ${WORKBASE}/Shape-Lakes "${DATASOURCE}" v0_lake
${OGRDECODE} --area-type IntermittentLake ${WORKBASE}/Shape-Lakes "${DATASOURCE}" v0_intermittentlake
${OGRDECODE} --area-type FloodLand ${WORKBASE}/Shape-Floodland "${DATASOURCE}" v0_floodland

# Population areas: cities and towns
${OGRDECODE} --area-type Urban ${WORKBASE}/Shape-Cities "${DATASOURCE}" v0_urban
${OGRDECODE} --area-type Town --point-width 400 ${WORKBASE}/Shape-Towns "${DATASOURCE}" v0_town

# Forest: deciduous broad, evergreen broad, mixed
${OGRDECODE} --area-type DeciduousBroadCover ${WORKBASE}/Shape-LandCover "${DATASOURCE}" v0_deciduousbroadcover
${OGRDECODE} --area-type EvergreenBroadCover ${WORKBASE}/Shape-LandCover "${DATASOURCE}" v0_evergreenbroadcover
${OGRDECODE} --area-type MixedForestCover ${WORKBASE}/Shape-LandCover "${DATASOURCE}" v0_mixedforestcover

# Ground cover: sand, tidal, lava, barren, grass, scrub, herb-tundra
${OGRDECODE} --area-type Sand ${WORKBASE}/Shape-LandCover "${DATASOURCE}" v0_sand
${OGRDECODE} --area-type Littoral ${WORKBASE}/Shape-LandCover "${DATASOURCE}" v0_marsh
${OGRDECODE} --area-type Lava ${WORKBASE}/Shape-LandCover "${DATASOURCE}" v0_lava
${OGRDECODE} --area-type BarrenCover ${WORKBASE}/Shape-LandCover "${DATASOURCE}" v0_barrencover
${OGRDECODE} --area-type GrassCover ${WORKBASE}/Shape-LandCover "${DATASOURCE}" v0_grasscover
${OGRDECODE} --area-type ScrubCover ${WORKBASE}/Shape-LandCover "${DATASOURCE}" v0_scrubcover
${OGRDECODE} --area-type HerbTundraCover ${WORKBASE}/Shape-LandCover "${DATASOURCE}" v0_herbtundracover

# Ice cover: glaciers, pack ice, and sea ice
${OGRDECODE} --area-type Glacier ${WORKBASE}/Shape-LandCover "${DATASOURCE}" v0_glacier
${OGRDECODE} --area-type PackIce ${WORKBASE}/Shape-LandCover "${DATASOURCE}" v0_packice
${OGRDECODE} --area-type PolarIce ${WORKBASE}/Shape-LandCover "${DATASOURCE}" v0_polarice

# Marshes: marsh and bog
${OGRDECODE} --area-type Marsh ${WORKBASE}/Shape-LandCover "${DATASOURCE}" v0_marsh
${OGRDECODE} --area-type Bog ${WORKBASE}/Shape-LandCover "${DATASOURCE}" v0_bog

# Crops: mixed pasture, dry crop, irrigated crop
${OGRDECODE} --area-type MixedCropPastureCover ${WORKBASE}/Shape-LandCover "${DATASOURCE}" v0_mixedcroppasturecover
${OGRDECODE} --area-type DryCropPastureCover ${WORKBASE}/Shape-LandCover "${DATASOURCE}" v0_drycroppasturecover
${OGRDECODE} --area-type IrrCropPastureCover ${WORKBASE}/Shape-LandCover "${DATASOURCE}" v0_irrcroppasturecover

# Roads: highway, freeway, trail
${OGRDECODE} --area-type Road --line-width 20m ${WORKBASE}/Shape-Roads "${DATASOURCE}" v0_road
${OGRDECODE} --area-type Freeway --line-width 50m ${WORKBASE}/Shape-Roads "${DATASOURCE}" v0_freeway

# Railroads: single and multiple tracks
${OGRDECODE} --area-type Railroad --line-width 10m ${WORKBASE}/Shape-Railroads "${DATASOURCE}" v0_railroad1
${OGRDECODE} --area-type Railroad --line-width 20m ${WORKBASE}/Shape-Railroads "${DATASOURCE}" v0_railroad2


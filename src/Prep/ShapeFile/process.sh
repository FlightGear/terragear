#!/bin/bash

# Put the directory with your extracted shapefiles here
SHAPEBASE=/stage/fgfs05/curt/RawData/Collections/VPF-ShapeFile-User

WORKBASE=/stage/fgfs05/curt/Work

SHAPEDECODE="./shape-decode --continue-on-errors --max-segment 400"

# World land mass
${SHAPEDECODE} ${SHAPEBASE}/landmass_default ${WORKBASE}/Shape-LandMass Default

# Inland moving water: rivers/streams, intermittent streams, and canals
${SHAPEDECODE} --line-width 40 ${SHAPEBASE}/rivers_stream ${WORKBASE}/Shape-Rivers Stream
${SHAPEDECODE} --line-width 30 ${SHAPEBASE}/rivers_intermittentstream ${WORKBASE}/Shape-Rivers IntermittentStream
${SHAPEDECODE} --line-width 50 ${SHAPEBASE}/canals_stream ${WORKBASE}/Shape-Canals Stream

# Inland still water: lakes, intermittent lakes, and flood land
${SHAPEDECODE} ${SHAPEBASE}/lakes_lake ${WORKBASE}/Shape-Lakes Lake
${SHAPEDECODE} ${SHAPEBASE}/lakes_intermittentlake ${WORKBASE}/Shape-Lakes IntermittentLake
${SHAPEDECODE} ${SHAPEBASE}/floodland_floodland ${WORKBASE}/Shape-Floodland FloodLand

# Population areas: cities and towns
${SHAPEDECODE} ${SHAPEBASE}/cities_urban ${WORKBASE}/Shape-Cities Urban
${SHAPEDECODE} --point-width 400 ${SHAPEBASE}/towns_town ${WORKBASE}/Shape-Towns Town

# Forest: deciduous broad, evergreen broad, mixed
${SHAPEDECODE} ${SHAPEBASE}/landcover_deciduousbroadcover ${WORKBASE}/Shape-LandCover DeciduousBroadCover
${SHAPEDECODE} ${SHAPEBASE}/landcover_evergreenbroadcover ${WORKBASE}/Shape-LandCover EvergreenBroadCover
${SHAPEDECODE} ${SHAPEBASE}/landcover_mixedforestcover ${WORKBASE}/Shape-LandCover MixedForestCover

# Ground cover: sand, tidal, lava, barren, grass, shrub, herb-tundra
${SHAPEDECODE} ${SHAPEBASE}/landcover_sand ${WORKBASE}/Shape-LandCover Sand
${SHAPEDECODE} ${SHAPEBASE}/landcover_marsh ${WORKBASE}/Shape-LandCover Marsh
${SHAPEDECODE} ${SHAPEBASE}/landcover_lava ${WORKBASE}/Shape-LandCover Lava
${SHAPEDECODE} ${SHAPEBASE}/landcover_barrencover ${WORKBASE}/Shape-LandCover BarrenCover
${SHAPEDECODE} ${SHAPEBASE}/landcover_grasscover ${WORKBASE}/Shape-LandCover GrassCover
${SHAPEDECODE} ${SHAPEBASE}/landcover_shrubcover ${WORKBASE}/Shape-LandCover ShrubCover
${SHAPEDECODE} ${SHAPEBASE}/landcover_herbtundracover ${WORKBASE}/Shape-LandCover HerbTundraCover

# Ice cover: glaciers, pack ice, and sea ice
${SHAPEDECODE} ${SHAPEBASE}/landcover_glacier ${WORKBASE}/Shape-LandCover Glacier
${SHAPEDECODE} ${SHAPEBASE}/landcover_packice ${WORKBASE}/Shape-LandCover PackIce
# FiXME: Two glacier themes; have same name in database
# ${SHAPEDECODE} ${SHAPEBASE}/landcover_glacier ${WORKBASE}/Shape-LandCover Glacier

# Marshes: marsh and bog
${SHAPEDECODE} ${SHAPEBASE}/landcover_marsh ${WORKBASE}/Shape-LandCover Marsh
${SHAPEDECODE} ${SHAPEBASE}/landcover_bog ${WORKBASE}/Shape-LandCover Bog

# Crops: mixed pasture, dry crop, irrigated crop
${SHAPEDECODE} ${SHAPEBASE}/landcover_mixedcroppasturecover ${WORKBASE}/Shape-LandCover MixedCropPastureCover
${SHAPEDECODE} ${SHAPEBASE}/landcover_drycroppasturecover ${WORKBASE}/Shape-LandCover DryCropPastureCover
${SHAPEDECODE} ${SHAPEBASE}/landcover_irrcroppasturecover ${WORKBASE}/Shape-LandCover IrrCropPastureCover

# Roads: highway, freeway, trail
${SHAPEDECODE} --line-width 20m ${SHAPEBASE}/roads_road ${WORKBASE}/Shape-Roads Road
${SHAPEDECODE} --line-width 50m ${SHAPEBASE}/roads_freeway ${WORKBASE}/Shape-Roads Freeway

# Railroads: single and multiple tracks
${SHAPEDECODE} --line-width 10m ${SHAPEBASE}/railroads_railroad10 ${WORKBASE}/Shape-Railroads Railroad
${SHAPEDECODE} --line-width 20m ${SHAPEBASE}/railroads_railroad20 ${WORKBASE}/Shape-Railroads Railroad


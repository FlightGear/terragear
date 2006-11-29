#!/bin/bash

VPFBASE=/stage/fgfs01/curt/RawData/Collections/VPF-Extracted

# WORKBASE=/stage/fgfs04/curt/Work/VPF
WORKBASE=/fgfs04/curt/Work/

TGVPF="./tgvpf"

# REGIONS="noamer eurnasia soamafr sasaus"
REGIONS="sasaus"


for REGION in $REGIONS; do
    echo Working on $REGION
    case "$REGION" in
        "noamer" )
            V0PATH=${VPFBASE}/v0noa/vmaplv0
            ;;
        "eurnasia" )
            V0PATH=${VPFBASE}/v0eur/vmaplv0
            ;;
        "soamafr" )
            V0PATH=${VPFBASE}/v0soa/vmaplv0
            ;;
        "sasaus" )
            V0PATH=${VPFBASE}/v0sas/vmaplv0
            ;;
        * )
            echo "Error: unrecognized region"
            exit 0
            ;;
    esac
    echo $V0PATH

    # World land mass
    # ${TGVPF} --work-dir=${WORKBASE}/VPF-LandMass --material=Default --max-segment=400 $V0PATH $REGION bnd polbnda

    # Inland moving water: rivers/streams, intermittent streams, and canals
    ${TGVPF} --work-dir=${WORKBASE}/VPF-Rivers --material=Stream --width=40 --att=hyc:8 --max-segment=400 $V0PATH $REGION hydro watrcrsl
    ${TGVPF} --work-dir=${WORKBASE}/VPF-Rivers --material=IntermittentStream --width=30 --att=hyc:6 --max-segment=400 $V0PATH $REGION hydro watrcrsl
    ${TGVPF} --work-dir=${WORKBASE}/VPF-Canals --material=Canal --width=50 --att=exs:1 --att=loc:8 --max-segment=400 $V0PATH $REGION hydro aquecanl

    # Inland still water: lakes, intermittent lakes, and flood land
    ${TGVPF} --work-dir=${WORKBASE}/VPF-Lakes --material=Lake --att=f_code:BH000 --att=hyc:8 --max-segment=400 $V0PATH $REGION hydro inwatera
    ${TGVPF} --work-dir=${WORKBASE}/VPF-Lakes --material=IntermittentLake --att=f_code:BH000 --att=hyc:6 --max-segment=400 $V0PATH $REGION hydro inwatera
    ${TGVPF} --work-dir=${WORKBASE}/VPF-FloodLand --material=FloodLand --att=f_code:BH090 --max-segment=400 $V0PATH $REGION hydro inwatera

    # Population areas: cities and towns
    ${TGVPF} --work-dir=${WORKBASE}/VPF-Cities --material=Urban --max-segment=400 $V0PATH $REGION pop builtupa
    ${TGVPF} --work-dir=${WORKBASE}/VPF-Towns --material=Town --width=400 --max-segment=400 $V0PATH $REGION pop mispopp

    # Forest: deciduous broad, evergreen broad, mixed
    ${TGVPF} --work-dir=${WORKBASE}/VPF-LandCover --material=DeciduousBroadCover --att=veg:24 --max-segment=400 $V0PATH $REGION veg treesa
    ${TGVPF} --work-dir=${WORKBASE}/VPF-LandCover --material=EvergreenBroadCover --att=veg:25 --max-segment=400 $V0PATH $REGION veg treesa
    ${TGVPF} --work-dir=${WORKBASE}/VPF-LandCover --material=MixedForestCover --att=veg:50 --max-segment=400 $V0PATH $REGION veg treesa

    # Ground cover: sand, tidal, lava, barren, grass, shrub, herb-tundra
    ${TGVPF} --work-dir=${WORKBASE}/VPF-LandCover --material=Sand --att=smc:88 --att=swc:0 --max-segment=400 $V0PATH $REGION phys grounda
    ${TGVPF} --work-dir=${WORKBASE}/VPF-LandCover --material=Littoral --att=smc:88 --att=swc:3 --max-segment=400 $V0PATH $REGION phys grounda
    ${TGVPF} --work-dir=${WORKBASE}/VPF-LandCover --material=Lava --att=smc:52 --max-segment=400 $V0PATH $REGION phys grounda
    ${TGVPF} --work-dir=${WORKBASE}/VPF-LandCover --material=BarrenCover --att=smc:119 --max-segment=400 $V0PATH $REGION phys grounda
    ${TGVPF} --work-dir=${WORKBASE}/VPF-LandCover --material=GrassCover --att=f_code:EB010 --max-segment=400 $V0PATH $REGION veg grassa
    ${TGVPF} --work-dir=${WORKBASE}/VPF-LandCover --material=ShrubCover --att=f_code:EB020 --max-segment=400 $V0PATH $REGION veg grassa
    ${TGVPF} --work-dir=${WORKBASE}/VPF-LandCover --material=HerbTundraCover --max-segment=400 $V0PATH $REGION veg tundraa

    # Ice cover: glaciers, pack ice, and sea ice
    ${TGVPF} --work-dir=${WORKBASE}/VPF-LandCover --material=Glacier --max-segment=400 $V0PATH $REGION phys landicea
    ${TGVPF} --work-dir=${WORKBASE}/VPF-LandCover --material=PackIce --att=f_code:BJ070 --max-segment=400 $V0PATH $REGION phys seaicea
    ${TGVPF} --work-dir=${WORKBASE}/VPF-LandCover --material=PolarIce --att=f_code:BJ080 --max-segment=400 $V0PATH $REGION phys seaicea

    # Marshes: marsh and bog
    ${TGVPF} --work-dir=${WORKBASE}/VPF-LandCover --material=Marsh --att=f_code:BH095 --max-segment=400 $V0PATH $REGION veg swampa
    ${TGVPF} --work-dir=${WORKBASE}/VPF-LandCover --material=Bog --att=f_code:BH015 --max-segment=400 $V0PATH $REGION veg swampa

    # Crops: mixed pasture, dry crop, irrigated crop
    ${TGVPF} --work-dir=${WORKBASE}/VPF-LandCover --material=MixedCropPastureCover --att=veg:0 --max-segment=400 $V0PATH $REGION veg cropa
    ${TGVPF} --work-dir=${WORKBASE}/VPF-LandCover --material=DryCropPastureCover --att=veg:1 --max-segment=400 $V0PATH $REGION veg cropa
    ${TGVPF} --work-dir=${WORKBASE}/VPF-LandCover --material=IrrCropPastureCover --att=veg:999 --max-segment=400 $V0PATH $REGION veg cropa

    # Roads: highway, freeway, trail
    ${TGVPF} --work-dir=${WORKBASE}/VPF-Roads --material=Road --width=20m '--att=!med:1' --max-segment=400 $V0PATH $REGION trans roadl
    ${TGVPF} --work-dir=${WORKBASE}/VPF-Roads --material=Freeway --width=50m --att=med:1 --max-segment=400 $V0PATH $REGION trans roadl
    # the following looks off (like we'd never get any data) because allowed values according to the docs are 5 and 28.
    # ${TGVPF} --work-dir=${WORKBASE}/Trails --material=Road --width=5m --att=exs:1 --max-segment=400 $V0PATH $REGION trans traill

    # Railroads: single and multiple tracks
    ${TGVPF} --work-dir=${WORKBASE}/VPF-Railroads --width=10m  --material=Railroad --att=fco:3 --max-segment=400 $V0PATH $REGION trans railrdl
    ${TGVPF} --work-dir=${WORKBASE}/VPF-Railroads --width=20m --material=Railroad --att=fco:2 --max-segment=400 $V0PATH $REGION trans railrdl
done

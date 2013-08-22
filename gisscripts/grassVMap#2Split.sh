#!/bin/bash
#
# Written by Martin Spott
#
# Copyright (C) 2010  Markus Metz @ GRASS GIS
# Copyright (C) 2010 - 2013  Martin Spott - Martin (at) flightgear (dot) org
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

LOADDIR=${HOME}/live/vmap0shp
DUMPDIR=${HOME}/shp
mkdir -p ${DUMPDIR}
cd `dirname ${0}` && export BASEDIR=`pwd`
#
MAPPINGFILE=${BASEDIR}/CORINEtoCS.txt
#
#SNAP=0.00001
SNAP=0.000001
#MIN_AREA=1
MIN_AREA=0.0000001
#MIN_AREA=0.000000001

fn_import() {
    for SHAPEFILE in `ls ${LOADDIR}/*-area.shp`; do
        LAYER=`basename ${SHAPEFILE} | cut -f 1 -d \.`
        MAP=`echo ${LAYER} | sed -e 's/-/_/g'`
        g.remove vect=${MAP}
        v.in.ogr dsn="${LOADDIR}" layer=${LAYER} output=${MAP} snap=${SNAP} --verbose
    done
}

########################################################################

# v.extract input=soa_veg_grassa_area type=area output=v0_grassland where="f_code LIKE 'EB010'"
#
#       sourcelayer        |    sourceattrs     |        pgislayer
#--------------------------+--------------------+--------------------------
# 'builtupa@pop(*)_area'   |                    | v0_urban
# 'cropa@veg(*)_area'      | veg=0              | v0_mixedcrop
# 'cropa@veg(*)_area'      | veg=1              | v0_drycrop
# 'cropa@veg(*)_area'      | veg=999            | v0_irrcrop
# 'grassa@veg(*)_area'     | f_code=EB010       | v0_grassland
# 'grassa@veg(*)_area'     | f_code=EB020       | v0_scrub
# 'grounda@phys(*)_area'   | smc=119            | v0_barrencover
# 'grounda@phys(*)_area'   | smc=52             | v0_lava
# 'grounda@phys(*)_area'   | smc=88:swc=0       | v0_sand
# 'grounda@phys(*)_area'   | smc=88:swc=3       | v0_littoral
# 'inwatera@hydro(*)_area' | f_code=BH000:hyc=6 | v0_intermittentlake
# 'inwatera@hydro(*)_area' | f_code=BH000:hyc=8 | v0_lake
# 'inwatera@hydro(*)_area' | f_code=BH090       | v0_floodland
# 'landicea@phys(*)_area'  |                    | v0_glacier
# 'polbnda@bnd(*)_area'    |                    | v0_landmass
# 'seaicea@phys(*)_area'   | f_code=BJ070       | v0_packice
# 'seaicea@phys(*)_area'   | f_code=BJ080       | v0_polarice
# 'swampa@veg(*)_area'     | f_code=BH015       | v0_bog
# 'swampa@veg(*)_area'     | f_code=BH095       | v0_marsh
# 'treesa@veg(*)_area'     | veg=24             | v0_deciduousforest
# 'treesa@veg(*)_area'     | veg=25             | v0_evergreenforest
# 'treesa@veg(*)_area'     | veg=50             | v0_mixedforest
# 'tundraa@veg(*)_area'    |                    | v0_herbtundra

fn_split() {
    g.remove vect=`g.mlist type=vect pattern="v0_*_???" separator=,`

    for CLASS in `g.mlist type=vect pattern="???_*_*_area" | cut -f 2,3 -d \_ | sort | uniq`; do  # veg_cropa, veg_grassa
        for LAYER in `g.mlist type=vect pattern="???_${CLASS}_area" | sort`; do  # eur_veg_cropa_area, noa_veg_cropa_area
            ZONE=`echo ${LAYER} | cut -f 1 -d \_`  # eur, noa
            V0LAYER=`echo ${LAYER} | awk -F\_ '{print $3 "@" $2 "(*)_" $4}'`  # cropa@veg(*)_area
            case ${V0LAYER} in
                "polbnda@bnd(*)_area")
                    v.extract -t input=${LAYER} type=area output=v0_landmass_${ZONE}
                ;;
                "inwatera@hydro(*)_area")
                    v.extract -t input=${LAYER} type=area output=v0_intermittentlake_${ZONE} where="f_code LIKE 'BH000' AND hyc != 8"
                    v.extract -t input=${LAYER} type=area output=v0_lake_${ZONE} where="f_code LIKE 'BH000' AND hyc = 8"
                    v.extract -t input=${LAYER} type=area output=v0_floodland_${ZONE} where="f_code NOT LIKE 'BH000'"
                ;;
                "extracta@ind(*)_area")
                    v.extract -t input=${LAYER} type=area output=v0_saline_${ZONE} where="f_code LIKE 'BH155'"
                    v.extract -t input=${LAYER} type=area output=v0_openmining_${ZONE} where="f_code NOT LIKE 'BH155'"
                ;;
                "fishinda@ind(*)_area")
                    v.extract -t input=${LAYER} type=area output=v0_fishing_${ZONE}
                ;;
                "grounda@phys(*)_area")
                    v.extract -t input=${LAYER} type=area output=v0_barrencover_${ZONE} where="smc != 52 AND smc != 88"
                    v.extract -t input=${LAYER} type=area output=v0_lava_${ZONE} where="smc = 52"
                    v.extract -t input=${LAYER} type=area output=v0_littoral_${ZONE} where="smc = 88 AND swc = 3"
                    v.extract -t input=${LAYER} type=area output=v0_sand_${ZONE} where="smc = 88 AND swc != 3"
                ;;
                "landicea@phys(*)_area")
                    v.extract -t input=${LAYER} type=area output=v0_glacier_${ZONE}
                ;;
                "seaicea@phys(*)_area")
                    v.extract -t input=${LAYER} type=area output=v0_packice_${ZONE} where="f_code NOT LIKE 'BJ080'"
                    v.extract -t input=${LAYER} type=area output=v0_polarice_${ZONE} where="f_code LIKE 'BJ080'"
                ;;
                "builtupa@pop(*)_area")
                    v.extract -t input=${LAYER} type=area output=v0_urban_${ZONE}
                ;;
                "mispopa@pop(*)_area")
                    v.extract -t input=${LAYER} type=area output=v0_suburban_${ZONE}
                ;;
                "cropa@veg(*)_area")
                    v.extract -t input=${LAYER} type=area output=v0_mixedcrop_${ZONE} where="f_code NOT LIKE 'BH135' and veg = 0"
                    v.extract -t input=${LAYER} type=area output=v0_drycrop_${ZONE} where="f_code NOT LIKE 'BH135' and veg = 1"
                    v.extract -t input=${LAYER} type=area output=v0_irrcrop_${ZONE} where="f_code NOT LIKE 'BH135' and veg > 1"
#                    v.extract -t input=${LAYER} type=area output=v0_rice_${ZONE} where="f_code LIKE 'BH135'"
                ;;
                "grassa@veg(*)_area")
                    v.extract -t input=${LAYER} type=area output=v0_grassland_${ZONE} where="f_code LIKE 'EB010'"
#                    v.extract -t input=${LAYER} type=area output=v0_bamboo_${ZONE} where="f_code LIKE 'EC010'"
                    v.extract -t input=${LAYER} type=area output=v0_scrub_${ZONE} where="f_code NOT LIKE 'EB010' AND f_code NOT LIKE 'EC010'"
                ;;
                "oasisa@veg(*)_area")
                    v.extract -t input=${LAYER} type=area output=v0_oasis_${ZONE}
                ;;
                "orcharda@veg(*)_area")
                    v.extract -t input=${LAYER} type=area output=v0_vineyard_${ZONE} where="f_code LIKE 'EA050'"
                    v.extract -t input=${LAYER} type=area output=v0_orchard_${ZONE} where="f_code NOT LIKE 'EA050'"
                ;;
                "swampa@veg(*)_area")
                    v.extract -t input=${LAYER} type=area output=v0_bog_${ZONE} where="f_code LIKE 'BH015'"
                    v.extract -t input=${LAYER} type=area output=v0_marsh_${ZONE} where="f_code NOT LIKE 'BH015'"
                ;;
                "treesa@veg(*)_area")
#                    v.extract -t input=${LAYER} type=area output=v0_mangrove_${ZONE} where="veg = 19"
                    v.extract -t input=${LAYER} type=area output=v0_deciduousforest_${ZONE} where="veg = 24"
                    v.extract -t input=${LAYER} type=area output=v0_evergreenforest_${ZONE} where="veg = 25"
                    v.extract -t input=${LAYER} type=area output=v0_mixedforest_${ZONE} where="veg != 19 AND veg != 24 AND veg != 25"
                ;;
                "tundraa@veg(*)_area")
                    v.extract -t input=${LAYER} type=area output=v0_herbtundra_${ZONE}
                ;;
                "vegvoida@veg(*)_area")
                    v.extract -t input=${LAYER} type=area output=v0_vegvoid_${ZONE}
                ;;
            esac
        done
    done
}

########################################################################

fn_reclass() {
    for OUTPUT in `g.mlist type=vect pattern="v0_*_???" | awk -F\_ '{print $1 "_" $2}' | sort | uniq`; do
        CATEGORY=`echo ${OUTPUT} | sed -e 's/^v0_/cs_/g'`
        CODECLC=`grep "\ ${CATEGORY}\$" ${MAPPINGFILE} | awk '{print $1}'`
        for ZONE in `g.mlist type=vect pattern="v0_*_[a-z][a-z][a-z]" | awk -F\_ '{print $3}' | sort | uniq`; do
            LCCMAP=${OUTPUT}_${ZONE}_lcclass
            g.remove vect=${LCCMAP}
            v.db.addtable map=${OUTPUT}_${ZONE}
            v.db.addcolumn map=${OUTPUT}_${ZONE} columns="newcodeCLC integer" --verbose
            v.db.update map=${OUTPUT}_${ZONE} column=newcodeCLC value=${CODECLC} --verbose
            v.db.dropcolumn map=${OUTPUT}_${ZONE} column=code_CLC --verbose
            v.db.renamecolumn map=${OUTPUT}_${ZONE} column=newcodeCLC,code_CLC --verbose
            v.reclass input=${OUTPUT}_${ZONE} output=${LCCMAP} column=code_CLC --verbose
        done
#        g.remove vect=${OUTPUT}_patched
        v.patch input=`g.mlist type=vect pattern="${OUTPUT}_[a-z][a-z][a-z]_lcclass" separator=,` output=${OUTPUT}_patched
        g.remove vect=${OUTPUT}_bpol,${OUTPUT}_snap,${OUTPUT}_split,${OUTPUT}_rmsa,${OUTPUT}_rmdangle,${OUTPUT}_rmarea,${OUTPUT}_prune,${OUTPUT}_polyline,${OUTPUT}_dissolved
#
        v.clean input=${OUTPUT}_patched output=${OUTPUT}_bpol -c tool=bpol type=boundary --verbose
        v.clean input=${OUTPUT}_bpol output=${OUTPUT}_snap -c tool=snap thresh=${SNAP} type=boundary --verbose
        v.split input=${OUTPUT}_snap output=${OUTPUT}_split length=40 units=kilometers --verbose
        v.clean input=${OUTPUT}_split output=${OUTPUT}_rmsa -c tool=rmsa type=boundary --verbose
        v.clean input=${OUTPUT}_rmsa output=${OUTPUT}_rmdangle tool=rmline,rmdangle thresh=0,-1 type=boundary --verbose
        date
        v.clean input=${OUTPUT}_rmdangle output=${OUTPUT}_rmarea tool=rmarea thresh=${MIN_AREA} type=boundary --verbose
        date
        v.clean input=${OUTPUT}_rmarea output=${OUTPUT}_prune tool=prune thresh=0.00001 type=boundary --verbose
        v.build.polylines input=${OUTPUT}_prune output=${OUTPUT}_polyline --verbose
        v.dissolve input=${OUTPUT}_polyline output=${OUTPUT}_dissolved --verbose
        v.out.ogr input=${OUTPUT}_dissolved type=area dsn=${DUMPDIR}/${OUTPUT}_pre-clip.shp
    done
}

########################################################################

fn_import
fn_split
fn_reclass

# EOF

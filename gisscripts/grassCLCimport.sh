#!/bin/sh
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

LOADDIR=${HOME}/live/corine
DUMPDIR=${HOME}/shp
mkdir -p ${DUMPDIR}
cd `dirname ${0}` && export BASEDIR=`pwd`
#
MAPPINGFILE=${BASEDIR}/CORINEtoCS.txt

SNAP=0.01
#SNAP=0.001
#SNAP=0.0001
#SNAP=0.000001
MIN_AREA=1
#MIN_AREA=0.000000001

YEAR=06
if [ ${YEAR} = "00" ]; then
    SRCID=18
elif [ ${YEAR} = "06" ]; then
    SRCID=17
else
    SRCID=0
fi
#PREFIX=nl
#PREFIX=clc${YEAR}_nl
PREFIX=clc${YEAR}

# DB-Connection
PGHOST=geoscope.optiputer.net
PGDATABASE=landcover
PGUSER=martin
DSN="PG:host=${PGHOST} dbname=${PGDATABASE} user=${PGUSER}"
LAYEROPTS="FID=ogc_fid, GEOMETRY_NAME=wkb_geometry, SPATIAL_INDEX=YES, PRIMARY_KEY=YES, SRID=4326"
PSQL="psql -tA -h ${PGHOST} -U ${PGUSER} -d ${PGDATABASE}"

fn_fixpostgis() {
    LAYER=${1}
    echo "DROP INDEX ${LAYER}_cat_idx;"
    echo "ALTER INDEX ${LAYER}_pkey RENAME TO ${LAYER}_pk;"
    echo "ALTER INDEX ${LAYER}_wkb_geometry_idx RENAME TO ${LAYER}_gindex;"
    echo "CLUSTER ${LAYER}_gindex ON ${LAYER};"
    echo "ALTER TABLE ${LAYER} ADD CONSTRAINT "enforce_dims_wkb_geometry" CHECK (ST_NDims(wkb_geometry) = 2);"
    echo "ALTER TABLE ${LAYER} ADD CONSTRAINT "enforce_geotype_wkb_geometry" CHECK (GeometryType(wkb_geometry) = 'POLYGON'::text);"
    echo "ALTER TABLE ${LAYER} ADD CONSTRAINT "enforce_srid_wkb_geometry" CHECK (ST_SRID(wkb_geometry) = 4326);"
    echo "ALTER TABLE ${LAYER} DROP COLUMN id;"
    echo "ALTER TABLE ${LAYER} ADD COLUMN src_id numeric(5,0) DEFAULT ${SRCID};"
    echo "ALTER TABLE ${LAYER} ADD COLUMN maint_id numeric(5,0) DEFAULT NULL;"
    echo "ALTER TABLE ${LAYER} ADD COLUMN ch_date timestamp;"
    echo "GRANT SELECT ON ${LAYER} TO webuser;"
    echo "VACUUM ANALYZE ${LAYER};"
}

fn_topostgis() {
    GRMAP=${1}
    PGLAYER=${PREFIX}_`echo ${GRMAP} | cut -f 2 -d \_`
    echo "DROP TABLE ${PGLAYER};" | ${PSQL}
    v.out.postgis input=${GRMAP} olayer=${PGLAYER} dsn="${DSN}" options="${LAYEROPTS}"
    fn_fixpostgis ${PGLAYER} | ${PSQL}
}

# Note: CORINE Shapefiles are EPSG:3035, Landcover-DB is EPSG:4326, but
# _both_ projections (reference systems) are using square meters as unit for
# area size declarations in GRASS !
# Units for snapping are different in GRASS, though. Whereas EPSG:3035 is
# using meters, EPSG:4326 is using degrees.
#

fn_import() {
    for SHAPEFILE in `ls ${LOADDIR}/${PREFIX}_c[0-9][0-9][0-9].shp`; do
        LAYER=`basename ${SHAPEFILE} | cut -f 1 -d \.`
        MAP=`echo ${LAYER} | cut -f 2 -d \_`
        CODECLC=`echo ${MAP} | tr -d c`
        INTMAP=${MAP}_int
        g.remove vect=${MAP}
        v.in.ogr dsn="${LOADDIR}" layer=${LAYER} output=${MAP} snap=${SNAP} --verbose
        v.db.addcolumn map=${MAP} columns="newcodeCLC integer" --verbose
        v.db.update map=${MAP} column=newcodeCLC value=${CODECLC} --verbose
        v.db.dropcolumn map=${MAP} column=code_${YEAR} --verbose
        v.db.renamecolumn map=${MAP} column=newcodeCLC,codeCLC --verbose
        g.remove vect=${INTMAP}
        v.reclass input=${MAP} output=${INTMAP} column=codeCLC --verbose
        v.out.ogr input=${INTMAP} type=area dsn=${DUMPDIR}/${PREFIX}_${MAP}_pre-merge.shp
    done
}

###############################################################################

fn_clean() {
    SELECTION=`g.mlist type=vect pattern="c[0-9][0-9][0-9]_int" separator=,`
    #
    g.remove vect=${PREFIX}_patched,${PREFIX}_bpol,${PREFIX}_snap,${PREFIX}_split,${PREFIX}_rmsa,${PREFIX}_rmdangle,${PREFIX}_rmarea,${PREFIX}_prune,${PREFIX}_polyline,${PREFIX}_dissolved
    v.patch input=${SELECTION} output=${PREFIX}_patched
    #
    v.clean input=${PREFIX}_patched output=${PREFIX}_bpol -c tool=bpol type=boundary --verbose
    v.clean input=${PREFIX}_bpol output=${PREFIX}_snap -c tool=snap thresh=${SNAP} type=boundary --verbose
    v.split input=${PREFIX}_snap output=${PREFIX}_split length=40 units=kilometers --verbose
    v.clean input=${PREFIX}_split output=${PREFIX}_rmsa -c tool=rmsa type=boundary --verbose
    v.clean input=${PREFIX}_rmsa output=${PREFIX}_rmdangle tool=rmline,rmdangle thresh=0,-1 type=boundary --verbose
    date
    v.clean input=${PREFIX}_rmdangle output=${PREFIX}_rmarea tool=rmarea thresh=${MIN_AREA} type=boundary --verbose
    date
    v.clean input=${PREFIX}_rmarea output=${PREFIX}_prune tool=prune thresh=0.00001 type=boundary --verbose
    v.build.polylines input=${PREFIX}_prune output=${PREFIX}_polyline --verbose
    v.dissolve input=${PREFIX}_polyline output=${PREFIX}_dissolved --verbose
}

###############################################################################

fn_proj() {
    # Re-project from EPSG:3035 - this one is _not_ to be run inside
    # the import location !!!
    MYLOCATION=`g.gisenv get=LOCATION_NAME`
    MYMAPSET=`g.gisenv get=MAPSET`
    g.mapset location=wgs84 mapset=${MYMAPSET}
    g.remove vect=${CLEANMAP}
    v.proj location=${MYLOCATION} mapset=${MYMAPSET} input=${CLEANMAP}  # output=${CLEANMAP}
}

fn_export() {
    # Print available (integer) categories - centroids only !!!
    #
    SELECTION=`v.category input=${CLEANMAP} type=centroid option=print | sort -n | uniq`
    for CATEGORY in ${SELECTION}; do
#        LAYER=${PREFIX}_${CATEGORY}
        LAYER=`grep \^${CATEGORY} ${MAPPINGFILE} | awk '{print $2}' | sed -e "s/^cs_/${PREFIX}_/g"`
        g.remove vect=${LAYER}
        v.extract cats=${CATEGORY} input=${CLEANMAP} output=${LAYER} type=area
#        v.out.ogr input=${LAYER} type=area dsn=${DUMPDIR}/${PREFIX}_c${CATEGORY}.shp
        fn_topostgis ${LAYER}
    done
}

########################################################################

fn_import
fn_clean
CLEANMAP=${PREFIX}_dissolved
fn_proj
fn_export

# EOF

#!/bin/bash
#

date

DUMPDIR=${HOME}/shp
mkdir -p ${DUMPDIR}
RUNDIR=`pwd`
cd `dirname ${0}` && export BASEDIR=`pwd`
cd ${RUNDIR}
#
MAPPINGFILE=${BASEDIR}/CORINEtoCS.txt

#SNAP=0.00001
SNAP=0.000001
MIN_AREA=100
#MIN_AREA=0.000000001

PREFIX=v0
CLEANMAP=worldcopy

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
    echo "ALTER TABLE ${LAYER} ADD COLUMN src_id numeric(5,0) DEFAULT 1;"
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

###############################################################################

fn_overlay() {
    COUNT=1
    for LAYER in v0_lake \
                 v0_intermittentlake \
                 v0_floodland \
                 v0_glacier \
                 v0_packice \
                 v0_polarice \
                 v0_urban \
                 v0_suburban \
                 v0_fishing \
                 v0_bog \
                 v0_marsh \
                 v0_saline \
                 v0_openmining \
                 v0_drycrop \
                 v0_irrcrop \
                 v0_mixedcrop \
                 v0_grassland \
                 v0_scrub \
                 v0_deciduousforest \
                 v0_evergreenforest \
                 v0_mixedforest \
                 v0_herbtundra \
                 v0_littoral \
                 v0_barrencover \
                 v0_sand \
                 v0_lava \
                 v0_landmass;
    do
        g.remove vect=${LAYER}_postsplit
        v.split input=${LAYER}_dissolved output=${LAYER}_postsplit length=40 units=kilometers --verbose
        v.out.ogr input=${LAYER}_postsplit type=area dsn=${DUMPDIR}/${LAYER}_post-split.shp
        if [ ${COUNT} -eq 1 ]; then
            g.remove vect=${CLEANMAP}
            g.copy vect=${LAYER}_postsplit,${CLEANMAP}
        else
            g.remove vect=tmp
            v.overlay ainput=${CLEANMAP} binput=${LAYER}_postsplit output=tmp operator=or snap=0.000001
            v.db.addcolumn map=tmp columns="newcat integer" --verbose
            v.db.update map=tmp column=newcat value=b_cat where="a_cat IS NULL" --verbose
            v.db.update map=tmp column=newcat value=a_cat where="b_cat IS NULL" --verbose
            v.db.update map=tmp column=newcat value=a_cat where="newcat IS NULL" --verbose
            v.db.update map=tmp column=newcat value=a_cat where="newcat=0" --verbose
            g.remove vect=${CLEANMAP}_reclass
            v.reclass input=tmp output=${CLEANMAP}_reclass column=newcat --verbose
            g.remove vect=${CLEANMAP}
            g.rename vect=${CLEANMAP}_reclass,${CLEANMAP}
        fi
        COUNT=`expr ${COUNT} + 1`
    done
}

fn_preexport() {
    SELECTION=`v.category input=${CLEANMAP} type=centroid option=print | sort -n | uniq`
    for CATEGORY in ${SELECTION}; do
        LAYER=`grep \^${CATEGORY} ${MAPPINGFILE} | awk '{print $2}' | sed -e "s/^cs_/${PREFIX}_/g"`
        g.remove vect=${LAYER}_postclip
        v.extract cats=${CATEGORY} input=${CLEANMAP} output=${LAYER}_postclip type=area
        v.out.ogr input=${LAYER}_postclip type=area dsn=${DUMPDIR}/${LAYER}_post-clip.shp
    done
}

###############################################################################

fn_clean() {
    # Caution, don't overwrite the only functional working copy while testing !!!
    g.remove vect=${PREFIX}_patched,${PREFIX}_bpol,${PREFIX}_snap,${PREFIX}_rmsa,${PREFIX}_rmdangle,${PREFIX}_rmarea,${PREFIX}_prune,${PREFIX}_polyline,${PREFIX}_dissolved
    g.rename vect=${CLEANMAP},${PREFIX}_patched
    #
    v.clean input=${PREFIX}_patched output=${PREFIX}_bpol -c tool=bpol type=boundary --verbose
    v.clean input=${PREFIX}_bpol output=${PREFIX}_snap -c tool=snap thresh=${SNAP} type=boundary --verbose
    v.clean input=${PREFIX}_snap output=${PREFIX}_rmsa -c tool=rmsa type=boundary --verbose
    v.clean input=${PREFIX}_rmsa output=${PREFIX}_rmdangle tool=rmline,rmdangle thresh=0,-1 type=boundary --verbose
    date
    v.clean input=${PREFIX}_rmdangle output=${PREFIX}_rmarea tool=rmarea thresh=${MIN_AREA} type=boundary --verbose
    date
    v.clean input=${PREFIX}_rmarea output=${PREFIX}_prune tool=prune thresh=0.00001 type=boundary --verbose
    v.build.polylines input=${PREFIX}_prune output=${PREFIX}_polyline --verbose
    v.dissolve input=${PREFIX}_polyline output=${PREFIX}_dissolved --verbose

    #v.split input=${PREFIX}_patched output=${PREFIX}_split layer=-1 vertices=100 --verbose
    #
    #oder kein v.split, kein v.clean tool=bpol, kein v.clean tool=snap, sondern
    #
    #v.clean input=${PREFIX}_patched output=${PREFIX}_rmarea tool=rmarea thresh=100
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
    if [ ${PREFIX} = "v0" ]; then
        g.remove vect=${PREFIX}_landmass_polyline
        v.proj location=${MYLOCATION} mapset=${MYMAPSET} input=${PREFIX}_landmass_polyline
    fi
}

###############################################################################

fn_export() {
    # Print available (integer) categories - centroids only !!!
    #
    SELECTION=`v.category input=${CLEANMAP} type=centroid option=print | sort -n | uniq`
    for CATEGORY in ${SELECTION}; do
        LAYER=`grep \^${CATEGORY} ${MAPPINGFILE} | awk '{print $2}' | sed -e "s/^cs_/${PREFIX}_/g"`
        g.remove vect=${LAYER}
        v.extract cats=${CATEGORY} input=${CLEANMAP} output=${LAYER} type=area
        if [ ${LAYER} = "${PREFIX}_landmass" ]; then
            NEWLAYER="${PREFIX}_void"
            g.remove vect=${NEWLAYER}
            g.rename vect=${LAYER},${NEWLAYER}
            LAYER=${NEWLAYER}
        fi
#        v.out.ogr input=${LAYER} type=area dsn=${DUMPDIR}/${LAYER}.shp
        fn_topostgis ${LAYER}
    done
    if [ ${PREFIX} = "v0" ]; then
#        v.out.ogr input=${PREFIX}_landmass_polyline type=area dsn=${DUMPDIR}/${PREFIX}_landmass.shp
        fn_topostgis ${PREFIX}_landmass_polyline
    fi
}

fn_overlay
fn_preexport
fn_clean
CLEANMAP=${PREFIX}_dissolved
fn_proj
fn_export

# EOF

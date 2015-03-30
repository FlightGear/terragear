#!/bin/bash
#
# Copyright (C) 2011 - 2014  John Holden, Martin Spott
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

# nohup /usr/bin/time env GRASS_BATCH_JOB=/home/martin/landcover/grassNLCD2011full.sh /opt/GRASS7/bin/grass71 ~/grassdata/aea/nlcd2011full/ > & ~/grassNLCD2011full.log &

# ll_w=-130.23282802
# ll_s=21.74230778
# ll_e=-63.67219185
# ll_n=49.17706319
#
# west:       -2493045
# south:      177285
# east:       2342655
# north:      3310005

# To put it simple:
#     S/N: 20/50    - 15 rows a 2° - 208848/row
#     W/E: -131/-63 - 34 cols a 2° - 142226/col

# DB-Connection
PGHOST=eclipse.optiputer.net
PGDATABASE=landcover
PGUSER=martin
DSN="PG:host=${PGHOST} dbname=${PGDATABASE} user=${PGUSER}"
LAYEROPTS="FID=ogc_fid, GEOMETRY_NAME=wkb_geometry, SPATIAL_INDEX=YES, PRIMARY_KEY=YES, SRID=4326"
PSQL="psql -tA -h ${PGHOST} -U ${PGUSER} -d ${PGDATABASE}"

#g.remove -f type=rast name=rast
#r.in.gdal input=/home/martin/live/MRLC/nlcd_2011_landcover_2011_edition_2014_03_31.img output=rast --verbose
#
#g.remove -f type=rast name=rast_null
#r.mapcalc "rast_null = if(rast[0,0]>0,rast[0,0],null())" --verbose
#
#g.remove -f type=rast name=rast_notnull
#r.mapcalc "rast_notnull = if(isnull(rast_null),mode(rast_null[3, -3],rast_null[3, -2],rast_null[3, -1],rast_null[3, 0],rast_null[3, 1],rast_null[3, 2],rast_null[3, 3],rast_null[2, -3],rast_null[2, -2],rast_null[2, -1],rast_null[2, 0],rast_null[2, 1],rast_null[2, 2],rast_null[2, 3],rast_null[1, -3],rast_null[1, -2],rast_null[1, -1],rast_null[1, 0],rast_null[1, 1],rast_null[1, 2],rast_null[1, 3],rast_null[0, -3],rast_null[0, -2],rast_null[0, -1],rast_null[0, 0],rast_null[0, 1],rast_null[0, 2],rast_null[0, 3],rast_null[-1, -3],rast_null[-1, -2],rast_null[-1, -1],rast_null[-1, 0],rast_null[-1, 1],rast_null[-1, 2],rast_null[-1, 3],rast_null[-2, -3],rast_null[-2, -2],rast_null[-2, -1],rast_null[-2, 0],rast_null[-2, 1],rast_null[-2, 2],rast_null[-2, 3],rast_null[-3, -3],rast_null[-3, -2],rast_null[-3, -1],rast_null[-3, 0],rast_null[-3, 1],rast_null[-3, 2],rast_null[-3, 3]), rast_null)" --verbose
#
#g.remove -f type=rast name=rast_n
#r.neighbors -c input=rast_notnull output=rast_n size=3 method=mode --verbose
#
#g.region rast=rast_n -b -g
#r.clump -d input=rast_n output=rast_clump --verbose
## Display map via "d.rast map=rast_clump", manually select ocean areas.
#r.mapcalc "ocean_mask = if(rast_clump == 24692641 || rast_clump == 35447169 || rast_clump == 35675692 || rast_clump == 35709489 || rast_clump == 36616464 || rast_clump == 37135708 || rast_clump == 37475930 || rast_clump == 53970505 || rast_clump == 56788713 || rast_clump == 57552405 || rast_clump == 58118016 || rast_clump == 60549014 || rast_clump == 63128733 || rast_clump == 85468901 || rast_clump == 85468949, 1, null())" --verbose
## Introduce new category ocean=9
#echo "* = 9" | r.reclass input=ocean_mask output=rast_ocean rules=-
#r.mask -i raster=ocean_mask --verbose
#r.mapcalc "rast_land = rast_n"
#r.mask -r raster=ocean_mask --verbose
#r.patch input=rast_land,rast_ocean output=rast_e

## m.proj -i coordinates=<lon>,<lat> separator=,
## Too large:
## "-111 29" "-104 36"
# Better:
# g.region w=-1457168.72 s=773770.48 e=-713870.56 n=1468802.52 --verbose
# "-111 29" "-107 33"

MYLOCATION=`g.gisenv get=LOCATION_NAME`
MYMAPSET=`g.gisenv get=MAPSET`

# Start looping over full NLCD2011 at: W=-131, S=21
# Base Package Scenery at            : W=-124, S=36, E=-120, N=39
S=177285
while [ ${S} -lt 3310005 ]; do
    W=-2493045
    while [ ${W} -lt 2342655 ]; do
        g.mapset location=${MYLOCATION} mapset=${MYMAPSET}

        E=`expr ${W} + 142226`
        N=`expr ${S} + 208848`
        SUFFIX="_`echo ${W}_${S} | tr -d \-`"

        # Clipping rectangle
        W1=`echo ${W} - 256 | bc`
        S1=`echo ${S} - 256 | bc`
        E1=`echo ${E} + 256 | bc`
        N1=`echo ${N} + 256 | bc`

        # Vectorize this
        W2=`echo ${W} - 1024 | bc`
        S2=`echo ${S} - 1024 | bc`
        E2=`echo ${E} + 1024 | bc`
        N2=`echo ${N} + 1024 | bc`

        # Convert lon/lat into map projection:
        LL="${W},${S}"
        UR="${E},${N}"
        read WP SP <<< `m.proj -o -d coordinates=${LL} | awk -F\| '{print $1, $2}'`
        read EP NP <<< `m.proj -o -d coordinates=${UR} | awk -F\| '{print $1, $2}'`

        # Base Package Scenery:
        #g.region w=-2465464.95 s=1804592.77 e=-2037425.60 n=2033555.35 --verbose
        g.region w=${W1} s=${S1} e=${E1} n=${N1} --verbose
        g.region align=rast_e -p --verbose
        v.in.region output=cliprect type=area --verbose --overwrite

        g.region w=${W2} s=${S2} e=${E2} n=${N2} --verbose
        g.region align=rast_e -p --verbose

        echo "### Vectorizing (${WP} ${SP}, ${EP} ${NP}) ### "
        r.to.vect -b -s input=rast_e output=vect_raw_s type=area --verbose --overwrite
        v.build map=vect_raw_s --verbose  # if "r.to.vect -b" was used
        NUMPOLYS=`v.info -t map=vect_raw_s | grep \= | grep -v \=0$ | wc -l`
        if [ ${NUMPOLYS} -eq 0 ]; then
            g.remove -f type=vect pattern=vect_raw_s
        else

            v.db.addcolumn map=vect_raw_s columns="codeNLCD integer" --verbose
            v.db.update map=vect_raw_s column=codeNLCD qcolumn="CAST(value AS integer)" --verbose
            v.db.update map=vect_raw_s column=codeNLCD value=1 where="value=0" --verbose
            v.reclass input=vect_raw_s output=vect_class${SUFFIX} column=codeNLCD --verbose --overwrite

            # Remove ocean layers from land map while reducing resolution in
            # order to prevent small land areas becoming ocean.
            g.copy vect=vect_class${SUFFIX},vect_land --verbose --overwrite
            v.extract -d input=vect_land output=vect_ocean cats=9 --verbose --overwrite
            v.edit map=vect_land tool=delete cats=9 --verbose

            v.clean input=vect_land output=rmarea_05 tool=rmarea thresh=49999 type=boundary --verbose --overwrite
            v.clean input=rmarea_05 output=rmarea_10 tool=rmarea thresh=99999 type=boundary --verbose --overwrite
            v.clean input=rmarea_10 output=rmarea_15 tool=rmarea thresh=149999 type=boundary --verbose --overwrite
            v.clean input=rmarea_15 output=rmarea_20 tool=rmarea thresh=199999 type=boundary --verbose --overwrite
            v.clean input=rmarea_20 output=rmarea_25 tool=rmarea thresh=249999 type=boundary --verbose --overwrite

            # Re-combine "low-resolution" land and ocean, ensure boundaries are clean.
            v.patch input=rmarea_25,vect_ocean output=vect_patched --verbose --overwrite

            v.clean input=vect_patched output=vect_bpol_1 -c tool=bpol type=boundary --verbose --overwrite
            v.clean input=vect_bpol_1 output=vect_snap_1 -c tool=snap thresh=0.01 type=boundary --verbose --overwrite
            v.split input=vect_snap_1 output=vect_split_1 length=40 units=kilometers --verbose --overwrite
            v.clean input=vect_split_1 output=vect_rmsa_1 -c tool=rmsa type=boundary --verbose --overwrite
            v.clean input=vect_rmsa_1 output=vect_rmdangle_1 tool=rmline,rmdangle thresh=0,-1 type=boundary --verbose --overwrite
            v.clean input=vect_rmdangle_1 output=vect_rmarea_1 tool=rmarea thresh=1 type=boundary --verbose --overwrite
            v.clean input=vect_rmarea_1 output=vect_prune_1 tool=prune thresh=0.00001 type=boundary --verbose --overwrite
            v.dissolve input=vect_prune_1 output=vect_dissolved_1${SUFFIX} --verbose --overwrite

            g.mapset location=wgs84 mapset=${MYMAPSET}
            v.proj location=${MYLOCATION} mapset=${MYMAPSET} input=vect_dissolved_1${SUFFIX} --verbose --overwrite
            v.proj location=${MYLOCATION} mapset=${MYMAPSET} input=cliprect --verbose --overwrite

            v.generalize input=vect_dissolved_1${SUFFIX} output=vect_gen${SUFFIX} method=snakes threshold=36 alpha=.2 beta=.2 --verbose --overwrite

            # Ensure boundaries are clean, otherwise "collection" map might contain holes.
            v.clean input=vect_gen${SUFFIX} output=vect_bpol_2 -c tool=bpol type=boundary --verbose --overwrite
            v.clean input=vect_bpol_2 output=vect_snap_2 -c tool=snap thresh=0.00000001 type=boundary --verbose --overwrite
            v.split input=vect_snap_2 output=vect_split_2 length=40 units=kilometers --verbose --overwrite
            v.clean input=vect_split_2 output=vect_rmsa_2 -c tool=rmsa type=boundary --verbose --overwrite
            v.clean input=vect_rmsa_2 output=vect_rmdangle_2 tool=rmline,rmdangle thresh=0,-1 type=boundary --verbose --overwrite
            v.clean input=vect_rmdangle_2 output=vect_rmarea_2 tool=rmarea thresh=0.0000001 type=boundary --verbose --overwrite
            v.clean input=vect_rmarea_2 output=vect_prune_2 tool=prune thresh=0.00001 type=boundary --verbose --overwrite
            v.dissolve input=vect_prune_2 output=vect_dissolved_2 --verbose --overwrite

            v.centroids input=vect_dissolved_2 output=vect_filled cat=9 step=0 --verbose --overwrite
            v.overlay ainput=vect_filled binput=cliprect output=vect_clipped operator=and olayer=0,1,0 --verbose --overwrite

            v.extract -d -t input=vect_clipped type=area output=vect_collect${SUFFIX} new=1 --verbose --overwrite
            v.out.postgis input=vect_collect${SUFFIX} type=area olayer=newcs_collect output="${DSN}" options="${LAYEROPTS}" --verbose --overwrite
#            v.out.ogr input=vect_collect${SUFFIX} type=area olayer=newcs_collect format=PostgreSQL output="${DSN}" --verbose --overwrite
            # For debug only.
#            v.out.ogr input=vect_collect${SUFFIX} type=area output=${HOME}/shp/nlcd2011collect.shp --verbose

            v.db.addtable map=vect_clipped
            v.db.addcolumn map=vect_clipped columns="pglayer varchar" --verbose
            v.db.update map=vect_clipped column=pglayer value=cs_void where="cat=1" --verbose
            v.db.update map=vect_clipped column=pglayer value=cs_ocean where="cat=9" --verbose
            v.db.update map=vect_clipped column=pglayer value=cs_lake where="cat=11" --verbose
            v.db.update map=vect_clipped column=pglayer value=cs_glacier where="cat=12" --verbose
            v.db.update map=vect_clipped column=pglayer value=cs_greenspace where="cat=21" --verbose
            v.db.update map=vect_clipped column=pglayer value=cs_town where="cat=22" --verbose
            v.db.update map=vect_clipped column=pglayer value=cs_suburban where="cat=23" --verbose
            v.db.update map=vect_clipped column=pglayer value=cs_urban where="cat=24" --verbose
            v.db.update map=vect_clipped column=pglayer value=cs_dirt where="cat=31" --verbose
            v.db.update map=vect_clipped column=pglayer value=cs_deciduousforest where="cat=41" --verbose
            v.db.update map=vect_clipped column=pglayer value=cs_evergreenforest where="cat=42" --verbose
            v.db.update map=vect_clipped column=pglayer value=cs_mixedforest where="cat=43" --verbose
            v.db.update map=vect_clipped column=pglayer value=cs_herbtundra where="cat=51" --verbose
            v.db.update map=vect_clipped column=pglayer value=cs_scrub where="cat=52" --verbose
            v.db.update map=vect_clipped column=pglayer value=cs_grassland where="cat=71" --verbose
            v.db.update map=vect_clipped column=pglayer value=cs_drycrop where="cat=81" --verbose
            v.db.update map=vect_clipped column=pglayer value=cs_naturalcrop where="cat=82" --verbose
            v.db.update map=vect_clipped column=pglayer value=cs_floodland where="cat=90" --verbose
            v.db.update map=vect_clipped column=pglayer value=cs_marsh where="cat=95" --verbose

            g.rename vect=vect_clipped,vect_full${SUFFIX} --verbose --overwrite
            v.edit map=vect_full${SUFFIX} tool=delete cats=9 --verbose
#            v.out.postgis input=vect_full${SUFFIX} type=area olayer=newcs_full output="${DSN}" options="${LAYEROPTS}" --verbose --overwrite
            v.out.ogr input=vect_full${SUFFIX} type=area olayer=newcs_full format=PostgreSQL output="${DSN}" --verbose --overwrite
            # For debug only.
#            v.out.ogr input=vect_full${SUFFIX} type=area output=${HOME}/shp/nlcd2011full.shp --verbose
#            ${PSQL} -c "SELECT DISTINCT fn_SceneDir(wkb_geometry), fn_SceneSubDir(wkb_geometry) FROM fgs_objects WHERE ob_id = 533101;"
            ${PSQL} -e -c "SELECT fn_CSMerge('${SUFFIX}');" && \
                echo "### region_tag # `g.region vect=vect_collect${SUFFIX} -b -g | tr "\n" \ `###"

        fi
        W=`expr ${W} + 142226`
    done
    S=`expr ${S} + 208848`
done

# EOF

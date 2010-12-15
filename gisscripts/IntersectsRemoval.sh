#!/bin/bash
#
# Written by Martin Spott
#
# Copyright (C) 2010  Martin Spott - Martin (at) flightgear (dot) org
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

PGHOST=geoscope.optiputer.net
PGUSER=martin
PGDATABASE=landcover
PGSQL2SHPCS="-h ${PGHOST} -u ${PGUSER} -g wkb_geometry -b -r ${PGDATABASE}"
PSQL="psql -tA -h ${PGHOST} -U ${PGUSER} -d ${PGDATABASE} -c"
SHAPE=`basename ${1} | cut -f 1 -d \. | uniq`
CUTLAYER=temphole
DIFFLAYER=tempdiff
LAYERPREFIX=cs_
LOGSCRIPT=${HOME}/cs_intersects.sql
WITHINSCRIPT=${HOME}/cs_withins.sql
WITHOUTSCRIPT=${HOME}/cs_withouts.log
DUMPDIR=${HOME}/shp
rm -f ${LOGSCRIPT} ${WITHINSCRIPT} ${WITHOUTSCRIPT}
mkdir -p ${DUMPDIR}

MODE="testing"  # production, testing

InitCutoutTable () {
    ${PSQL} "DROP TABLE ${CUTLAYER}"
    ${PSQL} "DELETE FROM geometry_columns WHERE f_table_name LIKE '${CUTLAYER}'"
    ${PSQL} "CREATE TABLE ${CUTLAYER} (ogc_fid serial NOT NULL, \
                 wkb_geometry geometry, \
                 CONSTRAINT enforce_dims_wkb_geometry CHECK (ndims(wkb_geometry) = 2), \
                 CONSTRAINT enforce_geotype_wkb_geometry CHECK ((geometrytype(wkb_geometry) = 'POLYGON'::text) \
                     OR  (geometrytype(wkb_geometry) = 'MULTIPOLYGON'::text)), \
                 CONSTRAINT enforce_srid_wkb_geometry CHECK (srid(wkb_geometry) = 4326) \
                 )"
    ${PSQL} "INSERT INTO geometry_columns (f_table_catalog, f_table_schema, f_table_name, f_geometry_column, coord_dimension, srid, type) \
                 VALUES ('', 'public', '${CUTLAYER}', 'wkb_geometry', 2, 4326, 'POLYGON')"
    
    ${PSQL} "ALTER TABLE ${CUTLAYER} ADD PRIMARY KEY (ogc_fid)"
    ${PSQL} "ALTER TABLE ${CUTLAYER} ALTER COLUMN wkb_geometry SET STORAGE MAIN"
}

InitDiffTable () {
    ${PSQL} "DROP TABLE ${DIFFLAYER}"
    ${PSQL} "DELETE FROM geometry_columns WHERE f_table_name LIKE '${DIFFLAYER}'"
    ${PSQL} "CREATE TABLE ${DIFFLAYER} (ogc_fid serial NOT NULL, wkb_geometry geometry NOT NULL, CONSTRAINT enforce_srid_wkb_geometry CHECK (srid(wkb_geometry) = 4326))"
    ${PSQL} "INSERT INTO geometry_columns (f_table_catalog, f_table_schema, f_table_name, f_geometry_column, coord_dimension, srid, type) \
                 VALUES ('', 'public', '${DIFFLAYER}', 'wkb_geometry', 2, 4326, 'POLYGON')"
}

ImportShape () {
    rm -f ${SHAPE}\.dbf
    ogr2ogr -f PostgreSQL PG:"host=${PGHOST} user=${PGUSER} dbname=${PGDATABASE}" -append -t_srs "EPSG:4326" ${SHAPE}\.shp -nln ${CUTLAYER} > /dev/null 2>&1
    RETURN=${?}
    if [ ${RETURN} -eq 0 ]; then
        echo "Layer ${SHAPE} erfolgreich nach ${CUTLAYER} importiert"
        ${PSQL} "CREATE INDEX ${CUTLAYER}_gindex ON ${CUTLAYER} USING GIST (wkb_geometry GIST_GEOMETRY_OPS)"
    else
        echo "Fehler beim Import von ${SHAPE} nach ${CUTLAYER} - exiting !!"
        exit 1
    fi
}

CheckWithin () {
    GEOM=${1}
    CSLAYER=`echo ${GEOM} | awk -F\# '{print $1}'`
    OGC_FID=`echo ${GEOM} | awk -F\# '{print $2}'`
    CHECKWITHINSTRING="SELECT ST_Within((SELECT wkb_geometry FROM ${CSLAYER} WHERE ogc_fid = ${OGC_FID}), (SELECT wkb_geometry FROM ${CUTLAYER}))"
    RETURN=`${PSQL} "${CHECKWITHINSTRING}"`
    if [ ${RETURN} == "t" ]; then
        DELSTRING="DELETE FROM ${CSLAYER} WHERE ogc_fid = ${OGC_FID}"
        if [ ${MODE} == "testing" ]; then
            echo ${DELSTRING} >> ${WITHINSCRIPT}
        fi
    else
        DIFFSTRING="SELECT ST_Difference((SELECT wkb_geometry FROM ${CSLAYER} WHERE ogc_fid = ${OGC_FID}), (SELECT wkb_geometry FROM ${CUTLAYER}))"
        ${PSQL} "INSERT INTO ${DIFFLAYER} (wkb_geometry) ${DIFFSTRING}"
        if [ ${MODE} == "testing" ]; then
            echo ${GEOM} >> ${WITHOUTSCRIPT}
        fi
    fi
}

SingularizeDump () {
    if [ `${PSQL} "SELECT COUNT(*) FROM ${DIFFLAYER}"` -gt 0 ]; then
        for OGC_FID in `${PSQL} "SELECT ogc_fid FROM ${DIFFLAYER} WHERE NumGeometries(wkb_geometry) IS NOT NULL"`; do
            ${PSQL} "INSERT INTO ${DIFFLAYER} (wkb_geometry) \
                (SELECT GeometryN(wkb_geometry, generate_series(1, NumGeometries(wkb_geometry))) \
                AS wkb_geometry \
                FROM ${DIFFLAYER} \
                WHERE ogc_fid = ${OGC_FID})"
            ${PSQL} "DELETE FROM ${DIFFLAYER} WHERE ogc_fid = ${OGC_FID}"
        done
        CPSTRING="INSERT INTO ${CSLAYER} (wkb_geometry) (SELECT wkb_geometry FROM ${DIFFLAYER})"
        if [ ${MODE} == "testing" ]; then
            pgsql2shp -f ${DUMPDIR}/${CSLAYER}.shp ${PGSQL2SHPCS} "SELECT * FROM ${DIFFLAYER}"
        elif [ ${MODE} == "production" ]; then
            ${PSQL} "${CPSTRING}"
        fi
    fi
}

CheckIntersects () {
    SINGULAR=`${PSQL} "SELECT COUNT(*) FROM ${CUTLAYER}"`
    if [ ${SINGULAR} -ne 1 ]; then
        echo "Selection requires to contain exactly one single feature - exiting !"
        exit 1
    else
        for CSLAYER in `${PSQL} "SELECT f_table_name FROM geometry_columns WHERE f_table_name LIKE '${LAYERPREFIX}%' \
            AND type LIKE 'POLYGON' ORDER BY f_table_name"`; do
            InitDiffTable
            for OGC_FID in `${PSQL} "SELECT ogc_fid FROM ${CSLAYER} WHERE wkb_geometry && (SELECT wkb_geometry FROM ${CUTLAYER}) ORDER BY ogc_fid"`; do
                RETURN=`${PSQL} "SELECT ST_Intersects((SELECT wkb_geometry FROM ${CUTLAYER}), (SELECT wkb_geometry FROM ${CSLAYER} WHERE ogc_fid = ${OGC_FID}))"`
                if [ ${RETURN} == "t" ]; then
                    DELSTRING="DELETE FROM ${CSLAYER} WHERE ogc_fid = ${OGC_FID}"
                    if [ ${MODE} == "testing" ]; then
                        echo ${DELSTRING} >> ${LOGSCRIPT}
                    elif [ ${MODE} == "production" ]; then
                        ${PSQL} "${DELSTRING}"
                    fi
                    CheckWithin "${CSLAYER}#${OGC_FID}"
                fi
            done
            SingularizeDump
        done
    fi
}


InitCutoutTable
ImportShape
CheckIntersects

# EOF

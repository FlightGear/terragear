#!/bin/sh
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

# Create reasonable symlinks pointing to this script to return proper
# $MODE-values for the "case" clause, like 'grass02reclass.sh_shp'
MODE=`basename ${0} | cut -f 2 -d \_`
RUNDIR=`pwd`
cd `dirname ${0}` && export BASEDIR=`pwd`
cd ${RUNDIR}
#
MAPPINGFILE=${BASEDIR}/CORINEtoCS.txt
#

case ${MODE} in
        shp)
            SELECTION=`g.mlist type=vect pattern="c[0-9][0-9][0-9]"`
            Code00map () {
                CODE00=`echo ${MAP} | tr -d c`
            }
        ;;
        ldb)
            SELECTION=`g.mlist type=vect pattern="cs_*"`
            Code00map () {
                CODE00=`grep "\ ${MAP}"\$ ${MAPPINGFILE} | awk '{print $1}'`
            }
        ;;
esac

#
# cat = code_00, integer
for MAP in ${SELECTION}; do
    Code00map
    INTMAP=${MAP}_int
    v.db.addcolumn map=${MAP} columns="newcode00 integer" --verbose
    v.db.update map=${MAP} column=newcode00 value=${CODE00} --verbose
    v.db.dropcolumn map=${MAP} column=code_00 --verbose
    v.db.renamecolumn map=${MAP} column=newcode00,code_00 --verbose
    v.reclass input=${MAP} output=${INTMAP} column=code_00 --verbose
done

# EOF

#! /bin/bash

for lat in s90 s80 s70 s60 s50 s40 s30 s20 s10 n00 n10 n20 n30 n40 n50 n60 \
           n70 n80; do
    for lon in w180 w170 w160 w150 w140 w130 w120 w110 w100 w090 w080 w070 \
               w060 w050 w040 w030 w020 w010 e000 e010 e020 e030 e040 e050 \
	       e060 e070 e080 e090 e100 e110 e120 e130 e140 e150 e160 e170; do
        echo -n "${lon}${lat} "
    done
    echo
done

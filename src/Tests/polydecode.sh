#!/bin/bash

# Put your datasource here
DATABASE=./testlandclass
WORKBASE=./Work

POLYDECODE=../../release/src/Prep/PolyDecode/poly-decode

rm -rf ./Paths/*
rm -rf ./Work/*

mkdir Paths
mkdir Work

echo "Decoding test2"
time ${POLYDECODE} --area-type test ${WORKBASE} ${DATABASE}/test2.shp

#!/bin/bash

home1="/home.dokt/mishkdmy/public/CVPR2014/binaries"
matcher="${home1}/./gdbicp"
dataset="/home.dokt/mishkdmy/public/CVPR2014/snavely"
logs="/home.dokt/mishkdmy/public/CVPR2014/logs-snavely/dual-bootstrap"

for pair in ${dataset}/*
do
if [ -d $pair ];
then
pairname=$(basename "$pair")

if [ ! -d ${logs} ];
then
mkdir ${logs}
mkdir ${logs}/img
mkdir ${logs}/m
mkdir ${logs}/logs
mkdir ${logs}/H
fi

files=(${pair}/*)
f1=$(basename "${files[0]}")
f2=$(basename "${files[1]}")
echo ${pairname}
cd $home1
T="$(date +%s%N)"
${matcher} ${files[0]} ${files[1]} 
# Time interval in nanoseconds
T="$(($(date +%s%N)-T))"
# Seconds
S="$((T/1000000000))"
# Milliseconds
M="$((T/1000000))"
SM="$((M-1000*S))"
mos="mosaic_${f1%.*}_to_${f2%.*}"
echo 
echo $mos
if [ -f $mos.xform ];
then
echo "$S.$SM 1 -1" > ${logs}/logs/${pairname}log.txt 
exec 5< $mos.xform
for (( i=1; i<31; i++ )); do
read line1 <&5
done
read H1 <&5
read H2 <&5
read H3 <&5
echo $H1 > ${logs}/H/${pairname}H.txt 
echo $H2 >> ${logs}/H/${pairname}H.txt 
echo $H3 >> ${logs}/H/${pairname}H.txt 
exec 5<&-
cp $mos.png ${logs}/img/${pairname}1.png
rm $mos*
rm xformed*
else
echo "$S.$SM 0 -1" > ${logs}/logs/${pairname}log.txt 
echo "0 0 0" > ${logs}/H/${pairname}H.txt 
echo "0 0 0" >> ${logs}/H/${pairname}H.txt 
echo "0 0 0" >> ${logs}/H/${pairname}H.txt 
fi

fi
done

fi
done

#!/bin/bash

home1="/home.dokt/mishkdmy/public/CVPR2014/binaries"
matcher="${home1}/./demo_ASIFT"
dataset="/home.dokt/mishkdmy/public/CVPR2014/EVD"
logs="/home.dokt/mishkdmy/public/CVPR2014/logs-EVD/asift"


if [ ! -d ${logs} ];
then
mkdir ${logs}
mkdir ${logs}/img
mkdir ${logs}/m
mkdir ${logs}/logs
mkdir ${logs}/F
fi

for pair in ${dataset}/1/*
do
if [ -f $pair ];
then
pairname=$(basename "$pair")

f1=${dataset}/1/$pairname
f2=${dataset}/2/$pairname
echo ${pairname}
cd $home1
${matcher} $f1 $f2 ${logs}/img/${pairname}1.png ${logs}/img/${pairname}2.png ${logs}/m/${pairname}.txt 1.txt 2.txt 0 log.txt
exec 5< log.txt
while read line1 <&5 ; do
a=( $line1 )
if (( ${a[1]} > 0 )); then
echo "${a[0]} 1 ${a[1]}" > ${logs}/logs/${pairname}log.txt 
else
echo "${a[0]} 0 ${a[1]}" > ${logs}/logs/${pairname}log.txt 
fi
done
exec 5<&-
rm log.txt
cp Fmatrix.txt ${logs}/F/${pairname}F.txt
rm Fmatrix.txt
fi
done

fi
done

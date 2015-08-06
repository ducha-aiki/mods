#!/bin/bash
home1="/home/old-ufo/mods/mods-new/build/configs"
matcher="${home1}/./demo_ASIFT"
dataset="/windows/datasets/EF"
logs="/windows/mods/BMVC-2015/det_test/EF"

#dets=("FOCI" "DoGAff" "MSER" "HessAff" "DoG" "FOCI" "AdaptHessAff" "AdaptDoG" "AdaptMSER" "iiDoG" "SURF" "SURFAff" "ORB" "KAZE"  )
#dets=("DoGAff" "MSER" "HessAff" "DoG" "FOCI" "AdaptHessAff" "AdaptDoG" "AdaptMSER" )
#dets=("FOCIAff" "iiDoG" "SURF" "SURFAff" "ORB" "KAZE" "KAZEAff")
#dets=("WXBS" "SFOP" "WADE" "WASH")
dets=("ASIFT")

for (( dd=0; dd<1; dd++)); do
det=${dets[${dd}]}
  logdir=${logs}/${det}
  echo $det

  
  if [ ! -d ${logdir} ];
  then
      mkdir ${logdir}
      mkdir ${logdir}/img
  fi
    
    
  for file1 in ${dataset}/*
  do
    if [ -d $file1 ];
    then
      filename=$(basename "$file1")
      d=${filename}
      files=(${dataset}/${d}/*)
      f1=img1.png
      if [ "$d" = "Obama" ]; then
      for num in {6..13}
      do
	nnn=$(( num-4 ))
	f2=$(basename "img${nnn}.png")
	echo ${d}
	${matcher} ${dataset}/${d}/${f1} ${dataset}/${d}/${f2} ${logdir}/img/${d}-1-${nnn%.*}1.png ${logdir}/img/${d}-1-${nnn%.*}2.png\
        1.txt 2.txt m.txt 0 ${logdir}/${d}-${nnn}.log
      done
      fi
      if [ "$d" = "NotreDame" ]; then
      for num in {6..10}
      do
	nnn=$(( num-4 ))
	f2=$(basename "img${nnn}.png")
	echo ${d}
	${matcher} ${dataset}/${d}/${f1} ${dataset}/${d}/${f2} ${logdir}/img/${d}-1-${nnn%.*}1.png ${logdir}/img/${d}-1-${nnn%.*}2.png\
        1.txt 2.txt m.txt 0 ${logdir}/${d}-${nnn}.log
      done
      fi
      
      if [ "$d" = "Rushmore" ]; then
      for num in {6..10}
      do
	nnn=$(( num-4 ))
	f2=$(basename "img${nnn}.png")
	echo ${d}
	${matcher} ${dataset}/${d}/${f1} ${dataset}/${d}/${f2} ${logdir}/img/${d}-1-${nnn%.*}1.png ${logdir}/img/${d}-1-${nnn%.*}2.png\
        1.txt 2.txt m.txt 0 ${logdir}/${d}-${nnn}.log
      done
      fi
      
      if [ "$d" = "Yosemite" ]; then
      for num in {6..13}
      do
	nnn=$(( num-4 ))
	f2=$(basename "img${nnn}.png")
	echo ${d}
	${matcher} ${dataset}/${d}/${f1} ${dataset}/${d}/${f2} ${logdir}/img/${d}-1-${nnn%.*}1.png ${logdir}/img/${d}-1-${nnn%.*}2.png\
        1.txt 2.txt m.txt 0 ${logdir}/${d}-${nnn}.log
      done
      fi        
      
      if [ "$d" = "PaintedLadies" ]; then
      for num in {6..12}
      do
	nnn=$(( num-4 ))
	f2=$(basename "img${nnn}.png")
	echo ${d}
	${matcher} ${dataset}/${d}/${f1} ${dataset}/${d}/${f2} ${logdir}/img/${d}-1-${nnn%.*}1.png ${logdir}/img/${d}-1-${nnn%.*}2.png\
        1.txt 2.txt m.txt 0 ${logdir}/${d}-${nnn}.log
      done
      fi      
    fi
  done
done

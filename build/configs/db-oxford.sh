#!/bin/bash
home1="/home/old-ufo/mods/mods-new/build/configs"
matcher="${home1}/./gdbicp"
dataset="/windows/datasets/Mik"
logs="/windows/mods/BMVC-2015/det_test/oxford"

dets=("DBstrap")

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
#      array=$(ls)
      files=(${dataset}/${d}/*)
      f1=$(basename "${files[5]}")
      for num in {6..10}
      do
	nnn=$(( num-4 ))
	f2=$(basename "${files[${num}]}")
	echo ${d}
	T="$(date +%s%N)"
	${matcher} ${dataset}/${d}/${f1} ${dataset}/${d}/${f2}
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
	echo "$S.$SM 1 -1" > ${logdir}/${d}-${nnn}.log
	exec 5< $mos.xform
	for (( i=1; i<31; i++ )); do
	read line1 <&5
	done
	cp $mos.png ${logdir}/img/${d}-1-${nnn%.*}1.png
	rm $mos*
	rm xformed*
	else
	echo "$S.$SM 0 -1" > ${logdir}/${d}-${nnn}.log
	fi
      done
    fi
  done
done

#!/bin/bash
home1="/home/old-ufo/mods/mods-new/build/configs"
matcher="${home1}/./gdbicp"
dataset="/windows/datasets/lost_in_past"
logs="/windows/mods/BMVC-2015/det_test/lostinpast"

#dets=("WXBS" "MSER" "HessAff" "DoG" "DoGAff" "AdaptHessAff" "AdaptDoG" "AdaptMSER"  "iiDoG" "SURF" "SURFAff" "ORB" "KAZE" "KAZEAff" "Hes" "MODS" "SFOP" "WADE" "WASH"  "FOCI" "FOCIAff")
dets=("DBstrap")

for (( dd=0; dd<1; dd++)); do
  det=${dets[${dd}]}
  logdir=${logs}/${det}
  echo $det
   
  if [ ! -d ${logdir} ];
  then
    mkdir ${logdir}
  fi
    
    
  for section in ${dataset}/new/*
  do
    if [ -d $section ];
    then
      sect_name=$(basename "$section")
      d=${sect_name}
      queries=${dataset}/new/${d}
      olds=${dataset}/old/${d}
      if [ ! -d ${logdir}/${d} ];
      then
	mkdir ${logdir}/${d}
	mkdir ${logdir}/${d}/img
      fi
      echo $d
      
      queries_array=($(ls -d ${queries}/*))
      len_q=${#queries_array[@]}
      olds_array=($(ls -d ${olds}/*))
      len_o=${#olds_array[@]}
   #   echo $len_q
   #   echo $len_o
      min_idx=$(($len_q<$len_o?$len_q:$len_o))
      for (( img_idx=0; img_idx<${min_idx}; img_idx++)); do
        query=${queries_array[${img_idx}]}
        old=${olds_array[${img_idx}]}       
	if [ -f $query ];
	then
	  f1=$(basename "${query}")
          if [ -f $old ];
	  then
	    f2=$(basename "${old}")
	    cd $home1   
	    echo $f1
	    echo $f2
	    
	    T="$(date +%s%N)"
	${matcher}  ${queries}/${f1} ${olds}/${f2}
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
	echo "$S.$SM 1 -1" > ${logdir}/${d}/${f1%.*}-${f2%.*}.log
	exec 5< $mos.xform
	for (( i=1; i<31; i++ )); do
	read line1 <&5
	done
	cp $mos.png ${logdir}/${d}/img/${f1%.*}-${f2%.*}-1.png
	rm $mos*
	rm xformed*
	else
	echo "$S.$SM 0 -1" > ${logdir}/${d}/${f1%.*}-${f2%.*}.log
	fi
	  fi
	fi
      done
    fi
  done
done

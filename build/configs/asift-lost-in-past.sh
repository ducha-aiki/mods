#!/bin/bash
home1="/home/old-ufo/mods/mods-new/build/configs"
matcher="${home1}/./demo_ASIFT"
dataset="/windows/datasets/lost_in_past"
logs="/windows/mods/BMVC-2015/det_test/lostinpast"

dets=("ASIFT")

for (( dd=0; dd<1; dd++)); do
  det=${dets[${dd}]}
  logdir=${logs}/${det}
 
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
	      ${matcher} ${queries}/${f1} ${olds}/${f2} ${logdir}/${d}/img/${f1%.*}-${f2%.*}-1.png ${logdir}/${d}/img/${f1%.*}-${f2%.*}-2.png\
	      1.txt 2.txt m.txt 0 ${logdir}/${d}/${f1%.*}-${f2%.*}.log
	  fi
	fi
      done
    fi
  done
done

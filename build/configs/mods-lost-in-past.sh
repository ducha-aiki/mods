#!/bin/bash
home1="/home/old-ufo/mods/wxbs-journal/build/configs"
matcher="${home1}/./mods"
dataset="/windows/datasets/lost_in_past"
logs="/windows/mods/BMVC-2015/det_test/lostinpast"

#dets=("WXBS" "MSER" "HessAff" "DoG" "DoGAff" "AdaptHessAff" "AdaptDoG" "AdaptMSER"  "iiDoG" "SURF" "SURFAff" "ORB" "KAZE" "KAZEAff" "Hes" "MODS" "SFOP" "WADE" "WASH"  "FOCI" "FOCIAff")
#dets=("AdHesAfR")
dets=("WXBS-plain")
dets=("TILDE-Cha" "TILDE-Cou" "TILDE-Fra" "TILDE-Mex" "TILDE-Pan" "TILDE-StL" )
dets=("saddle_nms_affine")
for (( dd=0; dd<1; dd++)); do
det=${dets[${dd}]}
  logdir=${logs}/${det}
  echo $det
  config_file=config_iter_${det}.ini
  
   
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
	    #cd $home1   
	    echo $f1
	    echo $f2
      ${matcher} ${queries}/${f1} ${olds}/${f2} ${logdir}/${d}/img/${f1%.*}-${f2%.*}-1.png ${logdir}/${d}/img/${f1%.*}-${f2%.*}-2.png\
      1.txt 2.txt m.txt ${logdir}/${d}/${f1%.*}-${f2%.*}.log 0 0 1.txt ${home1}/${config_file} $home1/iters_${det}.ini
	  fi
	fi
      done
    fi
  done
done



 
    if [ "$det" = "WXBS-plain" ]; then
    config_file=config_iter_match_AdHes.ini
  fi
  
  config_file=config_iter_match_fixed.ini
    if [ "$det" = "AdHesAfR" ]; then
    config_file=config_iter_match_adapt.ini
  fi
  if [ "$det" = "DoGAff" ]; then
    config_file=config_iter_match_fixed_DogAff.ini
  fi
  if [ "$det" = "AdaptHessAff" ]; then
    config_file=config_iter_match_adapt.ini
  fi
  if [ "$det" = "AdaptDoG" ]; then
    config_file=config_iter_match_adapt.ini
  fi
  if [ "$det" = "AdaptMSER" ]; then
    config_file=config_iter_match_adapt.ini
  fi
  if [ "$det" = "FOCIAff" ]; then
    config_file=config_iter_match_aff.ini
  fi
  if [ "$det" = "SURFAff" ]; then
    config_file=config_iter_match_aff.ini
  fi
  if [ "$det" = "KAZEAff" ]; then
    config_file=config_iter_match_aff.ini
  fi
  if [ "$det" = "iiDoG" ]; then
    config_file=config_iter_match_iiDoG.ini
  fi
  if [ "$det" = "Hes" ]; then
    config_file=config_iter_match_Hes.ini
  fi
  if [ "$det" = "AdHes" ]; then
    config_file=config_iter_match_AdHes.ini
  fi
  if [ "$det" = "WXBS" ]; then
    config_file=config_iter_match_adapt.ini
  fi
  if [ "$det" = "WXBS-plain" ]; then
    config_file=config_iter_match_AdHes.ini
  fi
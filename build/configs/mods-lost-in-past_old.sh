#!/bin/bash
home1="/home/old-ufo/mods/mods-new/build/configs"
matcher="${home1}/./mods"
dataset="/windows/datasets/lost_in_past"
logs="/windows/mods/BMVC-2015/det_test/lostinpast"

dets=("MSER" "HessAff" "DoG" "DoGAff" "AdaptHessAff" "AdaptDoG" "AdaptMSER"  "iiDoG" "SURF" "SURFAff" "ORB" "KAZE" "KAZEAff" "Hess" "MODS" "FOCI" "FOCIAff")
for (( dd=0; dd<15; dd++)); do
  det=${dets[${dd}]}
  logdir=${logs}/${det}
  echo $det
  config_file=config_iter_match_fixed.ini
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
      for query in ${queries}/*
      do
	if [ -f $query ];
	then
	  f1=$(basename "${query}")
	  for old in ${olds}/*
	  do
	    if [ -f $old ];
	    then
	      f2=$(basename "${old}")
	      cd $home1
	      
	      ${matcher} ${queries}/${f1} ${olds}/${f2} ${logdir}/${d}/img/${f1%.*}-${f2%.*}-1.png ${logdir}/${d}/img/${f1%.*}-${f2%.*}-2.png\
	      1.txt 2.txt m.txt ${logdir}/${d}/${f1%.*}-${f2%.*}.log 0 0 1.txt ${home1}/${config_file} $home1/iters_${det}.ini
	    fi
	  done
	fi
      done
    fi
  done
done

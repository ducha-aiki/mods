#!/bin/bash

home1="/home/old-ufo/mods/mods-new/build/configs"
matcher="${home1}/./mods"
dataset="/windows/datasets/WxBS"
logs="/windows/mods/BMVC-2015/det_test/wxbs"

#dets=("FOCI" "DoGAff" "MSER" "HessAff" "DoG" "FOCI" "AdaptHessAff" "AdaptDoG" "AdaptMSER" "iiDoG" "SURF" "SURFAff" "ORB" "KAZE"  )
#dets=("DoGAff" "MSER" "HessAff" "DoG" "FOCI" "AdaptHessAff" "AdaptDoG" "AdaptMSER" )
#dets=("FOCIAff" "iiDoG" "SURF" "SURFAff" "ORB" "KAZE" "KAZEAff")
#dets=("WXBS" "SFOP" "WADE" "WASH")
dets=("WXBS-plain")

for (( dd=0; dd<1; dd++)); do
det=${dets[${dd}]}
  logdir=${logs}/${det}
  echo $det
  
    if [ "$det" = "WXBS-plain" ]; then
    config_file=config_iter_match_AdHes.ini
  fi
      if [ "$det" = "AdHesAfR" ]; then
    config_file=config_iter_match_adapt.ini
  fi
  if [ "$det" = "AdHes" ]; then
    config_file=config_iter_match_AdHes.ini
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
  if [ "$det" = "WXBS" ]; then
    config_file=config_iter_match_adapt.ini
  fi  
    
  
  if [ ! -d ${logdir} ];
    then
      mkdir ${logdir}
    fi
    
    for section in ${dataset}/*
      do
      if [ -d $section ];
	then
	sectionname=$(basename "$section")

	if [ ! -d ${logdir}/${sectionname} ];
	  then
	  mkdir ${logdir}/${sectionname}
	  mkdir ${logdir}/${sectionname}/img

	  fi

	  for pair in ${section}/*
	  do
	  if [ -d $pair ];
	  then
	  files=(${pair}/*)
	  f1=$(basename "${files[0]}")
	  f2=$(basename "${files[1]}")
	  pairname=$(basename "$pair")
	  echo ${sectionname}-${pairname}
	  cd $home1
	  ${matcher} ${files[0]} ${files[1]} ${logdir}/${sectionname}/img/${pairname}1.png ${logdir}/${sectionname}/img/${pairname}2.png\
	  1.txt 2.txt m.txt ${logdir}/${sectionname}/${pairname}.log 0 0 1.txt ${home1}/${config_file} $home1/iters_${det}.ini
	  fi
	  done
	fi
done
done


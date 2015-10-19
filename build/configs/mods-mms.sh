#!/bin/bash

home1="/home/old-ufo/mods/wxbs-journal/build/configs"
matcher="${home1}/./mods"
dataset="/windows/datasets/MMS"
logs="/windows/mods/BMVC-2015/det_test/MMS"

#dets=("FOCI" "DoGAff" "MSER" "HessAff" "DoG" "FOCI" "AdaptHessAff" "AdaptDoG" "AdaptMSER" "iiDoG" "SURF" "SURFAff" "ORB" "KAZE"  )
#dets=("DoGAff" "MSER" "HessAff" "DoG" "FOCI" "AdaptHessAff" "AdaptDoG" "AdaptMSER" )
#dets=("FOCIAff" "iiDoG" "SURF" "SURFAff" "ORB" "KAZE" "KAZEAff")
#dets=("WXBS" "SFOP" "WADE" "WASH")
dets=("WXBS-plain")

dets=("TILDE" "TILDE-scalespace")
dets=("TILDE-Cha" "TILDE-Cou" "TILDE-Fra" "TILDE-Mex" "TILDE-Pan")


dets=("saddle_nms_affine")
for (( dd=0; dd<1; dd++)); do
det=${dets[${dd}]}
  logdir=${logs}/${det}
  echo $det
  config_file=config_iter_${det}.ini
  
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
      mkdir ${logdir}/img
    fi
  for i in {1..100}
  do
    echo ${i}
  
      ${matcher} ${dataset}/00000${i}_VSRRTC.bmp ${dataset}/00000${i}_IR1CTC.bmp ${logdir}/img/${i}-1.png ${logdir}/img/${i}-2.png\
      1.txt 2.txt m.txt ${logdir}/${i}.log 0 0 1.txt ${home1}/${config_file} $home1/iters_${det}.ini
  done
done


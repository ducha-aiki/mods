## MODS: Image Matching with On-Demand Synthesis.

Compilation. 
MODS depends on OpenCV version 2.4.9 and LAPACK

## How to compile MODS on clean ubuntu 14.04 (tested on amazon AWS instance)

sudo apt-get install git cmake gfortran libblas-dev liblapack-dev build-essential gcc-multilib libopencv-dev python-opencv

sudo add-apt-repository --yes ppa:xqms/opencv-nonfree
sudo apt-get update
sudo apt-get install libopencv-nonfree-dev

if you want to use edge foci detector and bice descriptor from Microsoft, you will need to install wine as well:
sudo apt-get install wine
git clone
cd mods

cd vlfeat
make

cd ../build
cmake ..
make

## Example of use:
Linux:
./mods examples/cat.png examples/cat2.png out1.png out2.png k1.txt k2.txt m.txt l.txt 0 1 examples/cat.txt config_iter_mods_cviu.ini iters_mods_cviu.ini


## Configurations:

config_iter_cviu.ini, iters_cviu.ini - version, created to hangle extreme view changes. 

Described in   
"MODS: Fast and Robust Method for Two-View Matching" by Dmytro Mishkin, Jiri Matas, Michal Perdoch.
http://arxiv.org/abs/1503.02619.

config_iter_wxbs.ini, iters_wxbs.ini - version, described in . 

"WxBS: Wide Baseline Stereo Generalizations" by Dmytro Mishkin, Jiri Matas, Michal Perdoch, Karel Lenc.
http://arxiv.org/abs/1504.06603
It handles extreme appearance and geometrical changes. A bit slower than previous, but much more powerful.
If use, please cite corresponding papers.

## How to save detectors\descriptors and use them for matching
Note that exctract features takes only one step, so you may need to edit iters*.ini file to be able to extract features from next steps. See an example in iters_mods_cviu_onestep.ini

./extract_features examples/cat.png  cat1.txt config_iter_cviu.ini iters_mods_cviu_onestep.ini
./extract_features examples/cat2.png  cat2.txt config_iter_cviu.ini iters_mods_cviu_onestep.ini

Now loading and matching
./mods examples/cat.png examples/cat2.png out1.png out2.png cat1.txt cat2.txt m.txt l.txt 0 1 examples/cat.txt config_iter_mods_cviu.ini iters_mods_cviu_onestep.ini 1


    
    
    
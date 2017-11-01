## MODS: Image Matching with On-Demand Synthesis.

## Binaries can be downloaded here

https://github.com/ducha-aiki/mods/releases/


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

## how to compile MODS on clean Windows 10
install cmake 
https://cmake.org/download/

install mingw 
http://www.mingw.org/

get lapack
https://icl.cs.utk.edu/lapack-for-windows/lapack/#libraries_mingw

add $mods_source_dir/lapack_for_windows/lib to your Path environment variable   

install OpenCV 2.4.8
If you have trouble compiling it, use this solution http://stackoverflow.com/a/21214333

Add opencv install root/bin to your path environmental variable

Put opencv install root CMakeLists.txt to 
SET (OpenCV_DIR "c:/opencv-2.4.8/opencv/sources/build/install")

    cd build
    cmake ..

Make sure, that CMake generates mingw32 make files, not Visual Studio.

    mingw32-make

## Example of use:
Linux:
    ./mods examples/cat.png examples/cat2.png out1.png out2.png k1.txt k2.txt m.txt l.txt 0 0 examples/cat.txt config_iter_mods_cviu.ini iters_mods_cviu.ini

Windows:
    ./mods.exe examples/cat.png examples/cat2.png out1.png out2.png k1.txt k2.txt m.txt l.txt 0 0 examples/cat.txt config_iter_mods_cviu.ini iters_mods_cviu.ini


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
    ./mods examples/cat.png examples/cat2.png out1.png out2.png cat1.txt cat2.txt m.txt l.txt 0 0 examples/cat.txt config_iter_mods_cviu.ini iters_mods_cviu_onestep.ini 1


 ## Citation

Please cite us if you use this code:

    @article{Mishkin2015MODS,
          title = "MODS: Fast and robust method for two-view matching ",
          journal = "Computer Vision and Image Understanding ",
          year = "2015",
          issn = "1077-3142",
          doi = "http://dx.doi.org/10.1016/j.cviu.2015.08.005",
          url = "http://www.sciencedirect.com/science/article/pii/S1077314215001800",
          author = "Dmytro Mishkin and Jiri Matas and Michal Perdoch"
          }
    
    

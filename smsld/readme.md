===== SOFTWARE =====

This software contains the SMSLD implementation as proposed by Bart Verhagen (barrie.verhagen@gmail.com), Radu Timofte (radu.timofte@esat.kuleuven.be) and Luc Van Gool (luc.vangool@esat.kuleuven.be). The current implementation is based on the MSLD implementation of Zhiheng Wang (zhwang@nlpr.ia.ac.cn), Fuchao Wu and Zhanyi Hu.

This software contains the SMSLD descriptor. The SMSLD descriptor is a scale-invariant line segment descriptor for wide baseline matching. More info can be found in the paper about SMSLD.

All comments can be addressed to bart.verhagen@tass.be or barrie.verhagen@gmail.com

===== Requirements =====

MATLAB
OpenCV
Preferably Microsoft Visual Studio
===== How to install =====

Change the *.props files in SMSLD/opencv to point to your openCV implementation.
Compile the SMSLD/SMSLD/SMSLD_Step2_Match/TianMatch.sln project using Visual Studio.
Change in matlab/workspace/testbench/SMSLD/testSMSLD.m and matlab/workspace/testbench/SMSLD/benchmark.m the 'path_to_exe' variable to point to the binary created during the compilation of the TianMatch.sln project.
Run testSMSLD using matlab. Note: default the paths are set to run the test algorithms in the directory they exist.
===== Additional notes =====

The MSLD implementation on which the SMSLD implementation is based is unstable. Rerunning the program a few times might give you the result in the end. Slightly changing a parameter may help to stabilize.

If you are using this software, please quote our associated work:

Scale-invariant Line Descriptors for Wide Baseline Matching by B. Verhagen, R. Timofte and L Van Gool. published at 2014 IEEE Winter Conference on Applications of Computer Vision (WACV) Copyright by IEEE
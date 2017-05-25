//
// Created by old-ufo on 3/30/15.
//
#ifndef MODS_NEW_IO_MODS_H
#define MODS_NEW_IO_MODS_H

#include "configuration.hpp"
#include "detectors/detectors_parameters.hpp"
#include "descriptors_parameters.hpp"
#include "matching.hpp"
#include "inih/cpp/INIReader.h"

const int Tmin = 9;//minimum number of command-line parameters

struct configs
{
    int n_threads;
    DescriptorsParameters DescriptorPars;
    DetectorsParameters DetectorsPars;
    DominantOrientationParams DomOriPars;
    int LoadColor;
    MatchPars Matchparam;
    RANSACPars RANSACParam;
    std::vector<IterationViewsynthesisParam> ItersParam;
    parameters CLIparams;
    filteringParams FilterParam;
    drawingParams DrawParam;
    outputParams OutputParam;
    bool read_pre_extracted;
    bool match_one_to_many;
    string descriptor, matching_lib, verification_type;
    configs()
    {
        n_threads = 1;
        LoadColor = 0;
        read_pre_extracted = false;
        match_one_to_many = false;
    }
};

void WriteLog(logs log, ostream& out);
void WriteTimeLog(TimeLog log, ostream &out,
                  const int writeRelValues = 1,
                  const int writeAbsValues = 0,
                  const int writeDescription = 0);
void GetMSERPars(extrema::ExtremaParams &MSERPars, INIReader &reader,const char* section="MSER");
void GetFASTPars(FASTParams &pars, INIReader &reader,const char* section="FAST");
void GetSTARPars(STARParams &pars, INIReader &reader,const char* section="STAR");

void GetTILDEPars(TILDEParams &pars, INIReader &reader,const char* section="TILDE");
void GetSFOPPars(SFOPParams &pars, INIReader &reader,const char* section="SFOP");
void GetSaddlePars(SaddleParams &pars, INIReader &reader,const char* section="Saddle");
void GetToSMSERPars(ToSMSERParams &pars, INIReader &reader,const char* section="TOS-MSER");

void GetWAVEPars(WAVEParams &pars, INIReader &reader,const char* section="WAVE");
void GetWASHPars(WASHParams &pars, INIReader &reader,const char* section="WASH");
void GetCLIDescPars(CLIDescriptorParams &pars, INIReader &reader,const char* section = "CLIDescriptor");
void GetSURFPars(SURFParams &pars, INIReader &reader,const char* section="SURF");
void GetBRISKPars(BRISKParams &pars, INIReader &reader,const char* section="BRISK");
void GetFREAKPars(FREAKParams &pars, INIReader &reader,const char* section="FREAK");
void GetMROGHPars(MROGHParams &pars, INIReader &reader,const char* section="MROGHDescriptor");
void GetLIOPPars(LIOPDescriptorParams &pars, INIReader &reader,const char* section="LIOP");
void GetKAZEPars(KAZEParams &pars, INIReader &reader,const char* section="AKAZE");
//void GetDALIPars(DALIParams &pars, INIReader &reader,const char* section="DALI");
void GetSMSLDPars(SMSLDParams &pars, INIReader &reader,const char* section="SMSLD");
void GetORBPars(ORBParams &pars, INIReader &reader,const char* section="ORB");
void GetDAISYPars(DAISYParams &pars, INIReader &reader,const char* section="DAISY");
void GetSSIMPars(SSIMParams &pars, INIReader &reader,const char* section="SSIM");
void GetReadPars(ReadAffsFromFileParams &pars, INIReader &reader,const char* section="ReadAffs");
void GetBICEPars(BICEParams &pars, INIReader &reader,const char* section="BICE");
void GetPixelPars(PIXELSDescriptorParams &pars, INIReader &reader,const char* section="PixelDescriptor");
void GetFOCIPars(FOCIParams &pars, INIReader &reader,const char* section="FOCI");
void GetHessPars(ScaleSpaceDetectorParams &HessPars, INIReader &reader,const char* section="HessianAffine");
void GetPatchExtractionPars(PatchExtractionParams &PEPars, INIReader &reader,const char* section);
void GetHarrPars(ScaleSpaceDetectorParams &HarrPars, INIReader &reader,const char* section="HarrisAffine");
void GetDoGPars(ScaleSpaceDetectorParams &DoGPars, INIReader &reader,const char* section="DoG");
void GetDomOriPars(DominantOrientationParams &DomOriPars, INIReader &reader,const char* section="DominantOrientation");
void GetBaumbergPars(AffineShapeParams &pars, INIReader &reader,const char* section="AffineAdaptation");

#ifdef WITH_CAFFE
void GetCaffePars(CaffeDescriptorParams &pars, INIReader &reader,const char* section="CaffeDescriptor");
#endif
void GetMatchPars(MatchPars &pars, INIReader &reader, INIReader &iter_reader, const char* section="Matching");
void GetSIFTDescPars(SIFTDescriptorParams &pars, INIReader &reader,const char* section="SIFTDescriptor");
void GetRANSACPars(RANSACPars &pars, INIReader &reader,const char* section="RANSAC");
void GetIterPars(std::vector<IterationViewsynthesisParam> &pars, INIReader &reader);
int getCLIparam(configs &conf1,int argc, char **argv);
int getCLIparamExtractFeatures(configs &conf1,int argc, char **argv);
int getCLIparamExtractFeaturesBenchmark(configs &conf1,int argc, char **argv);
int getCLIparamExportDescriptorsBenchmark(configs &conf1,int argc, char **argv);


#endif //MODS_NEW_IO_MODS_H

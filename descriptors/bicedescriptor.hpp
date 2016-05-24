#ifndef BICEDESCRIPTOR_HPP
#define BICEDESCRIPTOR_HPP
#include <iostream>
#include <fstream>
#include "detectors/structures.hpp"
#include <sys/time.h>

struct BICEParams {
  int dh;
  int dv;
  int dori;
  int dl;
    PatchExtractionParams PEParam;
//  int patchSize;
//  float mrScale;
  BICEParams() {
    dh = 24;
    dv = 8;
    dori = 12;
    dl = 1; //must be 1 or 2
//    patchSize = 32;
  //  mrScale = 8.0;
  }
};

inline long getMilliSecs2()
{
  timeval t;
  gettimeofday(&t, NULL);
  return t.tv_sec*1000 + t.tv_usec/1000;
}
inline std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
      elems.push_back(item);
    }
  return elems;
}
inline std::vector<std::string> split(const std::string &s, char delim) {
  std::vector<std::string> elems;
  split(s, delim, elems);
  return elems;
}

struct BICEDescriptor
{
public:
  BICEDescriptor(const BICEParams &par)
  {
    this->par = par;
    type = DESC_BICE;
  }
  void operator()(const cv::Mat& img, AffineRegionVector& temp_kp1_desc)
  {

    int rnd1 = (int) getMilliSecs2();
    std::string img_fname = "BICE"+std::to_string(rnd1)+".png";
    cv::imwrite(img_fname,img);
    std::string command = "wine EdgeFociAndBice.exe -md -i " + img_fname;
    std::string pts_fname = "BICE_IN" + std::to_string(rnd1) + ".txt";
    std::ofstream bice_kp(pts_fname);
    int kp_size = temp_kp1_desc.size();
    if (bice_kp.is_open()) {
        bice_kp << kp_size << std::endl;
        for (int kp_num=0; kp_num < kp_size; kp_num++)
          {
            AffineRegion temp_region = temp_kp1_desc[kp_num];
            double orient = atan2(temp_region.det_kp.a12 , temp_region.det_kp.a11);
            double ci = cos(-orient);
            double si = sin(-orient);
            double scale_sq = temp_region.det_kp.s * temp_region.det_kp.s ;
            bice_kp << temp_region.det_kp.x << " ";
            bice_kp << temp_region.det_kp.y << " ";
            bice_kp << scale_sq* (temp_region.det_kp.a11 * ci - temp_region.det_kp.a12 * si) * (temp_region.det_kp.a11 * ci - temp_region.det_kp.a12 * si)<< " ";
            bice_kp << scale_sq* (temp_region.det_kp.a21 * ci - temp_region.det_kp.a22 * si) * (temp_region.det_kp.a21 * ci - temp_region.det_kp.a22 * si)<< " ";
            bice_kp << scale_sq* (temp_region.det_kp.a21 * si + temp_region.det_kp.a22 * ci) * (temp_region.det_kp.a21 * si + temp_region.det_kp.a22 * ci)<< " ";
            bice_kp << 0.5 << " ";
            bice_kp << orient << std::endl;
          }
      }
    bice_kp.close();
    command += " -ip " + pts_fname;
    std::string fname1 = "BICE_OUT_" + std::to_string(rnd1) + ".txt";
    command += " -o " + fname1;
    command += " -dh " + std::to_string(par.dh);
    command += " -dl " + std::to_string(par.dl);
    command += " -do " + std::to_string(par.dori);
    command += " -dv " + std::to_string(par.dv);
    command += " -pd " + std::to_string(par.PEParam.patchSize);
    command += " -ps " + std::to_string(par.PEParam.mrSize);
    std::cerr << command <<std::endl;
    system(command.c_str());
    std::ifstream bice_out_kp(fname1);
    int thrown_pts = 0;
    if (bice_out_kp.is_open()) {
        std::string line12;
        getline(bice_out_kp,line12);
        std::vector<std::string> x2 = split(line12, ' ');
        int kp_size2;
        kp_size2 = std::stoi(x2[0]);
        int desc_size;
        desc_size = std::stoi(x2[1]);
        //    temp_kp1_desc.resize(kp_size2);
        std::string line1;
        int kp_num_db=0;
        int kp_num_bice=0;
        bool need_continue = true;
        bool need_read=true;
        bool need_next_pt=false;
        while (need_continue)
          {
            if (kp_num_db >= kp_size)
              break;
            if (kp_num_bice >= kp_size2)
              break;

            if (need_read){
                getline(bice_out_kp,line1);
                kp_num_bice++;
              }
            if (need_next_pt){
                kp_num_db++;
              }

            AffineRegion temp_region = temp_kp1_desc[kp_num_db];
            temp_region.desc.type=DESC_BICE;
            temp_region.desc.vec.resize(desc_size);
  //          std::cerr << line1 << std::endl;
            std::vector<std::string> x = split(line1, ' ');
            const float x_read = std::stof(x[0]);
            const float y_read = std::stof(x[1]);
            if ((fabs(temp_region.det_kp.y - y_read) > 0.5) && (fabs(temp_region.det_kp.x - x_read) > 0.5)) {
                std::cerr << " point mismatch";
                std::cerr << " " << x_read << " " << y_read << " "
                          << temp_region.det_kp.x << " " << temp_region.det_kp.y;
                temp_region.desc.vec = std::vector<float>(desc_size,0);
                thrown_pts++;
                need_next_pt = true;
                need_read = false;
              }
            else {
                float *vec1 = &(temp_region.desc.vec[0]);
                int desc_count = 0;
                for (int desc_part = 0; desc_part < x.size()-8; desc_part++)
                  {
                    std::string str_val = x[desc_part+7];
                    int val = std::stoi(str_val.substr(str_val.size()-1,1));
                    int len = std::stoi(str_val.substr(0,str_val.size()-1));
                    for (int bin_val=0; bin_val < len; bin_val++,vec1++){
                        desc_count++;
                        *vec1 = val;
                      }
                  }
                need_read = true;
              }
            temp_kp1_desc[kp_num_db]= temp_region;
          }
      }

    std::cerr << thrown_pts << " pts thrown away by mismatch" << std::endl;
    bice_out_kp.close();
    std::string rm_command = "rm " + fname1;
    system(rm_command.c_str());
    rm_command = "rm " + img_fname;
    system(rm_command.c_str());
    rm_command = "rm " + pts_fname;
    system(rm_command.c_str());

  }
public:
  descriptor_type type;

private:
  BICEParams par;
};


#endif // BICEDESCRIPTOR_HPP

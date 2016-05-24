int singleHDs2 (const double *u, const double *H, double *p, int * pool,
                int *pool2, int len, int* pos, const double * Z);

int waldH(double *u, int len, double th, double conf,
          int verif_type, double ep, double del,
          double *H, unsigned char * inl,
          int* data_out, int seed, double* hist,
          struct samhist **shout);

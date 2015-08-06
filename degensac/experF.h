/*struct samhist{
  double ep, del, A;
  int k;
  struct samhist * next;
};*/

int waldF(double *u, int len, double th, double conf,
          int verif_type, double ep, double del,
          double *F, unsigned char * inl,
          int* data_out, int seed, double* hist,
          struct samhist **shout);

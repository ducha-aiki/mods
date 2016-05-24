struct samhist
{
    double ep, del, A;
    int k;
    struct samhist * next;
};

double wlad_getA (double ep, double del, double *la_in, double *la_out,
                  double C_tM, double C_ms);

double wald_geth(double nep, double ep, double del);

int wald_nsamples(int ninl, int ptNum, int samsiz, double conf,
                  double A, struct samhist * sh);

struct samhist * wald_addsamhist(double ep, double del, double A,
                                 struct samhist * sh, int no_sam,
                                 int * lastAchg);



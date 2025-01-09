#ifndef __fft_h__
#define __fft_h__
#include "Img.h"
#include <complex>
#include <vector>
using namespace std;

vector<complex<double> > &FFT(vector<complex<double> > &original_vector);
vector<complex<double> > &iFFT(vector<complex<double> > &fft_vector);

Img<complex<double> > &FFT_2D(Img<complex<double> > &);
Img<complex<double> > &iFFT_2D(Img<complex<double> > &);

#endif

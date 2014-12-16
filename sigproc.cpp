/**
 * sigproc.cpp
 * This file is part of the Yate-BTS Project http://www.yatebts.com
 *
 * Signal processing library implementation
 *
 * Yet Another Telephony Engine - Base Transceiver Station
 * Copyright (C) 2014 Null Team Impex SRL
 *
 * This software is distributed under multiple licenses;
 * see the COPYING file in the main directory for licensing
 * information for this specific distribution.
 *
 * This use of this software may be subject to additional restrictions.
 * See the LEGAL file in the main directory for details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "sigproc.h"
#include <stdio.h>
#include <string.h>

//#define COMPLEX_DUMP_G

using namespace TelEngine;

static inline unsigned int sigProcMin(unsigned int v1, unsigned int v2)
{
    return (v1 <= v2) ? v1 : v2;
}


//
// Complex
//
void Complex::dump(String& dest) const
{
    char a[100];
#ifdef COMPLEX_DUMP_G
    int ret = sprintf(a,"(%g,%g)",real(),imag());
#else
    int ret = sprintf(a,"%+.5f%+.5fi, ",real(),imag());
#endif
    dest.append(a,ret);
}

// Multiply two complex arrays
void Complex::multiply(Complex* dest, unsigned int len,
    const Complex* c1, unsigned int len1, const Complex* c2, unsigned int len2)
{
    for (unsigned int n = sigProcMin(len,sigProcMin(len1,len2)); n; n--)
	multiply(*dest++,*c1++,*c2++);
}

// Compute the sum of two complex arrays
void Complex::sum(Complex* dest, unsigned int len,
    const Complex* c1, unsigned int len1, const Complex* c2, unsigned int len2)
{
    for (unsigned int n = sigProcMin(len,sigProcMin(len1,len2)); n; n--)
	sum(*dest++,*c1++,*c2++);
}

// Compute the difference of two complex arrays
void Complex::diff(Complex* dest, unsigned int len,
    const Complex* c1, unsigned int len1, const Complex* c2, unsigned int len2)
{
    for (unsigned int n = sigProcMin(len,sigProcMin(len1,len2)); n; n--)
	diff(*dest++,*c1++,*c2++);
}

// Compute the sum of product between complex number and its conjugate
//  of all numbers in an array
float Complex::sumMulConj(const Complex* data, unsigned int len)
{
    if (!data)
	return 0;
    float val = 0;
    for (; len; len--, data++)
	val += data->mulConj();
    return val;
}

// Compute the sum of product between 2 complex arrays
// E.g. dest = SUM(c1[i] * c2[i])
void Complex::sumMul(Complex& dest, const Complex* c1, unsigned int len1,
    const Complex* c2, unsigned int len2)
{
    if (!(c1 && c2))
	return;
    for (unsigned int n = sigProcMin(len1,len2); n; n--)
	sumMul(dest,*c1++,*c2++);
}

// Replace each element in an array with its conjugate
void Complex::conj(Complex* c, unsigned int len)
{
    if (!c)
	return;
    for (; len; len--)
	c->imag(-c->imag());
}

// Set complex elements from 16 bit integers array (pairs of real/imaginary parts)
void Complex::setInt16(Complex* dest, unsigned int len, const int16_t* src, unsigned int n,
    unsigned int offs)
{
    if (!(dest && len && src && n) || offs >= len)
	return;
    unsigned int room = len - offs;
    unsigned int full = sigProcMin(room,n / 2);
    bool rest = (full < room && (n % 2) != 0);
    for (dest += offs; full; full--, src += 2, dest++)
	dest->set((float)*src,(float)src[1]);
    if (rest)
	dest->set((float)*src);
}


// Dump a Complex vector
void Complex::dump(String& dest, const Complex* c, unsigned int len, const char* sep)
{
    if (!(c && len))
	return;
    for (; len; len--, c++) {
	if (dest && sep)
	    dest << sep;
	c->dump(dest);
    }
}


//
// SignalProcessing
//
SignalProcessing::~SignalProcessing()
{
    delete[] m_sinusoids;
}

void SignalProcessing::initialize(unsigned int oversample, unsigned int arfcns)
{
    m_oversample = oversample;
    m_arfcns = arfcns;
    fillFrequencyShift(&m_freqencyShift,m_oversample);
    fillLaurentPulseAproximation(&m_laurentPulseAproximation,m_oversample);
    buildSinusoids();
}

void SignalProcessing::fillFrequencyShift(ComplexArray* out, unsigned int oversample)
{
    if (!out)
	return;
    out->assign((int)BITS_PER_TIMESLOT);
    for (unsigned int i = 0; i < out->length(); i++) {
	// s[i] = e^(-jnPI/2S)
	// Set n*PI/2* oversample as the imaginary part of a complex number
	// Calculate the exponetial of that number
	//Complex::exp((*out)[i],0,-(float)i * PI / (2 * oversample));
	switch (i % 4) {
	    case 0:
		(*out)[i].set(1,0);
		break;
	    case 1:
		(*out)[i].set(0,1);
		break;
	    case 2:
		(*out)[i].set(-1,0);
		break;
	    case 3:
		(*out)[i].set(0,-1);
		break;
	}
    }
#ifdef DUMP_COMPLEX
    String outDump;
    out->dump(outDump);
    Debug(DebugAll,"SignalProcessing::buildFrequencyShift(%u) generated: %s",
	  oversample,outDump.c_str());
#endif
}

void SignalProcessing::fillLaurentPulseAproximation(ComplexArray* out, unsigned int oversample)
{
    if (!out)
	return;
    out->assign(3 * oversample);
    float halfLength = out->length() / 2;
    float sampled = 0.96 / oversample;
    for (int i = 0; i < (int)out->length(); i++) {
	float f = (i - halfLength) / oversample;
	f *= f;
	float g = -1.138 * f - 0.527 * f * f;
	(*out)[i] = sampled * ::expf(g);
    }
#ifdef DUMP_COMPLEX
    String outDump;
    out->dump(outDump);
    Debug(DebugAll,"SignalProcessing::buildLaurentPulseAproximation(%u) generated: %s",
	oversample,outDump.c_str());
#endif
}

ComplexArray* SignalProcessing::getConvolution(const ComplexArray& x,
    const ComplexArray& h)
{
#ifdef DUMP_COMPLEX
    String xDump,hDump;
    x.dump(xDump);
    Debug(DebugAll,"SignalProcessing::getConvolution x: %s, ",xDump.c_str());
    h.dump(hDump);
    Debug(DebugAll,"SignalProcessing::getConvolution h: %s",hDump.c_str());
#endif

    if (x.length() <= h.length()) {
	Debug(DebugWarn,"Invallid getConvolution input arguments! x size: %d, h size %d",
	    x.length(),h.length());
	return 0;
    }
    ComplexArray* ret = new ComplexArray(x.length());
    /**
     * Math formula: y[k] = SUM (i = 0 to n) from x[k+i]*h[n-i]
     * Where: n = h.length() - 1
     * The x index will go out of bounds at point x.length() - n
     * So I had splitted the math in two: untill and after: x.length() - n
     */
    Complex tmp;
    unsigned int endRef = h.length();
    for (unsigned int i = 0;i < x.length() - endRef + 1;i++) {
	for (unsigned int j = 0;j < endRef - 1;j++) {
	    Complex::multiply(tmp,x[i + j],h[endRef - j - 1]);
	    (*ret)[i] += tmp;
	}
    }

    for (unsigned int i = x.length() - endRef + 1;i < x.length();i++) {
	for (unsigned int j = 0;j < x.length() - i;j++) {
	    Complex::multiply(tmp,x[i + j],h[endRef - j - 1]);
	    (*ret)[i] += tmp;
	}
    }
    
#ifdef DUMP_COMPLEX
    String outDump;
    ret->dump(outDump);
    Debug(DebugAll,"SignalProcessing::getConvolution returning: %s",outDump.c_str());
#endif
    return ret;
}

ComplexArray* SignalProcessing::modulate(unsigned char* inData,
	unsigned int inDataLength, unsigned int oversample, const ComplexArray* frequencyShift,
	const ComplexArray* laurentPulseAproximation)
{
    if (!inData || inDataLength == 0 || !laurentPulseAproximation || !frequencyShift) {
	Debug(DebugStub,"SignalProcessing::modulate received invalid data! [%p] %u [%p] [%p]",
	      inData,inDataLength,laurentPulseAproximation,frequencyShift);
	return 0;
    }
	
#ifdef XDEBUG
    String inDump;
    inDump.hexify(inData,inDataLength,' ');
    Debug(DebugAll,"SignalProcessing::modulate( oversample %u, data: %s)",
	oversample,inDump.c_str());
#endif
    ComplexArray modulated(BITS_PER_TIMESLOT * oversample);
    unsigned int parseSize = inDataLength * oversample;
    if (parseSize > modulated.length())
	parseSize = modulated.length();

    for (unsigned int i = 0; i < parseSize; i++)
	Complex::multiplyF(modulated[i],(*frequencyShift)[i / oversample],inData[i / oversample] ? 1.0 : -1.0);

    // Fill the leftover data with 0.01
    // power ramp requirement
    for (unsigned int i = parseSize;i < modulated.length();i++)
	modulated[i].set(0.01,0);

    // Convolve the obtained array with Laurent pulse approximation array
    return getConvolution(modulated,*laurentPulseAproximation);
}

const ComplexArray* SignalProcessing::getSinusoid(unsigned int arfcn) const
{
    if (arfcn > m_arfcns)
	return 0;
    if (!m_sinusoids) {
	Debug(DebugStub,"SignalProcessing::initialize() not called!");
	return 0;
    }
    return &m_sinusoids[arfcn];
}

// Generate frequency shifting vector
// Each array element will be set with e ^ (-j * n * freqShiftParam)
//  where -j is the complex number (0,-1)
void SignalProcessing::setFreqShifting(ComplexVector* data, float val,
    unsigned int setLen)
{
    if (!data)
	return;
    if (setLen)
	data->resize(setLen);
    Complex* d = data->data();
    for (unsigned int i = 0; i < data->length(); i++)
	Complex::exp(*d++,0,-(float)i * val);
}

// Compute the power level from Complex array middle elements real part value
float SignalProcessing::computePower(const float* data, unsigned int len,
    unsigned int middle, float param)
{
    if (!(data && len && middle <= len))
	return 0;
    float val = 0;
    for (data += (len - middle) / 2; middle; middle--, data++)
	val += *data * *data;
    return param * val;
}

void SignalProcessing::buildSinusoids()
{
    if (m_sinusoids)
	delete[] m_sinusoids;
    m_sinusoids = new ComplexArray[m_arfcns];

    unsigned int outLen = BITS_PER_TIMESLOT * m_oversample;
    for (int i = 0;i < (int)m_arfcns;i++) {
	float exp = -2.0F * PI * ((float)((4.0F * i - 6.0F) * 100000.0F) / ((float)m_oversample * GSM_SYMBOL_RATE));
	m_sinusoids[i].assign(outLen);
	for (unsigned int j = 0;j < outLen;j++) {
	    Complex::exp(m_sinusoids[i][j],0,-exp*j);
	}
    }
}


//
// SigProcUtils
//
// Copy memory
unsigned int SigProcUtils::copy(void* dest, unsigned int len, const void* src,
    unsigned int n, unsigned int objSize, unsigned int offs)
{
    if (!(dest && src && n && objSize) || offs >= len)
	return 0;
    n = sigProcMin(n,len - offs);
    ::memcpy(((uint8_t*)dest) + offs * objSize,src,n * objSize);
    return n;
}

// Reset memory (set to 0)
unsigned int SigProcUtils::bzero(void* buf, unsigned int len,
    unsigned int objSize, unsigned int offs)
{
    if (!(buf && len && objSize) || offs >= len)
	return 0;
    len = sigProcMin(len,len - offs);
    ::memset(((uint8_t*)buf) + offs * objSize,0,len * objSize);
    return len;
}

/* vi: set ts=8 sw=4 sts=4 noet: */

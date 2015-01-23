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

// Laurent Pulse Approximation default table
static const float s_laurentPADef[] = {
    0.00121, 0.00469, 0.01285, 0.02933, 0.05858, 0.10498, 0.17141, 0.25819,
    0.36235, 0.47770, 0.59551, 0.70589, 0.79973, 0.87026, 0.91361, 0.92818,
    0.91361, 0.87026, 0.79973, 0.70589, 0.59551, 0.47770, 0.36235, 0.25819,
    0.17141, 0.10498, 0.05858, 0.02933, 0.01285, 0.00469, 0.00121
};

static inline unsigned int sigProcIters(unsigned int len, unsigned int step)
{
    if (step < 2)
	return len;
    return (len + (step - 1)) / step;
}

static inline unsigned int safeStrLen(const char* str)
{
    return (!TelEngine::null(str)) ? (unsigned int)::strlen(str) : 0;
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

// Set complex elements from 16 bit integers array (pairs of real/imaginary parts)
void Complex::setInt16(Complex* dest, unsigned int len, const int16_t* src, unsigned int n,
    unsigned int offs)
{
    if (!(dest && len && src && n) || offs >= len)
	return;
    unsigned int room = len - offs;
    unsigned int full = SigProcUtils::min(room,n / 2);
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
SignalProcessing::SignalProcessing()
    : m_oversample(0),
    m_gsmSlotLen(0),
    m_rampOffset(0),
    m_rampTrailIdx(0)
{
    setOversample(1);
}

void SignalProcessing::initialize(unsigned int oversample, unsigned int arfcns,
    LaurentPATable lpaTbl)
{
    setOversample(oversample);
    generateLaurentPulseAproximation(m_laurentPA,lpaTbl,m_oversample);
    generateARFCNsFreqShift(m_arfcnFS,arfcns,m_oversample);
}

void SignalProcessing::modulate(ComplexVector& out, const uint8_t* b, unsigned int len,
    FloatVector* tmpV, ComplexVector* tmpW) const
{
    static const float s_real[] = {1.0F, 0.0F, -1.0F, 0.0F};
    static const float s_imag[] = {0.0F, 1.0F, 0.0F, -1.0F};
    FloatVector localV;
    ComplexVector localW;

    if (!(b && len)) {
	out.resize(m_gsmSlotLen,true);
	return;
    }
    if (!tmpV)
	tmpV = &localV;
    if (!tmpW)
	tmpW = &localW;

    // Calculate v: v[n] = 2 * buf[n] - 1
    // David Burgess: Note the shift to allow for power ramp shaping
    tmpV->resize(m_gsmSlotLen,true);
    float* v = tmpV->data() + m_rampOffset;
    unsigned int n = sigProcIters(m_gsmSlotLen - m_rampOffset,m_oversample);
    for (; len && n; --len, --n, ++b, v += m_oversample)
	*v = 2.0F * *b - 1.0F;
    v = tmpV->data();
    // David Burgess's comment:
    //   The spec for power ramping is GSM 05.05 Annex B.
    //   The signal must start down-ramp within 10 us (2.7 symbols), down at least 6 dB.
    //   The signal falls at least 30 dB within 18 us (4.9 symbols); 6 dB per symbol.
    //   -6 dB is amplitude factor of 0.5.
    if (m_rampOffset) {
	// Power ramping - leading edge.
	v[m_rampOffset - m_oversample] = v[m_rampOffset] * 0.5F;
	// Power ramping - trailing edge
	v[m_rampTrailIdx] = v[m_rampTrailIdx - m_oversample] * 0.5F;
    }

    // Calculate w: w[n] = v[n] * s[n]
    // We exploit the fact that most of the values in v[] and w[] are zero.
    tmpW->resize(m_gsmSlotLen + m_laurentPA.length(),true);
    Complex* w = tmpW->data() + m_laurentPA.length() / 2;
    n = sigProcIters(m_gsmSlotLen,m_oversample);
    unsigned int idx = 0;
    for (; n; --n, idx = (idx + 1) % 4, w += m_oversample, v += m_oversample)
	w->set(*v * s_real[idx],*v * s_imag[idx]);

    // Calculate the convolution
    convolution(out,*tmpW,m_laurentPA);
}

// Calculate the convolution of 2 vectors
// out[n] = SUM(i=0..Lp)(f[n + i] * g[Lp - 1 - i])
// Where Lp is the g vector length
// We assume the 'f' vector is padded with Lp/2 0 values (leading and trailing)
void SignalProcessing::convolution(ComplexVector& out, const ComplexVector& fVect,
    const FloatVector& gVect)
{
    if (!gVect.length() || fVect.length() < gVect.length()) {
	out.resize(fVect.length(),true);
	return;
    }
    out.resize(fVect.length() - gVect.length());
    Complex* x = out.data();
    unsigned int Lp_1 = gVect.length() - 1;
    const Complex* parseF = fVect.data();
    const float* gLast = gVect.data() + Lp_1;
    for (unsigned int n = out.length(); n; --n, ++x, ++parseF) {
	const Complex* f = parseF;
	const float* g = gLast;
	// We don't reset the destination vector (it might contain old data)
	// For the first index just set the product in destination
	Complex::multiplyF(*x,*f,*g);
	// Multiply the remaining
	for (unsigned int conv = Lp_1; conv; --conv) {
	    ++f;
	    --g;
	    Complex::sumMulF(*x,*f,*g);
	}
    }
}

// Sum the ARFCN frequency shift vectors
// out[n] = SUM(i=0..k)(m_arfcnFS[i][n])
void SignalProcessing::sumFreqShift(ComplexVector& out, unsigned int arfcnMask) const
{
    if (!m_arfcnFS.length()) {
	out.clear();
	return;
    }
    bool first = true;
    for (unsigned int i = 1; i < m_arfcnFS.length(); ++i, arfcnMask >>= 1)
	if ((arfcnMask & 0x01) != 0)
	    sum(out,m_arfcnFS[i],first);
    if (first)
	out.resize(m_arfcnFS[0].length(),true);
}

void SignalProcessing::generateLaurentPulseAproximation(FloatVector& out,
    LaurentPATable lpaTbl, unsigned int oversample)
{
    if (lpaTbl == LaurentPADef)
	out.assign(s_laurentPADef,sizeof(s_laurentPADef) / sizeof(float));
    else {
	Debug(DebugStub,
	    "SignalProcessing::generateLaurentPulseAproximation() not tested for tbl=%d oversample=%u",
	    lpaTbl,oversample);
	unsigned int K = oversample ? oversample : 1;
	out.resize(4 * K);
	float* d = out.data();
	float halfLen = (float)out.length() / 2;
	float sampled = 0.96 / K;
	for (unsigned int n = 0; n < out.length(); n++) {
	    float f = ((float)n - halfLen) / K;
	    float f2 = f * f;
	    float g = -1.138 * f2 - 0.527 * f2 * f2;
	    *d++ = sampled * ::expf(g);
	}
    }
#ifdef XDEBUG
    String s;
    out.dump(s,SigProcUtils::appendFloat);
    Debug(DebugAll,
	"SignalProcessing::generateLaurentPulseAproximation(%d,%u) len=%u data: %s",
	lpaTbl,oversample,out.length(),s.c_str());
#endif
}

// Generate GMSK frequency shifting vector
static void generateGMSKFreqShift(ComplexVector& out, float omega)
{
    Complex* s = out.data();
    float phi = 0;
    for (unsigned int n = out.length(); n; --n, ++s) {
	s->set(::cosf(phi),::sinf(phi));
	phi += omega;
	if (phi > PI2)
	    phi -= PI2;
    }
}

// Generate ARFCNs frequency shifting vectors
// fk = (4 * k - 6)(100 * 10^3)
// omegaK = (2 * PI * fk) / Fs
// s[n] = e ^ (-j * n * omegaK)
void SignalProcessing::generateARFCNsFreqShift(ComplexVectorVector& out,
    unsigned int arfcns, unsigned int oversample)
{
    unsigned int K = oversample ? oversample : 1;
    unsigned int Ls = gsmSlotLen(K);
    float Fs = GSM_SYMBOL_RATE * K;
    out.resize(arfcns);
    for (unsigned int i = 0; i < out.length(); ++i) {
	out[i].resize(Ls);
	float fk = (4 * (float)i - 6) * 1e5;
	float omega = (PI2 * fk) / Fs;
	generateGMSKFreqShift(out[i],omega);
#ifdef XDEBUG
	String s;
	out[i].dump(s,SigProcUtils::appendComplex);
	Debug(DebugAll,
	    "SignalProcessing::generateARFCNsFreqShift(%u) ARFCN=%u len=%u data: %s",
	    oversample,i,out[i].length(),s.c_str());
#endif
    }
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

// Init oversampling rate and related data
void SignalProcessing::setOversample(unsigned int oversample)
{
    m_oversample = oversample ? oversample : 1;
    m_gsmSlotLen = gsmSlotLen(m_oversample);
    // Calculate power ramping 
    // Make sure the indexes are not out of range
    // Disable power ramping if so
    m_rampOffset = 4 * m_oversample;
    unsigned int* wrong = 0;
    if (m_rampOffset < m_gsmSlotLen) {
	// Power ramping - trailing edge
	// v[m_rampTrailIdx] = v[m_rampTrailIdx - m_oversample]
	m_rampTrailIdx = 147 * m_oversample + m_rampOffset + m_oversample;
	if (m_rampTrailIdx >= m_gsmSlotLen)
	    wrong = &m_rampTrailIdx;
    }
    else
	wrong = &m_rampOffset;
    if (wrong) {
	Debug(DebugFail,
	    "SignalProcessing oversample=%u: power ramping %s %u is out of range",
	    m_oversample,((wrong == &m_rampOffset) ? "offset" : "trailing index"),
	    *wrong);
	m_rampOffset = 0;
	m_rampTrailIdx = 0;
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
    n = min(n,len - offs);
    ::memcpy(((uint8_t*)dest) + offs * objSize,src,n * objSize);
    return n;
}

// Reset memory (set to 0)
unsigned int SigProcUtils::bzero(void* buf, unsigned int len,
    unsigned int objSize, unsigned int offs)
{
    if (!(buf && len && objSize) || offs >= len)
	return 0;
    len = min(len,len - offs);
    ::memset(((uint8_t*)buf) + offs * objSize,0,len * objSize);
    return len;
}

// Copy string, advance dest and src, return src
static inline const char* copyInc(char*& dest, const char* src, unsigned int n)
{
    ::strncpy(dest,src,n);
    dest += n;
    return src + n;
}

String& SigProcUtils::appendSplit(String& buf, const String& str, unsigned int lineLen,
    unsigned int offset, const char* linePrefix, const char* suffix)
{
    if (TelEngine::null(linePrefix))
	linePrefix = suffix;
    unsigned int len = str.length();
    unsigned int linePrefLen = safeStrLen(linePrefix);
    // No lines ?
    if (!(lineLen && len && linePrefLen) || lineLen >= len) {
	buf << str << suffix;
	return buf;
    }
    unsigned int firstLineLen = 0;
    if (offset && offset < lineLen) {
	firstLineLen = min(len,lineLen - offset);
	len -= firstLineLen;
	// Nothing to be added after first line ?
	if (!len) {
    	    buf << str << suffix;
	    return buf;
	}
    }
    unsigned int nFullLines = len / lineLen;
    unsigned int lastLineLen = len % lineLen;
    unsigned int suffixLen = safeStrLen(suffix);
    unsigned int nSep = nFullLines + (lastLineLen ? 1 : 0);
    char* tmpBuf = new char[str.length() + nSep * linePrefLen + suffixLen + 1];
    char* dest = tmpBuf;
    const char* src = str.c_str();
    if (firstLineLen)
	src = copyInc(dest,src,firstLineLen);
    for (; nFullLines; nFullLines--) {
	copyInc(dest,linePrefix,linePrefLen);
	src = copyInc(dest,src,lineLen);
    }
    if (lastLineLen) {
	copyInc(dest,linePrefix,linePrefLen);
	src = copyInc(dest,src,lastLineLen);
    }
    if (suffixLen)
	copyInc(dest,suffix,suffixLen);
    *dest = 0;
    buf << tmpBuf;
    delete[] tmpBuf;
    return buf;
}

// Append float value to a String (using %g format)
String& SigProcUtils::appendFloat(String& dest, const float& val, const char* sep)
{
    char s[80];
    sprintf(s,"%g",val);
    return dest.append(s,sep);
}

// Append a Complex number to a String (using "%g%+gi" format)
String& SigProcUtils::appendComplex(String& dest, const Complex& val, const char* sep)
{
    char s[170];
    sprintf(s,"%g%+gi",val.real(),val.imag());
    return dest.append(s,sep);
}

/* vi: set ts=8 sw=4 sts=4 noet: */

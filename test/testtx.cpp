/**
 * testtx.cpp
 * This file is part of the Yate-BTS Project http://www.yatebts.com
 *
 * Transceiver TX test
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

#include "transceiver.h"
#include <yatengine.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

using namespace TelEngine;

// GSM training sequences
static int s_normalTraining[8][26] = {
    {0,0,1,0,0,1,0,1,1,1,0,0,0,0,1,0,0,0,1,0,0,1,0,1,1,1},
    {0,0,1,0,1,1,0,1,1,1,0,1,1,1,1,0,0,0,1,0,1,1,0,1,1,1},
    {0,1,0,0,0,0,1,1,1,0,1,1,1,0,1,0,0,1,0,0,0,0,1,1,1,0},
    {0,1,0,0,0,1,1,1,1,0,1,1,0,1,0,0,0,1,0,0,0,1,1,1,1,0},
    {0,0,0,1,1,0,1,0,1,1,1,0,0,1,0,0,0,0,0,1,1,0,1,0,1,1},
    {0,1,0,0,1,1,1,0,1,0,1,1,0,0,0,0,0,1,0,0,1,1,1,0,1,0},
    {1,0,1,0,0,1,1,1,1,1,0,1,1,0,0,0,1,0,1,0,0,1,1,1,1,1},
    {1,1,1,0,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,0,1,1,1,1,0,0},
};

static const String s_arfcnIn0('0',GSM_BURST_LENGTH);
static const String s_arfcnIn1('1',GSM_BURST_LENGTH);
static const float s_gsmSymbolRate = 13e6 / 48.0;

enum CompareSource
{
	CmpNone = 0,
	CmpFromSignalProcessing,
	CmpFromCfg,
};

const TokenDict s_cmpName[] =
{
	{"signalprocessing", CmpFromSignalProcessing},
	{"config", CmpFromCfg},
	{"none", CmpNone},
	{0,0},
};

static int s_code = 0;
static Configuration s_cfg;
static unsigned int s_arfcns = 0;
static unsigned int s_oversampling = 8;
static int s_cmpSrc = CmpNone;
static String s_extraInputParams;
static String s_extraResultParams;
// Output
static unsigned int s_lineLen = 120;
static unsigned int s_lineDec = 2;
static bool s_debug = false;
static bool s_dumpStr = true;
static bool s_dumpHex = true;
static bool s_confFileStyle = true;
static String s_linePrefix;
static SignalProcessing s_signalProcessing;

enum CheckPoint {
	Input = 0,
	LaurentPA,
	LaurentFS,
	ARFCNv,
	ARFCNw,
	ARFCNx,
	ARFCNs,
	ARFCNTx,
	FreqShift,
	EngOut,
	CheckPointOk
};

const TokenDict s_pointLabel[] =
{
	{"Input", Input},
	{"Laurent pulse approximation", LaurentPA},
	{"Laurent frequency shifting", LaurentFS},
	{"v", ARFCNv},
	{"w", ARFCNw},
	{"modulated", ARFCNx},
	{"tx", ARFCNTx},
	{"frequency_shifting", ARFCNs},
	{"Frequency shifted", FreqShift},
	{"energize_out", EngOut},
	{0,0},
};

static inline const char* pointLabel(int point)
{
	return lookup(point,s_pointLabel);
}

String& appendUInt8(String& dest, const uint8_t& val, const char* sep)
{
	char s[80];
	sprintf(s,"%u",val);
	return dest.append(s,sep);
}

String& appendInt(String& dest, const int& val, const char* sep)
{
    char s[80];
    sprintf(s,"%d",val);
    return dest.append(s,sep);
}

static unsigned int addPName(String& buf, const String& pName, const char* suffix)
{
	if (!pName)
		return 0;
	String p(pName + suffix);
	buf << p;
	if (!s_lineDec || s_lineDec > p.length())
		return p.length();
	if (s_lineLen)
		return p.length() - s_lineDec;
	return p.length();
}

static inline bool invalidConfParam(const char* p, const char* v, const char* sect)
{
	Debug(DebugConf,"Invalid %s='%s' in '%s' section",p,v,sect);
	return false;
}

class TestDataVectorIface
{
public:
	virtual unsigned int dataLen() const  = 0;
	virtual unsigned int compareLen() const  = 0;
	virtual String& appendFromIndex(String& dest, unsigned int index, bool data) = 0;
	virtual String& dump(String& dest, const String& pName = String::empty(),
	bool* dumpStr = 0, bool* dumpHex = 0, const char* sep = ",",
	bool data = true, unsigned int n = 0) const  = 0;
	virtual bool compare(unsigned int& result) = 0;
};

template <class Obj, bool basicType, String& (*funcAppendToStr)(String& dest, const Obj& item, const char* sep)>
class TestDataVector : public TestDataVectorIface
{
public:
	void reset() {
		m_data.clear();
		m_compare.clear();
	}

	virtual unsigned int dataLen() const
	{ return m_data.length(); }

	virtual unsigned int compareLen() const
	{ return m_compare.length(); }

	virtual String& appendFromIndex(String& dest, unsigned int index, bool data) {
		const SigProcVector<Obj,basicType>& what = data ? m_data : m_compare;
		if (index < what.length())
			return (*funcAppendToStr)(dest,what[index],0);
		return dest;
	}

	bool readBits(const String& src) {
		if (m_data.length() > src.length())
			return false;
		for (unsigned int i = 0; i < m_data.length(); i++) {
			char c = src[i];
			if (c != '1' && c != '0')
				return false;
			m_data[i] = (Obj)(c - '0');
		}
		return true;
	}

	virtual String& dump(String& dest, const String& pName = String::empty(),
		bool* dumpStr = 0, bool* dumpHex = 0, const char* sep = ",",
		bool data = true, unsigned int n = 0) const {
		if (!dumpStr)
			dumpStr = &s_dumpStr;
		if (!dumpHex)
			dumpHex = &s_dumpHex;
		if (!(*dumpStr || *dumpHex))
			return dest;
		const SigProcVector<Obj,basicType>* what = data ? &m_data : &m_compare;
		SigProcVector<Obj,basicType> tmp;
		if (n && n < what->length()) {
			tmp.resize(n);
			tmp.copy(what->data(),n);
			what = &tmp;
		}
		if (pName)
			dest << pName << "_length=" << what->length() << "\r\n";
		unsigned int first = 0;
		if (*dumpStr) {
			first = addPName(dest,pName,"_str=");
			if (!s_lineLen)
				what->dump(dest,funcAppendToStr,sep) += "\r\n";
			else
				what->appendSplit(dest,s_lineLen,funcAppendToStr,first,s_linePrefix,"\r\n",sep);
		}
		if (!*dumpHex)
			return dest;
		first = addPName(dest,pName,"_hex=");
		return what->appendSplitHex(dest,s_lineLen,first,s_linePrefix,"\r\n");
	}

	// Unhexify compare from config param
	int unHexify(const String& sect, const String& param) {
		NamedString* tmp = s_cfg.getKey(sect,param);
		int res = 0;
		if (!TelEngine::null(tmp))
			res = m_compare.unHexify(tmp->c_str(),tmp->length());
		else
			m_compare.clear();
		if (res != 0) {
			const char* what = (res > 0 ? "length" : "format");
			if (s_debug)
				Debug(DebugNote,
					"Invalid hex data %s param=%s in section [%s]",
					what,param.c_str(),sect.c_str());
			s_extraInputParams << "invalid_input_skip_compare=invalid hex data " <<
				what << " for '" << param << "' section=[" << sect << "]\r\n";
		}
		else if (s_debug && !TelEngine::null(tmp))
			Debug(DebugAll,
				"Read %u elements from hex data param=%s in section [%s]",
				m_compare.length(),param.c_str(),sect.c_str());
		return res;
	}

	// Compare data if m_compare length is not 0. Result is set only if comparing
	virtual bool compare(unsigned int& result) {
		if (!m_compare.length())
			return true;
		result = m_data.compare(m_compare);
		return result == m_data.length();
	}

	SigProcVector<Obj,basicType> m_data;
	SigProcVector<Obj,basicType> m_compare;
};

typedef TestDataVector<uint8_t,true,appendUInt8> TestUint8Unit;
typedef TestDataVector<float,true,SigProcUtils::appendFloat> TestFloatUnit;
typedef TestDataVector<Complex,false,SigProcUtils::appendComplex> TestComplexUnit;
typedef TestDataVector<int,true,appendInt> TestIntUnit;

class TestARFCN : public String
{
public:

	inline TestARFCN()
	: m_filler(false), m_inPower(0), m_arfcn(0xffffffff)
	{}

	inline unsigned int arfcn() const
	{ return m_arfcn; }

	inline void setArfcn(unsigned int arfcn) {
		assign("arfcn") += arfcn;
		m_arfcn = arfcn;
	}

	bool m_filler;
	float m_inPower;
	TestUint8Unit m_inBits;
	TestFloatUnit m_v;
	TestComplexUnit m_w;
	TestComplexUnit m_x;
	TestComplexUnit m_s;
	TestComplexUnit m_tx;

private:
	unsigned int m_arfcn;

};

class BuildTx
{
public:
	BuildTx();

	inline ~BuildTx()
	{ resetArfcns(); }

	bool test();

	bool checkPoint(unsigned int point, TestARFCN* arfcn = 0);

protected:

	void saveResult();

	void resetArfcns(unsigned int n = 0);

	unsigned int m_checkPoint;
	unsigned int m_failedArfcn;
	unsigned int m_arfcnCount;
	String m_failedPoint;
	TestARFCN* m_arfcns;
	TestFloatUnit m_laurentPA;
	TestComplexUnit m_laurentFS;
	TestComplexUnit m_y;
	TestIntUnit m_out;
};


//
// BuildTx
//
BuildTx::BuildTx()
	: m_checkPoint(0),
	m_failedArfcn(0xffffff),
	m_arfcnCount(0),
	m_arfcns(0)
{
	NamedList* params = s_cfg.createSection("general");
	resetArfcns(s_arfcns);
	unsigned int i = 0;
	for (; i < m_arfcnCount; i++) {
		TestARFCN& a = m_arfcns[i];
		const String& name = a;
		String s = params->getValue(a);
		a.m_filler = (s == "filler");
		if (!a.m_filler) {
			if (s.isBoolean())
				// Configuratio of "true" or "false" selects all-1 or all-0 burst.
				s = s.toBoolean() ? s_arfcnIn1: s_arfcnIn0;
			else if (!s) {
				// If nothing specfied, build a normal burst with random payload fields.
				// The fields of the normal burst are defined in GSM 05.03 3.1.4 and GSM 05.01 5.2.
				static const unsigned s_tailLen = 3;
				static const unsigned s_payloadLen = 58;
				static const unsigned s_midambleLen = 26;
				static const char* s_digits[2]={"0","1"};
				String tail('0',s_tailLen);
				// tail bits
				s << tail;
				// payload bits
				for (unsigned i=0; i<s_payloadLen; i++)
					s.append(s_digits[random()%2]);
				// midamble
				for (unsigned i=0; i<s_midambleLen; i++)
					s.append(s_digits[s_normalTraining[2][i]]);
				// payload bits
				for (unsigned i=0; i<s_payloadLen; i++)
					s.append(s_digits[random()%2]);
				// tail bits
				s << tail;
			}
			a.m_inBits.m_data.resize(GSM_BURST_LENGTH);
			if (!a.m_inBits.readBits(s)) {
				invalidConfParam(name,s,*params);
				break;
			}
			a.m_inPower = 1.0;
		}
		switch (s_cmpSrc) {
			case CmpFromCfg:
#define ARFCN_TEST_UNHEXIFY(vect,point) a.vect.unHexify(name,name + "_" + pointLabel(point) + "_hex")
				if (!a.m_filler) {
					ARFCN_TEST_UNHEXIFY(m_v,ARFCNv);
					ARFCN_TEST_UNHEXIFY(m_w,ARFCNw);
					ARFCN_TEST_UNHEXIFY(m_x,ARFCNx);
					ARFCN_TEST_UNHEXIFY(m_tx,ARFCNTx);
				}
				ARFCN_TEST_UNHEXIFY(m_s,ARFCNs);
#undef ARFCN_TEST_UNHEXIFY
				break;
			case CmpFromSignalProcessing:
				s_signalProcessing.modulate(a.m_x.m_compare,
							    a.m_inBits.m_data.data(),
							    a.m_inBits.m_data.length(),
							    &a.m_v.m_compare,&a.m_w.m_compare);
				// The signal processing added padding to 'w'
				if (a.m_w.m_compare.length() > a.m_x.m_compare.length()) {
					unsigned int wLen = a.m_w.m_compare.length();
					unsigned int diff = wLen - a.m_x.m_compare.length();
					a.m_w.m_compare.assignSlice(diff / 2,wLen - diff);
				}
				else if (a.m_w.m_compare.length() < a.m_x.m_compare.length())
					a.m_w.m_compare.clear();
				a.m_s.m_compare = s_signalProcessing.arfcnFS(i);
				a.m_tx.m_compare = a.m_x.m_compare;
				s_signalProcessing.freqShift(a.m_tx.m_compare,i);
				break;
		}
	}
	if (i < m_arfcnCount) {
		resetArfcns();
		return;
	}
	switch (s_cmpSrc) {
		case CmpFromCfg:
#define BUILD_TEST_UNHEXIFY(vect,point) vect.unHexify(pointLabel(point),"data_hex")
			BUILD_TEST_UNHEXIFY(m_laurentPA,LaurentPA);
			BUILD_TEST_UNHEXIFY(m_laurentFS,LaurentFS);
			BUILD_TEST_UNHEXIFY(m_y,FreqShift);
			BUILD_TEST_UNHEXIFY(m_out,EngOut);
#undef BUILD_TEST_UNHEXIFY
			break;
		case CmpFromSignalProcessing:
			m_laurentPA.m_compare = s_signalProcessing.laurentPA();
			m_y.m_compare.clear();
			for (unsigned int i = 0; i < m_arfcnCount; i++) {
				TestARFCN& a = m_arfcns[i];
				if (a.m_filler)
					continue;
				bool first = m_y.m_compare.length() == 0;
				SignalProcessing::sum(m_y.m_compare,a.m_tx.m_compare,first);
			}
			m_out.m_compare.resize(m_y.m_compare.length() * 2);
			int* out = m_out.m_compare.data();
			unsigned int clamp = 0;
			for (unsigned int y = 0;y < m_y.m_compare.length();y++,out++) {
			    (*out) = ::round(SigProcUtils::energize(m_y.m_compare[y].real(),2047,1,clamp));
			    out++;
			    (*out) = ::round(SigProcUtils::energize(m_y.m_compare[y].imag(),2047,1,clamp));
			}
			break;
	}
}

// Test
bool BuildTx::test()
{
	if (!m_arfcns)
		return false;

#define CHECKPOINT(point,arfcn) \
	m_checkPoint = point; \
	if (!checkPoint(point,arfcn)) \
	break

	// By hardcoding the Laurent pulse oversampling is hardcoded to 8.
	// But the QMF design in the receive side hardcodes the oversampling to 8 anyway.
	// The 31-point filter gives a noise level of -45 dB.
	static const unsigned Lp = 31;
	static const float c0[Lp] = {0.00121, 0.00469, 0.01285, 0.02933, 0.05858, 0.10498, 0.17141, 0.25819, 0.36235, 0.47770, 0.59551, 0.70589, 0.79973, 0.87026, 0.91361, 0.92818, 0.91361, 0.87026, 0.79973, 0.70589, 0.59551, 0.47770, 0.36235, 0.25819, 0.17141, 0.10498, 0.05858, 0.02933, 0.01285, 0.00469, 0.00121};
	// The 29-point filter gives a noise level of -40 dB.
	//static const unsigned Lp = 29;
	//static const float c0[Lp] = {0.00469, 0.01285, 0.02933, 0.05858, 0.10498, 0.17141, 0.25819, 0.36235, 0.47770, 0.59551, 0.70589, 0.79973, 0.87026, 0.91361, 0.92818, 0.91361, 0.87026, 0.79973, 0.70589, 0.59551, 0.47770, 0.36235, 0.25819, 0.17141, 0.10498, 0.05858, 0.02933, 0.01285, 0.00469};
	// The 27-point filter gives a noise level of -30 dB.
	//static const unsigned Lp = 27;
	//static const float c0[Lp] = {0.01285, 0.02933, 0.05858, 0.10498, 0.17141, 0.25819, 0.36235, 0.47770, 0.59551, 0.70589, 0.79973, 0.87026, 0.91361, 0.92818, 0.91361, 0.87026, 0.79973, 0.70589, 0.59551, 0.47770, 0.36235, 0.25819, 0.17141, 0.10498, 0.05858, 0.02933, 0.01285};
	// The 23-point filter gives a noise level of -30 dB.
	//static const unsigned Lp = 23;
	//static const float c0[Lp] = {0.02692, 0.07322, 0.14196, 0.23320, 0.34342, 0.46566, 0.59039, 0.70711, 0.80643, 0.88108, 0.92690, 0.94228, 0.92690, 0.88108, 0.80643, 0.70711, 0.59039, 0.46566, 0.34342, 0.23320, 0.14196, 0.07322, 0.02692};

	while (true) {
		m_failedArfcn = 0xffffff;
		CHECKPOINT(Input,0);
		// General input:
		// K: oversampling (s_oversampling)
		float K = s_oversampling;
		// F0: GSM symbol rate, s_gsmSymbolRate
		float F0 = s_gsmSymbolRate;
		// Fs: sample rate K * F0
		float Fs = K * F0;
		// Ls: the length of a GSM slot sampled at K
		unsigned int Ls = 156.25 * K;
		// Generate Laurent pulse approximation sampled at Fs
		// Extending this to 5 symbols does not change performance.
		//unsigned int Lp = 3 * s_oversampling - 1;
		m_laurentPA.m_data.resize(Lp);
		float* hp = m_laurentPA.m_data.data();
		for (unsigned int n = 0; n < m_laurentPA.m_data.length(); n++) {
#if 0
			float f = ((float)n - (Lp / 2)) / K;
			float f2 = f * f;
			// DAB - Note sign error in the first term in the line below.
			float g = -1.138 * f2 - 0.527 * f2 * f2;
			//hp[n] = 0.96 * ::expf(g);
#endif
			hp[n] = 0.90*c0[n];
		}
		CHECKPOINT(LaurentPA,0);
		// Generate Laurent frequency shift vector
		// s[n] = e ^ ((j * n * PI) / (2 * K))
		m_laurentFS.m_data.resize(Ls);
		Complex* s = m_laurentFS.m_data.data();
		float omega = PI / (2.0F*K);
		float phi = 0;
		for (unsigned int n = 0; n < m_laurentFS.m_data.length(); n++) {
			s[n].real(::cosf(phi));
			s[n].imag(::sinf(phi));
			phi += omega;
			if (phi>PI2) phi -= PI2;
		}
		CHECKPOINT(LaurentFS,0);
		// Modulate each ARFCN
		for (unsigned int i = 0; i < m_arfcnCount; i++) {
			TestARFCN& a = m_arfcns[i];
			if (!a.m_filler) {
				// Modulate -----------------------
				// Data:
				// b: Channel bits
				// x: modulated output at rate Fs
				// hp: Laurent pulse approximation sampled at Fs, length: Lp
				// s: complex-valued frequency shift sequence
				uint8_t* b = a.m_inBits.m_data.data();
				// Calculate v: v[n] = 2 * b[n] * floor(n / K) - 1
				a.m_v.m_data.assign(Ls);
				float* v = a.m_v.m_data.data();
				static const int rampOffset = 4 * s_oversampling;
				for (unsigned int n = 0; n < a.m_v.m_data.length(); n++) {
					if (n%s_oversampling) continue;
					unsigned int idx = n / s_oversampling;;
					if (idx >= a.m_inBits.m_data.length())
						break;
					// DAB - Note the shift to allow for power ramp shaping.
					int nCurr = n + rampOffset;
					v[nCurr] = 2.0F * b[idx] - 1.0F;
				}

				// The spec for power ramping is GSM 05.05 Annex B.
				// The signal must start down-ramp within 10 us (2.7 symbols), down at least 6 dB.
				// The signal falls at least 30 dB within 18 us (4.9 symbols); 6 dB per symbol.
				// -6 dB is amplitude factor of 0.5.

				// DAB - Power ramping - leading edge.
				float vLead = v[rampOffset];
				v[rampOffset - s_oversampling] = vLead * 0.5F;

				// DAB - Power ramping - trailing edge.
				float vTrail = v[147*s_oversampling+rampOffset];
				v[147*s_oversampling+rampOffset+s_oversampling] = vTrail * 0.71F;

				CHECKPOINT(ARFCNv,&a);
				// Calculate w: w[n] = v[n] * s[n]
				// DAB - Note the we are no longer using the Laurent shift vector caclulcated earlier.
				// This is faster and more accurate.
				// We exploit the fact that most of the values in v[] and w[] are zero.
				a.m_w.m_data.resize(a.m_v.m_data.length());
				Complex* w = a.m_w.m_data.data();
				static const float s1[] = {1.0F, 0.0F, -1.0F, 0.0F};
				static const float s2[] = {0.0F, 1.0F, 0.0F, -1.0F};
				int n2=0;
				for (unsigned int n = 0; n < a.m_w.m_data.length(); n += s_oversampling) {
					w[n].real(s1[n2%4] * v[n]);
					w[n].imag(s2[n2%4] * v[n]);
					n2++;
				}
				CHECKPOINT(ARFCNw,&a);
				// Modulate, build w padded with Lp/2 elements at each end
				// x[n] = SUM(i=0..Lp)(w[n + i] * hp[Lp - 1 - i])
				ComplexVector neww(a.m_w.m_data.length() + Lp);
				neww.copy(a.m_w.m_data.data(),a.m_w.m_data.length(),Lp / 2);
				w = neww.data();
				a.m_x.m_data.assign(a.m_w.m_data.length());
				Complex* x = a.m_x.m_data.data();
				for (unsigned int n = 0; n < a.m_x.m_data.length(); n++)
					for (unsigned int i = 0; i < Lp; i++)
						x[n] += w[n + i] * hp[Lp - 1 - i];
			}
			else
				// DAB - Note typo in line below: Ls not Fs.
				a.m_x.m_data.assign(Ls);

			CHECKPOINT(ARFCNx,&a);
			// Frequency shifting vector
			// fk = (4 * k - 6)(100 * 10^3)
			// omegaK = (2 * PI * fk) / Fs
			// s[n] = e ^ (-j * n * omegaK)
			a.m_s.m_data.resize(a.m_x.m_data.length());
			// DAB - Note the critical cast to a signed type in the line below.
			const float fk = (4 * (int)a.arfcn() - 6) * 1e5;
			const float omegaK = (PI2 * fk) / Fs;
			float phi = 0.0F;
			for (unsigned int n = 0; n < a.m_s.m_data.length(); n++) {
				a.m_s.m_data[n].real(::cosf(phi));
				a.m_s.m_data[n].imag(::sinf(phi));
				// DAB - This incrementing approach gives a smaller cumulative error.
				phi += omegaK;
				if (phi>PI2) phi -= PI2;
			}
			CHECKPOINT(ARFCNs,&a);
			
			// TX data
			a.m_tx.m_data.assign(a.m_x.m_data.length());
			Complex* x = a.m_x.m_data.data();
			Complex* s = a.m_s.m_data.data();
			Complex* tx = a.m_tx.m_data.data();
			for (unsigned int n = 0; n < a.m_tx.m_data.length(); n++)
			    tx[n] = x[n] * s[n];
			CHECKPOINT(ARFCNTx,&a);
		}

		if (m_failedArfcn < m_arfcnCount)
			break;
		// Frequency shifting
		// y[n] = SUM(i=0..k)(xi[n] * si[n])
		unsigned int k = m_arfcnCount;
		m_y.m_data.assign(Ls);
		for (unsigned int n = 0; n < m_y.m_data.length(); n++) {
			for (unsigned int i = 0; i < k; i++) {
				Complex* x = m_arfcns[i].m_x.m_data.data();
				Complex* s = m_arfcns[i].m_s.m_data.data();
				m_y.m_data[n] += x[n] * s[n];
			}
		}
		CHECKPOINT(FreqShift,0);
		// Clip the value into the +/-1 range.
		unsigned clipped = 0;
		m_out.m_data.resize(m_y.m_data.length() * 2);
		int* out = m_out.m_data.data();
		for (unsigned int n = 0; n < m_y.m_data.length(); n++) {
			Complex r = m_y.m_data[n];
			if (r.real()>1.0) {
				m_y.m_data[n].real(1.0);
				clipped++;
			}
			else if (r.real()<-1.0) {
				m_y.m_data[n].real(-1.0);
				clipped++;
			}
			*out++ = ::round(m_y.m_data[n].real() * 2047);
			
			if (r.imag()>1.0) {
				m_y.m_data[n].imag(1.0);
				clipped++;
			}
			else if (r.imag()<-1.0) {
				m_y.m_data[n].imag(-1.0);
				clipped++;
			}
			*out++ = ::round(m_y.m_data[n].imag() * 2047);
		}
		if (clipped>0) {
			Debug(DebugAll, "%d outupt values clipped", clipped);
		}
		CHECKPOINT(EngOut,0);
		m_checkPoint = CheckPointOk;
		break;
	}
	saveResult();
	return m_checkPoint == CheckPointOk;
}


bool BuildTx::checkPoint(unsigned int point, TestARFCN* arfcn)
{
	TestDataVectorIface* v = 0;
	switch (point) {

#define SET_POINT_VECT_BREAK(cv,fv) { \
	v = cv ? static_cast<TestDataVectorIface*>(cv) : static_cast<TestDataVectorIface*>(fv); \
	break; \
	}

#define CASE_POINT_VECT(point,cv,fv) \
	case point: \
	SET_POINT_VECT_BREAK(cv,fv);

#define CASE_POINT_VECT_A(point,cv,fv) \
	case point: \
	if (!arfcn) \
		return false; \
	SET_POINT_VECT_BREAK(cv,fv)

		case Input:
			return true;
		CASE_POINT_VECT(LaurentPA,0,&m_laurentPA);
		CASE_POINT_VECT(LaurentFS,&m_laurentFS,0);
		CASE_POINT_VECT_A(ARFCNv,0,&arfcn->m_v);
		CASE_POINT_VECT_A(ARFCNw,&arfcn->m_w,0);
		CASE_POINT_VECT_A(ARFCNx,&arfcn->m_x,0);
		CASE_POINT_VECT_A(ARFCNs,&arfcn->m_s,0);
		CASE_POINT_VECT_A(ARFCNTx,&arfcn->m_tx,0);
		CASE_POINT_VECT(FreqShift,&m_y,0);
		CASE_POINT_VECT(EngOut,&m_out,0);
		default:
			return true;
	}

	bool ok = true;
	// Compare
	while (v) {
		String s;
		if (s_debug) {
			s << "Comparison for '" << pointLabel(point) << "' vector";
			if (arfcn)
				s << " (ARFCN=" << arfcn->arfcn() << ")";
		}
		unsigned int result = 0;
		if (v->compare(result)) {
			if (s) {
				if (v->compareLen())
					Debug(DebugInfo,"%s: PASSED",s.c_str());
				else
					Debug(DebugAll,"%s: no data",s.c_str());
			}
			break;
		}
		m_failedPoint.clear();
		m_failedPoint << "'" << pointLabel(point) << "'";
		if (arfcn) {
			m_failedArfcn = arfcn->arfcn();
			m_failedPoint << " ARFCN " << m_failedArfcn;
		}
		s_extraResultParams << "compare_failed=";
		if (result < v->dataLen()) {
			s_extraResultParams << "index " << result << "\r\n";
			s_extraResultParams << "generated_value=";
			v->appendFromIndex(s_extraResultParams,result,true) += "\r\n";
			s_extraResultParams << "compare_value=";
			v->appendFromIndex(s_extraResultParams,result,false) += "\r\n";
			if (s) {
				s << " FAILED index=" << result;
				s << " generated_value=";
				v->appendFromIndex(s,result,true);
				s << " compare_value=";
				v->appendFromIndex(s,result,false);
			}
		}
		else {
			s_extraResultParams << "vectors have different length\r\n";
			if (s)
				s << " FAILED: vectors have different length";
		}
		if (s)
			Debug(DebugNote,"%s",s.c_str());
		ok = false;
		break;
	}
	return ok;
}

static inline void addSectPoint(String& dest, int point)
{
	dest << "\r\n[" << pointLabel(point) << "]\r\n";
}

static inline String* createSectPoint(int point)
{
	String* s = new String;
	addSectPoint(*s,point);
	return s;
}

void BuildTx::saveResult()
{
	ObjList list;
	ObjList* o = &list;
	String* buf = 0;
	// Input:
	buf = createSectPoint(Input);
	*buf << "arfcns=" << m_arfcnCount << "\r\n";
	*buf << "oversampling=" << s_oversampling << "\r\n";
	for (unsigned int i = 0; i < m_arfcnCount; i++) {
		const String& name = m_arfcns[i];
		*buf << name << "_filler=" << String::boolText(m_arfcns[i].m_filler) << "\r\n";
		if (m_arfcns[i].m_filler)
			continue;
		*buf << name << "_power=";
		SigProcUtils::appendFloat(*buf,m_arfcns[i].m_inPower,0) += "\r\n";
		bool str = true;
		bool hex = false;
		m_arfcns[i].m_inBits.dump(*buf,name + "_bits",&str,&hex,0);
	}
	*buf << s_extraInputParams;
	o = o->append(buf);
	// Dump data
	if (s_dumpStr || s_dumpHex) {
		// Laurent approximation
		if (m_checkPoint >= LaurentPA) {
			buf = createSectPoint(LaurentPA);
			m_laurentPA.dump(*buf,"data");
			o = o->append(buf);
		}
		// Laurent freq shifting
		if (m_checkPoint >= LaurentFS) {
			buf = createSectPoint(LaurentFS);
			m_laurentFS.dump(*buf,"data");
			o = o->append(buf);
		}
		// ARFCNs data
		if (m_checkPoint >= ARFCNv) {
			for (unsigned int i = 0; i < m_arfcnCount; i++) {
				TestARFCN& a = m_arfcns[i];
				buf = new String;
				*buf << "\r\n[" << a << "]\r\n";
				if (!a.m_filler) {
					if (i < m_failedArfcn || m_checkPoint >= ARFCNv)
						a.m_v.dump(*buf,pointLabel(ARFCNv));
					if (i < m_failedArfcn || m_checkPoint >= ARFCNw)
						a.m_w.dump(*buf,pointLabel(ARFCNw));
					if (i < m_failedArfcn || m_checkPoint >= ARFCNx)
						a.m_x.dump(*buf,pointLabel(ARFCNx));
				}
				if (i < m_failedArfcn || m_checkPoint >= ARFCNs)
					a.m_s.dump(*buf,pointLabel(ARFCNs));
				o = o->append(buf);
				if (i == m_failedArfcn)
					break;
			}
		}
		// Freq shifted result
		if (m_checkPoint >= FreqShift) {
			buf = createSectPoint(FreqShift);
			m_y.dump(*buf,"data");
			o = o->append(buf);
		}
		if (m_checkPoint >= EngOut) {
		    buf = createSectPoint(EngOut);
		    m_out.dump(*buf,"data");
		    o = o->append(buf);
		}
	}
	buf = new String;
	*buf << "\r\n[result]\r\n";
	if (m_checkPoint == CheckPointOk)
		*buf << "status=PASSED\r\n";
	else {
		*buf << "status=FAILED\r\n";
		*buf << "cause=" << m_failedPoint << "\r\n";
	}
	*buf << s_extraResultParams;
	o = o->append(buf);
	o = o->append(new String("\r\n"));
	String buffer;
	buffer.append(list);
	printf("%s",buffer.c_str());
}

void BuildTx::resetArfcns(unsigned int n)
{
	if (m_arfcns)
		delete[] m_arfcns;
	m_arfcns = 0;
	m_arfcnCount = n;
	if (!m_arfcnCount)
		return;
	m_arfcns = new TestARFCN[m_arfcnCount];
	for (unsigned int i = 0; i < m_arfcnCount; i++)
		m_arfcns[i].setArfcn(i);
}

static void sigHandler(int sig)
{
	switch (sig) {
		case SIGINT:
		case SIGTERM:
			s_code = 0;
		break;
	}
}

extern "C" int main(int argc, const char** argv, const char** envp)
{
	::signal(SIGINT,sigHandler);
	::signal(SIGTERM,sigHandler);
	Debugger::enableOutput(true,true);
	TelEngine::debugLevel(DebugAll);
	s_cfg = "testtx.conf";
	s_cfg.load();
	NamedList* general = s_cfg.createSection("general");
	s_arfcns = (unsigned int)general->getIntValue("arfcns",4,1,4);
	s_oversampling = (unsigned int)general->getIntValue("oversampling",8,1,8);
	s_cmpSrc = lookup(general->getValue("compare","none"),s_cmpName);
	NamedList* out = s_cfg.createSection("output");
	s_debug = out->getBoolValue("debug");
	s_dumpHex = out->getBoolValue("dump_hex");
	s_dumpStr = out->getBoolValue("dump_str",true);
	s_confFileStyle = out->getBoolValue("conf_file_style",true);
	if (s_confFileStyle)
		s_linePrefix = "\\\r\n  ";
	else
		s_linePrefix = "\r\n  ";
	s_lineLen = (unsigned int)out->getIntValue("line_length",120,0,1000);
	if (s_lineLen) {
		if (s_lineLen < 40)
			s_lineLen = 40;
		else if (s_lineLen > 1000)
			s_lineLen = 1000;
		s_lineLen -= s_lineDec;
	}
	s_signalProcessing.initialize(s_oversampling,s_arfcns);
	s_code = 0;
	BuildTx tx;
	if (!tx.test())
		s_code = 2;
	return s_code;
}

/* vi: set ts=8 sw=4 sts=4 noet: */

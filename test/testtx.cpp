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

typedef SigProcVector<uint8_t,true> UInt8Vector;

static const String s_arfcnIn0('0',GSM_BURST_LENGTH);
static const String s_arfcnIn1('1',GSM_BURST_LENGTH);
static const float s_gsmSymbolRate = (13 * 1000000) / 48;

static int s_code = 0;
static Configuration s_cfg;
static NamedList* s_general = 0;
static unsigned int s_arfcns = 0;
static unsigned int s_oversampling = 0;
// Output
static unsigned int s_lineLen = 120;
static unsigned int s_lineDec = 2;
static bool s_dumpStr = true;
static bool s_dumpHex = true;
static bool s_confFileStyle = true;
static String s_linePrefix;

class TestTxDbg : public DebugEnabler
{
public:
    inline TestTxDbg(const char* name)
	{ debugName(name); }
    inline bool invalidConfParam(const char* p, const char* v, const char* sect)
	{ return invalidConfParam(p,v,sect,this); }
    static inline bool invalidConfParam(const char* p, const char* v, const char* sect,
	DebugEnabler* dbg) {
	    Debug(dbg,DebugConf,"Invalid %s='%s' in '%s' section",p,v,sect);
	    return false;
	}
};

class TestARFCN : public String
{
public:
    inline TestARFCN()
	: m_filler(false), m_inPower(0), m_arfcn(0xffffffff)
	{}
    inline unsigned int arfcn() const
	{ return m_arfcn; }
    inline void reset(unsigned int arfcn) {
	    assign("arfcn") += arfcn;
	    m_arfcn = arfcn;
	    m_inPower = 0;
	    m_inBits.resize(GSM_BURST_LENGTH);
	}
    bool m_filler;
    float m_inPower;
    UInt8Vector m_inBits;
    FloatVector m_v;
    ComplexVector m_omega;
    ComplexVector m_x;
    ComplexVector m_s;
private:
    unsigned int m_arfcn;
};

class BuildTx : public TestTxDbg
{
public:
    enum CheckPoint {
	Input,
	LaurentPA,
	LaurentFS,
	ARFCNv,
	ARFCNomega,
	ARFCNx,
	ARFCNs,
	FreqShift,
	CheckPointOk
    };
    inline BuildTx()
	: TestTxDbg("BUILD"),
	m_checkPoint(0), m_failedArfcn(0xffffff),
	m_arfcnCount(0), m_arfcns(0)
	{ init(); }
    inline ~BuildTx()
	{ resetArfcns(); }
    bool init();
    bool test();
    bool checkPoint(unsigned int point, TestARFCN* arfcn = 0);
    static const TokenDict s_pointLabel[];
protected:
    void saveResult();
    void resetArfcns(unsigned int n = 0);
    unsigned int m_checkPoint;
    unsigned int m_failedArfcn;
    unsigned int m_arfcnCount;
    String m_failedPoint;
    TestARFCN* m_arfcns;
    FloatVector m_laurentPA;
    ComplexVector m_laurentFS;
    ComplexVector m_y;
};

const TokenDict BuildTx::s_pointLabel[] =
{
    {"Input", Input},
    {"Laurent pulse approximation", LaurentPA},
    {"Laurent frequency shifting", LaurentFS},
    {"v", ARFCNv},
    {"omega", ARFCNomega},
    {"modulated", ARFCNx},
    {"frequency_shifting", ARFCNs},
    {"Frequency shifted", FreqShift},
    {0,0},
};

static inline void dumpParams(String& dest, const NamedList& list)
{
    for (const ObjList* o = list.paramList()->skipNull(); o; o = o->skipNext()) {
	const NamedString* ns = static_cast<const NamedString*>(o->get());
	dest << ns->name() << "=" << *ns << "\r\n";
    }
}

static String& appendUInt8(String& dest, const uint8_t& val, const char* sep)
{
    char s[80];
    sprintf(s,"%u",val);
    return dest.append(s,sep);
}

static String& appendFloat(String& dest, const float& val, const char* sep)
{
    char s[80];
    sprintf(s,"%g",val);
    return dest.append(s,sep);
}

static String& appendComplex(String& dest, const Complex& val, const char* sep)
{
    char s[170];
    sprintf(s,"%g%+gi",val.real(),val.imag());
    return dest.append(s,sep);
}

static inline bool setDump(bool*& dumpStr, bool*& dumpHex)
{
    if (!dumpStr)
	dumpStr = &s_dumpStr;
    if (!dumpHex)
	dumpHex = &s_dumpHex;
    return *dumpStr || *dumpHex;
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

static String& dumpVector(String& dest, FloatVector& v,
    const String& pName = String::empty(),
    bool* dumpStr = 0, bool* dumpHex = 0)
{
    if (!setDump(dumpStr,dumpHex))
	return dest;
    if (pName)
	dest << pName << "_length=" << v.length() << "\r\n";
    unsigned int first = 0;
    if (*dumpStr) {
	first = addPName(dest,pName,"_str=");
	if (!s_lineLen)
	    v.dump(dest,appendFloat) += "\r\n";
	else
	    v.appendSplit(dest,s_lineLen,appendFloat,first,s_linePrefix);
    }
    if (!*dumpHex)
	return dest;
    first = addPName(dest,pName,"_hex=");
    return v.appendSplitHex(dest,s_lineLen,first,s_linePrefix);
}

static String& dumpVector(String& dest, ComplexVector& v,
    const String& pName = String::empty(),
    bool* dumpStr = 0, bool* dumpHex = 0)
{
    if (!setDump(dumpStr,dumpHex))
	return dest;
    if (pName)
	dest << pName << "_length=" << v.length() << "\r\n";
    unsigned int first = 0;
    if (*dumpStr) {
	first = addPName(dest,pName,"_str=");
	if (!s_lineLen)
	    v.dump(dest,appendComplex) += "\r\n";
	else
	    v.appendSplit(dest,s_lineLen,appendComplex,first,s_linePrefix);
    }
    if (!*dumpHex)
	return dest;
    first = addPName(dest,pName,"_hex=");
    return v.appendSplitHex(dest,s_lineLen,first,s_linePrefix);
}

static String& dumpVector(String& dest, UInt8Vector& v,
    const String& pName = String::empty(),
    bool* dumpStr = 0, bool* dumpHex = 0, bool addLen = true,
    const char* sep = ",")
{
    if (!setDump(dumpStr,dumpHex))
	return dest;
    if (addLen && pName)
	dest << pName << "_length=" << v.length() << "\r\n";
    unsigned int first = 0;
    if (*dumpStr) {
	first = addPName(dest,pName,"_str=");
	if (!s_lineLen)
	    v.dump(dest,appendUInt8,sep) += "\r\n";
	else
	    v.appendSplit(dest,s_lineLen,appendUInt8,first,s_linePrefix,"\r\n",sep);
    }
    if (!*dumpHex)
	return dest;
    first = addPName(dest,pName,"_hex=");
    return v.appendSplitHex(dest,s_lineLen,first,s_linePrefix);
}

static inline bool readBits(UInt8Vector& dest, const String& src)
{
    unsigned int n = dest.length();
    if (n > src.length())
	return false;
    const char* s = src.c_str();
    uint8_t* d = dest.data();
    for (; n; n--) {
	if (*s == '1')
	    *d++ = 1;
	else if (*s == '0')
	    *d++ = 0;
	else
	    return false;
    }
    return true;
}


//
// BuildTx
//
bool BuildTx::init()
{
    NamedList& params = *s_general;
    resetArfcns(s_arfcns);
    for (unsigned int i = 0; i < m_arfcnCount; i++) {
	const String& name = m_arfcns[i];
	String s = params[name];
	if (s.isBoolean())
	    s = s.toBoolean() ? s_arfcnIn1: s_arfcnIn0;
	else if (!s)
	    s = s_arfcnIn1;
	else if (s == "filler") {
	    m_arfcns[i].m_filler = true;
	    continue;
	}
	if (!readBits(m_arfcns[i].m_inBits,s)) {
	    resetArfcns();
	    return invalidConfParam(name,s,params);
	}
	m_arfcns[i].m_inPower = 0.01;
    }
    return true;
}

// Build input data (TX)
bool BuildTx::test()
{
    if (!m_arfcns)
	return false;
#define CHECKPOINT(point,arfcn) \
    m_checkPoint = point; \
    if (!checkPoint(point,arfcn)) \
	break
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
	// Generate Laurent pulse approximation sampled at Fs, length: Lp = 4 * K
	unsigned int Lp = 4 * (unsigned int)K;
	m_laurentPA.resize(Lp);
	float* hp = m_laurentPA.data();
	for (unsigned int n = 0; n < m_laurentPA.length(); n++) {
	    float f = ((float)n - (Lp / 2)) / K;
	    float f2 = f * f;
	    float g = 1.138 * f2 - 0.527 * f2 * f2;
	    hp[n] = (1 / K) * 0.96 * ::expf(g);
	}
	CHECKPOINT(LaurentPA,0);
	// Generate Laurent frequency shift vector
	// s[n] = e ^ ((-j * n * PI) / (2 * K))
	m_laurentFS.resize(Ls);
	Complex* s = m_laurentFS.data();
	for (unsigned int n = 0; n < m_laurentFS.length(); n++) {
	    float real = ::expf(0);
	    float imag = (-(n * PI) / (2 * K));
	    s[n].real(real * ::cosf(imag));
	    s[n].imag(real * ::sinf(imag));
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
		// hp: Laurent pulse approximation sampled at Fs, length: Lp = 4 * K
		// s: complex-valued frequency shift sequence
		uint8_t* b = a.m_inBits.data();
		// Calculate v: v[n] = 2 * b[n] * floor(n / K) - 1
		a.m_v.assign(Ls);
		float* v = a.m_v.data();
		for (unsigned int n = 0; n < a.m_v.length(); n++) {
		    unsigned int idx = ::floor(n / K);
		    if (idx >= a.m_inBits.length())
			break;
		    v[n] = 2 * b[idx] - 1;
		}
		CHECKPOINT(ARFCNv,&a);
		// Calculate omega: omega[n] = v[n] * s[n]
		a.m_omega.resize(a.m_v.length());
		Complex* omega = a.m_omega.data();
		for (unsigned int n = 0; n < a.m_omega.length(); n++)
		    omega[n] = s[n] * v[n];
		CHECKPOINT(ARFCNomega,&a);
		// Modulate, build omega padded with Lp/2 elements at each end
		// x[n] = SUM(i=0..Lp)(omega[n + i] * hp[Lp - 1 - i])
		ComplexVector newOmega(a.m_omega.length() + Lp);
		newOmega.copy(a.m_omega.data(),a.m_omega.length(),Lp / 2);
		omega = newOmega.data();
		a.m_x.assign(a.m_omega.length());
		Complex* x = a.m_x.data();
		for (unsigned int n = 0; n < a.m_x.length(); n++)
		    for (unsigned int i = 0; i < Lp; i++)
			x[n] += omega[n + i] * hp[Lp - 1 - i];
	    }
	    else
		a.m_x.assign(Fs);
	    CHECKPOINT(ARFCNx,&a);
	    // Frequency shifting vector
	    // fk = (4 * k - 6)(100 * 10^3)
	    // omegaK = (2 * PI * fk) / Fs
	    // s[n] = e ^ (-j * n * omegaK)
	    a.m_s.resize(a.m_x.length());
	    float real = ::expf(0);
	    for (unsigned int n = 0; n < a.m_s.length(); n++) {
		float fk = (4 * a.arfcn() - 6) * (100 * 1000);
		float omegaK = (2 * PI * fk) / Fs;
		float imag = -n * omegaK;
		a.m_s[n].real(real * ::cosf(imag));
		a.m_s[n].imag(real * ::sinf(imag));
	    }
	    CHECKPOINT(ARFCNs,&a);
	}
	if (m_failedArfcn < m_arfcnCount)
	    break;
	// Frequency shifting
	// y[n] = SUM(i=0..k)(xi[n] * si[n])
	unsigned int k = m_arfcnCount;
	m_y.assign(Ls);
	for (unsigned int n = 0; n < m_y.length(); n++) {
	    for (unsigned int i = 0; i < k; i++) {
		Complex* x = m_arfcns[i].m_x.data();
		Complex* s = m_arfcns[i].m_s.data();
		m_y[n] += x[n] * s[n];
	    }
	}
	CHECKPOINT(FreqShift,0);
	m_checkPoint = CheckPointOk;
	break;
    }
    saveResult();
    return m_checkPoint == CheckPointOk;
}

bool BuildTx::checkPoint(unsigned int point, TestARFCN* arfcn)
{
    FloatVector* fVect = 0;
    ComplexVector* cVect = 0;
    String sectName;
    String dataName = "data";
    switch (point) {
#define CASE_POINT_VECT(point,cv,fv) \
    case point: sectName = lookup(point,s_pointLabel); fVect = fv; cVect = cv; break
#define CASE_POINT_VECT_A(point,cv,fv) \
    case point: \
	if (arfcn) { \
	    sectName = *arfcn; \
	    dataName = lookup(point,s_pointLabel); \
	    cVect = cv; \
	    fVect = fv; \
	    break; \
	} \
	return false
	case Input:
	    return true;
	CASE_POINT_VECT(LaurentPA,0,&m_laurentPA);
	CASE_POINT_VECT(LaurentFS,&m_laurentFS,0);
	CASE_POINT_VECT_A(ARFCNv,0,&arfcn->m_v);
	CASE_POINT_VECT_A(ARFCNomega,&arfcn->m_omega,0);
	CASE_POINT_VECT_A(ARFCNx,&arfcn->m_x,0);
	CASE_POINT_VECT_A(ARFCNs,&arfcn->m_s,0);
	CASE_POINT_VECT(FreqShift,&m_y,0);
	default:
	    return true;
    }
    // Compare
    String tmp;
    if (cVect)
	tmp.hexify(cVect->data(),cVect->size());
    else if (fVect)
	tmp.hexify(fVect->data(),fVect->size());
    if (!tmp)
	return true;
    NamedList* sect = s_cfg.getSection(sectName);
    const String& str = sect ? (*sect)[dataName + "_hex"] : String::empty();
    if (!str || tmp == str)
	return true;
    m_failedPoint.clear();
    m_failedPoint << "'" << lookup(point,s_pointLabel) << "'";
    if (arfcn) {
	m_failedArfcn = arfcn->arfcn();
	m_failedPoint << " ARFCN " << m_failedArfcn;
    }
    printf("Failed comparison %s\r\n-----\r\ncurrent=%u %s\r\ncompare=%u %s\r\n-----\r\n",
	m_failedPoint.safe(),tmp.length(),tmp.c_str(),str.length(),str.c_str());
    return false;
}

static inline void addSectPoint(String& dest, int point)
{
    dest << "\r\n[" << lookup(point,BuildTx::s_pointLabel) << "]\r\n";
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
	appendFloat(*buf,m_arfcns[i].m_inPower,0) += "\r\n";
	bool str = true;
	bool hex = false;
	dumpVector(*buf,m_arfcns[i].m_inBits,name + "_bits",&str,&hex,false,0);
    }
    o = o->append(buf);
    // Laurent approximation
    if (m_checkPoint >= LaurentPA) {
	buf = createSectPoint(LaurentPA);
	dumpVector(*buf,m_laurentPA,"data");
	o = o->append(buf);
    }
    // Laurent freq shifting
    if (m_checkPoint >= LaurentFS) {
	buf = createSectPoint(LaurentFS);
	dumpVector(*buf,m_laurentFS,"data");
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
		    dumpVector(*buf,a.m_v,lookup(ARFCNv,s_pointLabel));
		if (i < m_failedArfcn || m_checkPoint >= ARFCNomega)
		    dumpVector(*buf,a.m_omega,lookup(ARFCNomega,s_pointLabel));
		if (i < m_failedArfcn || m_checkPoint >= ARFCNx)
		    dumpVector(*buf,a.m_x,lookup(ARFCNx,s_pointLabel));
	    }
	    if (i < m_failedArfcn || m_checkPoint >= ARFCNs)
		dumpVector(*buf,a.m_s,lookup(ARFCNs,s_pointLabel));
	    o = o->append(buf);
	    if (i == m_failedArfcn)
		break;
	}
    }
    // Freq shifted result
    if (m_checkPoint >= FreqShift) {
	buf = createSectPoint(FreqShift);
	dumpVector(*buf,m_y,"data");
	o = o->append(buf);
    }
    buf = new String;
    *buf << "\r\n[result]\r\n";
    if (m_checkPoint == CheckPointOk)
	*buf << "status=PASSED\r\n";
    else {
	*buf << "status=FAILED\r\n";
	*buf << "cause=" << m_failedPoint << "\r\n";
    }
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
	m_arfcns[i].reset(i);
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
    // Load config
    TelEngine::debugLevel(DebugAll);
    s_cfg = "./test/testtx.conf";
    s_cfg.load();
    s_general = s_cfg.createSection("general");
    s_arfcns = (unsigned int)s_general->getIntValue("arfcns",4,1,4);
    s_oversampling = (unsigned int)s_general->getIntValue("oversampling",8,1,8);
    NamedList* out = s_cfg.createSection("output");
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
    s_code = 0;
    BuildTx tx;
    if (!tx.test())
	s_code = 2;
    return s_code;
}

/* vi: set ts=8 sw=4 sts=4 noet: */

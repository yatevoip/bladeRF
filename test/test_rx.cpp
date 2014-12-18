/**
 * test_rx.cpp
 * This file is part of the Yate-BTS Project http://www.yatebts.com
 *
 * Test Part for Transceiver RX side
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

// Compile: g++ ../sigproc.cpp test_rx.cpp  -I/usr/local/include/yate -lyate -g -o test_rx.out
// Run: ./test_rx.out

#include "../sigproc.h"
#include <yatengine.h>
#include <stdio.h>

using namespace TelEngine;

class DataComparator : public String
{
public:
    inline DataComparator(const char* name)
	: String(name)
	{}
    void initialize(const NamedList& params);
    void dump(const ComplexArray& array);
    void dump(const FloatVector& array);
    void loadCompare(String& data);
    String m_fileName;
    String m_prefix;
    String m_postfix;
    bool m_output;
    int m_printLen;
    bool m_append;
    String m_format;
    ComplexArray m_compare;
    FloatVector m_fcompare;
    bool m_isFloat;
    bool m_printStats;
    int m_comparePadding;
};

class QmfBlock
{
public:
    QmfBlock(unsigned int index, unsigned int len, bool final, Configuration& cfg);
    unsigned int m_index;
    ComplexArray m_freqencyShift;
    ComplexArray m_lowData;
    ComplexArray m_highData;
    bool m_final;
    DataComparator* m_fs;
    DataComparator* m_x;
    DataComparator* m_xp;
    DataComparator* m_w;
};


static Configuration s_cfg;

#define s_hqLength 23
static int s_n0 = (s_hqLength -1) / 2;
FloatVector s_hq;
QmfBlock* s_qmfs[15];
bool s_normalBurst = true;
unsigned int s_timeslot = 0;
float s_powerMin = 1.0;
#define FLOATABS 20e-6

static float s_qmfFrequencyShiftParameter[] = {
    PI / 2,              // 0
    0.749149017394489,   // 1
    -0.749149017394489,  // 2
    -0.821647309400407,  // 3
    -2.31994534418939,   // 4
    -0.821647309400407,  // 5
    -2.31994534418939,   // 6
    -PI/4,               // 7
    1,                   // 8
    -PI/4,               // 9
    1,                   // 10
    -PI/4,               // 11
    1,                   // 12
    -PI/4,               // 13
    1,                   // 14
};

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

static int s_accessTraining[] = {0,1,0,0,1,0,1,1,0,1,1,1,1,1,1,1,1,0,0,1,1,0,0,1,1,0,1,0,1,0,1,0,0,0,1,1,1,1,0,0,0};

bool floatEqual(float a, float b)
{
    
    float x = a > b ? a - b : b - a;
    return x < FLOATABS;
}

static void generateHQ()
{
    s_hq.resize(s_hqLength);
    for (int i = 0;i < s_hqLength;i++) {
	float fn = (PI * (i - s_n0)) / 2;
	float wn = 0.54 - 0.46 * ::cosf(((2 * PI) * i) / (s_hqLength - 1));
	if (fn)
	    s_hq[i] = wn * ::sin(fn) / fn;
	else
	    s_hq[i] = wn;
    }
}

DataComparator* getDC(const String prefix, const String name,NamedList& cfg)
{
    DataComparator* dc = new DataComparator(prefix + ": " + name.substr(0,name.length() - 1));
    if (TelEngine::null(name)) {
	dc->initialize(cfg);
	return dc;
    }
    NamedList params("");
    params.copySubParams(cfg,name);
    if (!params.count()) {
	TelEngine::destruct(dc);
	return 0;
    }
    dc->initialize(params);
    return dc;
}

QmfBlock::QmfBlock(unsigned int index, unsigned int len, bool final, Configuration& cfg)
    : m_index(index), m_final(final), m_fs(0), m_x(0), m_xp(0), m_w(0)
{
    m_freqencyShift.assign(len);
    for (unsigned int i = 0;i < len;i++) {
	Complex::exp(m_freqencyShift[i],0,i*s_qmfFrequencyShiftParameter[index]);
    }
    String name = "qmf(";
    name << index;
    name << ")";
    NamedList* section = cfg.getSection(name);
    if (!section)
	return;
    m_fs = getDC(name,"fs.",*section);
    m_x = getDC(name,"x.",*section);
    m_xp = getDC(name,"xp.",*section);
    m_w = getDC(name,"w.",*section);
  
    if (m_fs)
	m_fs->dump(m_freqencyShift);
}

void DataComparator::initialize(const NamedList& params)
{
    m_fileName = params.getValue(YSTRING("file_name"),0);
    m_append = params.getBoolValue(YSTRING("file_append"),true);
    m_prefix = params.getValue(YSTRING("prefix"),0);
    m_postfix = params.getValue(YSTRING("postfix"),0);
    m_format = params.getValue(YSTRING("dump_format"),"%+g%+gi");
    m_output = params.getValue(YSTRING("output"));
    m_printLen = params.getIntValue("print-len",-1);
    m_printStats = params.getValue(YSTRING("print_stats"));
    m_comparePadding = params.getIntValue("compare_padding",0);
    String compare = params.getValue(YSTRING("compare_file"));
    m_isFloat = params.getBoolValue(YSTRING("is_float"),false);
    while (!TelEngine::null(compare)) {
	File f;
	if (!f.openPath(compare)) {
	    Debug(c_str(),DebugWarn,"Failed to open file %s",compare.c_str());
	    break;
	}
	DataBlock buf(0,(unsigned int)f.length());
	f.readData(buf.data(),buf.length());
	
	String s((char*)buf.data(),buf.length());
	
	String* line = 0;
	
	ObjList* lines = s.split('\n',false);
	if (!lines)
	    break;
	
	for (ObjList* o = lines->skipNull();o;) {
	    String* l = static_cast<String*>(o->get());
	    if (l->endsWith(" \\")) {
		l->assign(l->substr(0,l->length() - 2));
		if (line) {
		    line->append(*l);
		    o->remove();
		    o = o->skipNull();
		} else {
		    line = l;
		    o = o->skipNext();
		}
		continue;
	    }
	    if (!line) {
		o = o->skipNext();
		continue;
	    }
	    line->append(*l);
	    o->remove();
	    line = 0;
	    o = o->skipNull();
	}
	String fprefix = params.getValue("file_prefix");
	String* comp = 0;
	for (ObjList* o = lines->skipNull();o;o = o->skipNext()) {
	    String* l = static_cast<String*>(o->get());
	    if (TelEngine::null(fprefix) || l->startsWith(fprefix)) {
		comp = l;
		break;
	    }
	}
	if (!comp) {
	    Debug(c_str(),DebugWarn,"Unable to load data from file for prefix %s",fprefix.c_str());
	    TelEngine::destruct(lines);
	    return;
	}
	if (!TelEngine::null(fprefix))
	    comp->startSkip(fprefix,false);
	loadCompare(*comp);
	TelEngine::destruct(lines);
	return;
    }
    compare = params.getValue(YSTRING("compare"));
    if (!TelEngine::null(compare))
	loadCompare(compare);
}

void DataComparator::dump(const ComplexArray& array)
{
    // Dump the data
    String dump = m_prefix;
    char tmpd[50];
    int len = m_printLen;
    if (len < 0 || len > (int)array.length())
	len = array.length();
    for (unsigned int i = 0;i < len;i++) {
	int outl = ::sprintf(tmpd,m_format,array[i].real(),array[i].imag());
	dump.append(tmpd,outl);
	if (i != len - 1)
	    dump << ",";
    }
    dump << m_postfix;
    if (!TelEngine::null(m_fileName)) {
	File f;
	if (!f.openPath(m_fileName, true, false, true, m_append))
	    Debug(c_str(),DebugWarn,"Failed to open file %s",m_fileName.c_str());
	f.writeData(dump.c_str(),dump.length());
    }
    if (m_output) {
	::printf("\n%s: len %d\n%s\n",c_str(),array.length(),dump.c_str());
    }
    if (m_compare.length() > 0) {
	unsigned int end = m_compare.length();
	if (end > array.length())
	    end = array.length();
	if (m_compare.length() != array.length())
	    Debug(c_str(),DebugNote,"Different lengths! Compare %d data %d",m_compare.length(), array.length());
	unsigned int i = 0;
	for (;i < end;i ++) {
	    if (floatEqual(m_compare[i].real() ,array[i + m_comparePadding].real()) && 
		    floatEqual(m_compare[i].imag(), array[i + m_comparePadding].imag()))
		continue;
	    Debug(c_str(),DebugWarn,"Values differ expected index %d (%g, %g)  data (%g,%g) at index %d real diff %f imag diff %f",
		  i + m_comparePadding ,array[i + m_comparePadding].real(),array[i + m_comparePadding].imag(),
		  m_compare[i].real(),m_compare[i].imag(),i,m_compare[i].real() - array[i + m_comparePadding].real(),
		  m_compare[i].imag()- array[i + m_comparePadding].imag());
	    break;
	}
	if (i == end)
	    Debug(c_str(),DebugNote,"Data Mached!");
    }
    if (m_printStats) {
	float min = 0,max = 0, poz = 0,neg = 0;
	int tpoz = 0,tneg = 0;
	for (unsigned int i = 0;i < len;i++) {
	    if (min > array[i].real())
		min = array[i].real();
	    else if (max < array[i].real())
		max = array[i].real();
	    if (array[i].real() < 0) {
		neg += array[i].real();
		tneg++;
		continue;
	    }
	    poz += array[i].real();
	    tpoz++;
	}
	Debug(c_str(),DebugNote,"Min %f max %f avg poz: %f avg neg: %f",
	    min,max,poz/tpoz,neg/tneg);
    }
}

void DataComparator::dump(const FloatVector& array)
{
    // Dump the data
    String dump = m_prefix;
    char tmpd[50];
    int len = m_printLen;
    if (len < 0 || len > (int)array.length())
	len = array.length();
    for (unsigned int i = 0;i < len;i++) {
	int outl = ::sprintf(tmpd,m_format,array[i]);
	dump.append(tmpd,outl);
	if (i != len - 1)
	    dump << ",";
    }
    dump << m_postfix;
    if (!TelEngine::null(m_fileName)) {
	File f;
	if (!f.openPath(m_fileName, true, false, true, m_append))
	    Debug(c_str(),DebugWarn,"Failed to open file %s",m_fileName.c_str());
	f.writeData(dump.c_str(),dump.length());
    }
    if (m_output) {
	::printf("\n%s: len %d\n%s\n",c_str(),array.length(),dump.c_str());
    }

    if (m_fcompare.length() > 0) {
	unsigned int end = m_fcompare.length();
	if (end > array.length())
	    end = array.length();
	if (m_fcompare.length() != array.length())
	    Debug(c_str(),DebugNote,"Different lengths! Compare %d data %d",m_fcompare.length(), array.length());
	unsigned int i = 0;
	for (;i < end;i ++) {
	    if (floatEqual(m_fcompare[i],array[i + m_comparePadding]))
		continue;
	    Debug(c_str(),DebugWarn,"Values differ expected index %d (%g)  data (%g) at index %d diff %f",
		  i + m_comparePadding ,array[i + m_comparePadding], m_fcompare[i],i,m_fcompare[i] - array[i + m_comparePadding]);
	    break;
	}
	if (i == end)
	    Debug(c_str(),DebugNote,"Data Mached!");
    }
    if (m_printStats) {
	float min = 0,max = 0, poz = 0,neg = 0;
	int tpoz = 0,tneg = 0;
	for (unsigned int i = 0;i < len;i++) {
	    if (min > array[i])
		min = array[i];
	    else if (max < array[i])
		max = array[i];
	    if (array[i] < 0) {
		neg += array[i];
		tneg++;
		continue;
	    }
	    poz += array[i];
	    tpoz++;
	}
	Debug(c_str(),DebugNote,"Min %f max %f avg poz: %f avg neg: %f",
	      min,max,poz/tpoz,neg/tneg);
    }
}

void DataComparator::loadCompare(String& data)
{
    String tmp = data.trimSpaces();
    ObjList* split = tmp.split(',',false);
    if (!split) {
	// TODO check Hex
	return;
    }
    int index = 0;
    if (m_isFloat) {
	m_fcompare.resize(split->count());
	for (ObjList * o = split->skipNull();o;o = o->skipNext(),index ++) {
	    String* s = static_cast<String*>(o->get());
	    m_fcompare[index] = (float)s->toDouble();
	}
	TelEngine::destruct(split);
	return;
    }
    m_compare.resize(split->count());
    for (ObjList * o = split->skipNull();o;o = o->skipNext(),index ++) {
	String* s = static_cast<String*>(o->get());
	float r = 0,i = 0;
	int got = ::sscanf(s->c_str(),"%g %g",&r,&i);
	if (got == 2) {
	    m_compare[index].set(r,i);
	    continue;
	}
	Debug(c_str(),DebugNote,"Invalid compare value '%s'",s->c_str());
	m_compare.resize(0);
	break;
    }
    TelEngine::destruct(split);
}

static void qmf(QmfBlock* q, ComplexArray& dataIn)
{
    Debug(DebugAll,"qmf for index %d",q->m_index);
    if (q->m_x)
	q->m_x->dump(dataIn);
    // Build xp. dataIn should be already padded
    // xp contains the dataIn multiplied by frequency shift array
    unsigned int inLen = dataIn.length();

    // NOTE TODO TEST Uncertenty 
    // The repeat period for the final leafs of the qmf tree if 4 before frequency 
    // shift. If we apply the frequency shift array of the final leaf the reapet 
    // period will be 8;
    // How is correct?
    if (q->m_final) {
	q->m_lowData.assign(inLen);
	for (unsigned int i = 0; i < q->m_lowData.length();i++)
	    q->m_lowData[i] = dataIn[i];
	return;
    }
    

    if (inLen != q->m_freqencyShift.length()) {
	Debug("qmf",DebugWarn,"Missmatching lengths!! in %d freq %d",
		inLen, q->m_freqencyShift.length());
	return;
    }
    ComplexArray xp(inLen);
    String tmpu;
    for (unsigned int i = 0;i < inLen;i++)
	xp[i] = dataIn[i] * q->m_freqencyShift[i];

    if (q->m_xp)
	q->m_xp->dump(xp);
    
    if (q->m_final) {
	q->m_lowData.assign(inLen);
	for (unsigned int i = 0; i < q->m_lowData.length();i++)
	    q->m_lowData[i] = xp[i];
	return;
    }
    
    // Calculate w
    ComplexArray paddedXP(inLen + s_hqLength);
    for (unsigned int i = 0;i < inLen;i++)
	paddedXP[i + s_n0] = xp[i];
    ComplexArray w(inLen);
    for (unsigned int i = 0;i < w.length();i++) {
	for (int j = 0;j < s_n0;j++)
	    w[i] += paddedXP[i + 2 * j] * s_hq[2 * j];
    }

    if (q->m_w)
	q->m_w->dump(w);

    // Calculate low
    q->m_lowData.assign(w.length() / 2);
    for (unsigned int i = 0;i < w.length() - s_n0;i += 2) {
	q->m_lowData[i / 2] = xp[i] + w[i];
    }
    qmf(s_qmfs[2* q->m_index + 1],q->m_lowData);
    
    // Calculate High
    q->m_highData.assign(w.length() / 2);
    for (unsigned int i = 0;i < w.length() - s_n0;i+= 2) {
	Complex::diff(q->m_highData[i / 2], xp[i], w[i]);
    }
    qmf(s_qmfs[2 * q->m_index + 2],q->m_highData);
}

float getPower(ComplexArray& dataIn)
{
    unsigned int center = 148 / 2;
    float power = 0;
    for (unsigned int i = center - 2 ;i < center + 2; i++)
	power += dataIn[i].mulConj();
    return 0.2 * power;
}

void checkHQ(NamedList& params)
{
    String compare = params.getValue("compare");
    while (!TelEngine::null(compare)) {
	ObjList* split = compare.split(',',false);
	if (!split)
	    break;
	if (split->count() != s_hqLength) {
	    Debug("hq",DebugNote,"Different Lengths compare %d s_hq %d",split->count(), s_hqLength);
	    break;
	}
	unsigned int i = 0;
	for (ObjList* o = split->skipNull();o;o = o->skipNext()) {
	    String* tmp = static_cast<String*>(o->get());
	    float x = 0;
	    if (::sscanf(tmp->c_str(),"%f",&x) != 1) {
		Debug("hq",DebugNote,"Unable to decode float from %s",tmp->c_str());
		break;
	    }
	    if (!floatEqual(x, s_hq[i])) {
		Debug("hq",DebugNote,"Different values at index %d recv %f generated %f",i,x,s_hq[i]);
		break;
	    }
	    i++;
	}
	if (i == s_hqLength)
	    Debug("hq",DebugNote,"HQ matches!");
	TelEngine::destruct(split);
	break;
    }
    if (!params.getBoolValue("output",false))
	return;
    String dump = params.getValue("prefix");
    char tx[50];
    for (unsigned int i = 0;i < s_hqLength;i++) {
	int len = ::sprintf(tx,"%f",s_hq[i]);
	dump.append(tx,len);
	if (i != s_hqLength - 1)
	    dump << ",";
    }
    dump << params.getValue("postfix");
    printf("\nHQ: len %d\n%s\n",s_hqLength,dump.c_str());
}

ComplexArray* convolve(ComplexArray& in, FloatVector& h)
{
    ComplexArray padded(in.length() + h.length());
    unsigned int n0 = (h.length() - 1) / 2;
    for (unsigned int i = 0;i < in.length();i++)
	padded[i + n0] = in[i];

    ComplexArray* out = new ComplexArray(in.length());
    for (unsigned int i = 0;i < in.length();i++) {
	for (unsigned int j = 0;j < h.length();j++) {
	    Complex c;
	    Complex::multiplyF(c,padded[i + j],h[h.length() - (j + 1)]);
	    
	    (*out)[i] += c;
//	    Debug(DebugAll, " i %d j %d padded [%d] ( %g %g) h[%d] %f  out[i] = (%g % g)",
//		  i,j,i+j,padded[i + j].real(),padded[i + j].imag(),h.length() - (j + 1),h[h.length() - (j + 1)],(*out)[i].real(),(*out)[i].imag());
	}
    }
    return out;
}

ComplexArray* convolveConj(ComplexArray& in, ComplexArray& h)
{
    ComplexArray padded(in.length() + h.length());
    unsigned int n0 = (h.length() - 1) / 2;
    for (unsigned int i = 0;i < in.length();i++)
	padded[i + n0] = in[i];
    
    ComplexArray conj(h.length());
    for (unsigned int i = 0;i < h.length();i++) {
	h[i].imag(-h[i].imag());
    }
    
    ComplexArray* out = new ComplexArray(in.length());
    for (unsigned int i = 0;i < in.length();i++) {
	for (unsigned int j = 0;j < h.length();j++) {
	    Complex t = h[h.length() - (j + 1)];
	    (*out)[i] += padded[i + j] * t;
	}
    }
    return out;
}


void demodulate(QmfBlock* inData, Configuration& cfg,int arfcnIndex)
{
    // Power detection
    float power = getPower(inData->m_lowData);
    Debug(DebugAll,"Power Level %f",power);
    /*if (power < s_powerMin) {
	Debug(DebugNote,"Ignoring Data! Power to low");
	return;
    }*/
    
    // TODO implement me!!
    // Channel Estimation
    ComplexArray* he = 0;
    if (s_normalBurst) {
	ComplexArray x(26);
	int startIndex = inData->m_lowData.length() / 2 - 13;
	int endIndex = startIndex + 26;
	int index = 0;
	for (unsigned int i = startIndex; i < endIndex;i++,index++)
	    x[index] = inData->m_lowData[i];
	
	FloatVector sm(16);
	int* s = s_normalTraining[s_timeslot] + 21;
	for (unsigned int i = 0; i < 16;i++,s++)
	    sm[i] = *s ? 1 : -1;
	he = convolve(x,sm);
    } else {
	ComplexArray x(41);
	int index = 0;
	for (unsigned int i = 8; i < 48;i++,index++)
	    x[index] = inData->m_lowData[i];
	
	FloatVector sm(16);
	int* s = s_accessTraining;
	for (unsigned int i = 0; i < 16;i++,s++)
	    sm[i] = *s ? 1 : -1;
	
	he = convolve(x,sm);
    }
    String arfcnPrefix(arfcnIndex);
    arfcnPrefix << ".";
    NamedList* dhe = cfg.getSection("demod-he");
    while (dhe) {
	DataComparator* cHe = getDC("demod-he",arfcnPrefix,*dhe);
	if (cHe)
	    cHe->dump(*he);
	TelEngine::destruct(cHe);
	break;
    }
    
    ComplexArray* w = convolveConj(inData->m_lowData,*he);
    TelEngine::destruct(he);

    FloatVector u(w->length());
    for (unsigned int i = 0; i < u.length();i++)
	u[i] = (*w)[i].real();
    TelEngine::destruct(w);

    NamedList* dv = cfg.getSection("demod-u");
    while (dv) {
	DataComparator* cv = getDC("demod-u",arfcnPrefix,*dv);
	if (cv)
	    cv->dump(u);
	TelEngine::destruct(cv);
	break;
    }

    float psum = 0;
    for (unsigned int i = 72; i <= 76;i++)
	psum += u[i] * u[i];
    float p = 0.2 * psum;
    
    float sqrtP = ::sqrt(p);
    
    
    FloatVector v(u.length());
    for (unsigned int i = 0; i < v.length();i++)
	v[i] = u[i] / sqrtP;
   
    
    
    FloatVector wf(u.length());
    for (unsigned int i = 0; i < u.length();i++)
	wf[i] = (u[i] + 1) / 2;
    
    NamedList* dout = cfg.getSection("demod-out");
    while (dout) {
	DataComparator* cout = getDC("demod-out",arfcnPrefix,*dout);
	if (cout)
	    cout->dump(wf);
	TelEngine::destruct(cout);
	break;
    }
    
    FloatVector est(wf.length());
    for (unsigned int i = 0; i < wf.length();i++) {
	est[i] = (int)::round(wf[i] * 255);
    }
    
    NamedList* estimate = cfg.getSection("estimate");
    while (estimate) {
	DataComparator* es = getDC("estimate",arfcnPrefix,*estimate);
	if (es)
	    es->dump(est);
	TelEngine::destruct(es);
	break;
    }
}

extern "C" int main(int argc, const char** argv, const char** envp)
{
    Debugger::enableOutput(true,true);
    TelEngine::debugLevel(10);
    Output("RX_Test is starting");
    s_cfg = "test_rx.conf";
    if (!s_cfg.load()) {
	Debug(DebugFail,"Failed to load config!");
	return 0;
    }
    generateHQ();
    NamedList* hq = s_cfg.getSection("hq");
    if (hq) {
	DataComparator* cw = getDC("hq","",*hq);
	cw->dump(s_hq);
	TelEngine::destruct(cw);
    }

    unsigned int qmfLen = 1250;
    for (unsigned int i = 0;i < 15; i++) {
	if (i == 1 || i == 3 || i == 7)
	    qmfLen /= 2;
	s_qmfs[i] = new QmfBlock(i,qmfLen,i >= 7,s_cfg);
    }
    
    // Load Input Data
    NamedList* dataIn = s_cfg.getSection(YSTRING("data_in"));
    if (!dataIn) {
	Debug("main",DebugNote,"No Input Data!! Unable to continue Testing");
	return 0;
    }
    s_normalBurst = dataIn->getBoolValue(YSTRING("normalBurst"),s_normalBurst);
    s_timeslot = dataIn->getBoolValue(YSTRING("timeslot"),s_timeslot);
    
    DataComparator* indc =  new DataComparator("In_data");
    indc->initialize(*dataIn);
    if (indc->m_compare.length() <= 0) {
	Debug("main",DebugNote,"Unable to load input data!");
	return 0;
    }
    if (indc->m_compare.length() < 1250) {
	Debug("main",DebugNote,"Short input data! Read %d expected 1250",indc->m_compare.length());
	return 0;
    }
    
    ComplexArray ind(indc->m_compare.length());
    // Pad the data with zeros
    for (unsigned int i = 0;i < indc->m_compare.length();i++)
	ind[i].set(indc->m_compare[i].real() / 100000,indc->m_compare[i].imag() / 100000);
    
    TelEngine::destruct(indc);
    // Run the data trough qmf filter
    qmf(s_qmfs[0],ind);
    
    
    demodulate(s_qmfs[7],s_cfg,0);
    
    
    Output("RX_Test terminated");
    return 0;
}
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
	void dump(const ComplexVector& array);
	void dump(const FloatVector& array);
	void loadCompare(String& data);

	String m_fileName;
	String m_prefix;
	String m_postfix;
	bool m_output;
	int m_printLen;
	bool m_append;
	String m_format;
	ComplexVector m_compare;
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
	ComplexVector m_freqencyShift;
	ComplexVector m_lowData;
	ComplexVector m_highData;
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
unsigned int s_tsc = 2;
unsigned int s_delay = 0;
unsigned int s_imageDelay = 0;
unsigned int s_imageGain = 0;
unsigned int s_imagePhase = 0;
unsigned int s_image2Delay = 0;
unsigned int s_image2Gain = 0;
unsigned int s_image2Phase = 0;
unsigned int s_maxPropDelay = 63;
float s_powerMin = 1.0;
#define FLOATABS 20e-4

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

void DataComparator::dump(const ComplexVector& array)
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
		float max = 0, rms = 0;
		Complex mean(0,0);
		for (unsigned int i = 0;i < len;i++) {
			float power = array[i].mulConj();
			if (power>max)
				max = power;
			mean += array[i];
			rms += power;
		}
		rms = ::sqrtf(rms/len);
		mean = mean * (1.0F/len);
		max = ::sqrtf(max);
		Debug(c_str(),DebugNote,"mean %f%+fi max abs %f rms %f ",
			mean.real(), mean.imag(), max, rms);
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
		int len = array.length();
		float mean = 0, max = 0, rms = 0;
		for (unsigned int i = 0;i < len;i++) {
			float power = array[i] * array[i];
			if (power>max)
				max = power;
			mean += array[i];
			rms += power;
		}
		rms = ::sqrtf(rms/len);
		mean = mean * (1.0F/len);
		max = ::sqrtf(max);
		Debug(c_str(),DebugNote,"mean %f max abs %f rms %f ",
			mean, max, rms);
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

static void qmf(QmfBlock* q, ComplexVector& dataIn)
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
	ComplexVector xp(inLen);
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
	ComplexVector paddedXP(inLen + s_hqLength);
	for (unsigned int i = 0;i < inLen;i++)
	    paddedXP[i + s_n0] = xp[i];


	ComplexVector w(inLen);
	for (unsigned int i = 0;i < w.length();i+=2) {
		for (int j = 0;j < s_hqLength;j+=2) {
			w[i] += paddedXP[i + j] * s_hq[j];
		}
	}

	if (q->m_w)
		q->m_w->dump(w);

	// Calculate low
	q->m_lowData.assign(w.length() / 2);
	for (unsigned int i = 0;i < w.length();i += 2) {
		q->m_lowData[i / 2] = xp[i] + w[i];
		q->m_lowData[i/2] = q->m_lowData[i/2] * 0.5F;
	}
	qmf(s_qmfs[2* q->m_index + 1],q->m_lowData);
	
	// Calculate High
	q->m_highData.assign(w.length() / 2);
	for (unsigned int i = 0;i < w.length();i+= 2) {
		Complex::diff(q->m_highData[i / 2], xp[i], w[i]);
		q->m_highData[i/2] = q->m_highData[i/2] * 0.5F;
	}
	qmf(s_qmfs[2 * q->m_index + 2],q->m_highData);
}

float getPower(ComplexVector& dataIn)
{
	unsigned int center = 148 / 2;
	float power = 0;
	for (unsigned int i = center - 2 ;i < center + 2; i++)
		power += dataIn[i].mulConj();
	return 0.2 * power;
}

float getNoisePower(ComplexVector& dataIn)
{
	// Use the gaurd interval for noise measurement.
	unsigned int center = 148 / 2;
	float power = 0;
	for (unsigned int i = 0; i<4; i++)
		power += dataIn[dataIn.length()-i-1].mulConj();
	return 0.25 * power;
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

ComplexVector* correlate(ComplexVector& in, FloatVector& h)
{
	ComplexVector padded(in.length() + h.length());
	unsigned int n0 = (h.length() - 1) / 2;
	for (unsigned int i = 0;i < in.length();i++)
		padded[i + n0] = in[i];

	ComplexVector* out = new ComplexVector(in.length());
	for (unsigned int i = 0;i < in.length();i++) {
		for (unsigned int j = 0;j < h.length();j++) {
			Complex c;
			Complex::multiplyF(c,padded[i + j],h[j]);
			
			(*out)[i] += c;
		}
	}
	return out;
}

ComplexVector* correlate(ComplexVector& in, ComplexVector& h)
{
	ComplexVector padded(in.length() + h.length());
	unsigned int n0 = (h.length() - 1) / 2;
	for (unsigned int i = 0;i < in.length();i++)
		padded[i + n0] = in[i];
	
	ComplexVector conj(h.length());
	for (unsigned int i = 0;i < h.length();i++) {
		conj[i].real(h[i].real());
		conj[i].imag(-h[i].imag());
	}
	
	ComplexVector* out = new ComplexVector(in.length());
	for (unsigned int i = 0;i < in.length();i++) {
		for (unsigned int j = 0;j < h.length();j++) {
			(*out)[i] += padded[i + j] * conj[j];
		}
	}
	return out;
}


void demodcheck(ComplexVector& in, String name, Configuration& cfg,int arfcnIndex)
{
	String arfcnPrefix(arfcnIndex);
	arfcnPrefix << ".";
	NamedList* dhe = cfg.getSection(name);
	while (dhe) {
		DataComparator* cHe = getDC(name,arfcnPrefix,*dhe);
		if (cHe)
			cHe->dump(in);
		TelEngine::destruct(cHe);
		break;
	}   
}

void demodcheck(FloatVector& in, String name, Configuration& cfg,int arfcnIndex)
{
	String arfcnPrefix(arfcnIndex);
	arfcnPrefix << ".";
	NamedList* dhe = cfg.getSection(name);
	while (dhe) {
		DataComparator* cHe = getDC(name,arfcnPrefix,*dhe);
		if (cHe)
			cHe->dump(in);
		TelEngine::destruct(cHe);
		break;
	}   
}


void demodulate(QmfBlock* inData, Configuration& cfg,int arfcnIndex)
{
	//
	// NOTE: inData should be scaled to +1..-1, where +/-1 represents the
	// limits of the A/D converter in the receiver.
	// For a 12-bit receiver like the BaldeRF, this means that the raw
	// samples need to be divided by 2048 before feeding into the QMF tree.
	// It also means that the overall gain of the QMF filter needs to be 1.0.
	//
	// Power detection
	float power = getPower(inData->m_lowData);
	float noise = getNoisePower(inData->m_lowData);
	float SNR = power/noise;
	Debug(DebugAll,"Power Level %f, noise %f, SNR %f",power, noise, SNR);
	//
	// TODO: The average noise level for this ARFCN needs to be tracked somewhere.
	// It should be tracked on an ARFCN-by-ARFCN basis.
	//
	// TODO: Abort based on low SNR.
	// The thresh should be configurable in dB. A good default is 2 dB.
	// if (SNR < thr) {
	//   Debug(level,"Discard burst due to low SNR %f", SNR);
	//   ...
	// }
	//
	// This should be reported in the burst message in dB wrt full scale.
	// If the input data are properly scaled, this will always be negative.
	float dB = 10*log10f(power);
	//
	// TODO: At this point, we should check max against a threshold and log an event if there is an overdrive of the recevier.
	// The alert threshold should be configurable in dB wrt full scale. -2 dB is a good default.
	// Continue processing, though.
	// if  (dB > thr) 
	//   Debug(someLevel, "receiver clipping on C%dT%d, %f dB", carrierIndex, slotNumber, dB);


	// DAB - This implements a -pi/2 frequency shift.
	ComplexVector xf(inData->m_lowData.length());
	static const Complex s[] = {Complex(1,0), Complex(0,-1), Complex(-1,0), Complex(0,1)};
	for (unsigned int i = 0; i < inData->m_lowData.length(); i++) {
		xf[i] = s[i%4] * inData->m_lowData[i];
	}
	
	// Channel Estimation
	ComplexVector* he = 0;
	int center;
	if (s_normalBurst) {
		// isolate the recevied midamble
		ComplexVector x(26);
		int startIndex = inData->m_lowData.length() / 2 - 13;
		int endIndex = startIndex + 26;
		int index = 0;
		for (unsigned int i = startIndex; i < endIndex;i++,index++)
			x[index] = xf[i];
		
		// Correlating with the middle 16 symbols of the training seqeunce
		// gives a clean autocorrealtion in the middle 11 samples of the result.
		// The scaling factor gives a unit-gain filter.
		FloatVector sm(16);
		int* s = s_normalTraining[s_tsc] + 5;
		static const float scv = 1.0F / 16.0F;
		for (unsigned int i = 0; i < 16;i++,s++)
			sm[i] = *s ? scv : -scv;

		// Note: Only the middle 11 samples actually need to be calculated in he.
		
		demodcheck(x, "correlate-in1", cfg,arfcnIndex);
		demodcheck(sm, "correlate-in2", cfg,arfcnIndex);
		
		he = correlate(x,sm);
		center = (he->length()/2)-1;

	} else {
		// isolate the synchronization sequence, plus up to 63 samples of delay
		unsigned xlen = 41+s_maxPropDelay;
		ComplexVector x(xlen);
		int index = 0;
		for (unsigned int i = 8; i < xlen + 8;i++,index++)
			x[index] = xf[i];
		
		// The actual extended training sequence is 41 symbols.
		FloatVector sm(41);
		int* s = s_accessTraining;
		static const float scv = 1.0F / 41.0F;
		for (unsigned int i = 0; i < 41;i++,s++)
			sm[i] = *s ? scv : -scv;
		
		// The entire correlation is needed for initial delay estimation.
		// Note that we zero-delay point ("center") is NOT centered in he.
		// NOTE: This correlation accounts for the majority of CPU usage in an idle BTS.
		demodcheck(x, "correlate-in1", cfg,arfcnIndex);
		demodcheck(sm, "correlate-in2", cfg,arfcnIndex);
		he = correlate(x,sm);
		// TODO: *WHY* is 24 the value that works here?
		center = 24;
	}
	demodcheck(*he, "correlate-out", cfg,arfcnIndex);

	// At this point, he contains a channel estimate with the zero-delay point at "center".

	Debug(DebugAll,"Chan center at %d", center);

	// Find the peak power in the channel estimate.
	float max = 0;
	float rms = 0;
	int maxIndex;
	for (unsigned i=center-5; i<he->length(); i++) {
		float pwr = (*he)[i].mulConj();
		rms += pwr;
		if (pwr<max) continue;
		max = pwr;
		maxIndex = i;
	}
	rms/= (he->length() - (center-5));
	float peakOverMean = max/rms;
	Debug(DebugAll,"Chan max %f @ %d, rms %f, peak/mean %f", max, maxIndex, rms, peakOverMean);
	//
	// TODO: If peak/mean is too low, we can abort processing.
	// The threshold should be configurable in dB; a good default is probably 3 dB.
	// if (peakOverMean < thr) {
	//   Debug(someLevel,"Discarding burst on C%dT%d due to low peak/mean of %f",carrierIndex,slotNumber,peak/mean);
	//   ...
	// }
	//
	// TOA error.  Negative for early, positive for late.
	int toaError = maxIndex - center;
	Debug(DebugAll,"TOA error %d", toaError);

	// indexing relationships
	// maxIndex - actual t0 of the received signal channel estimate
	// center - expected t0 of the received signal channel estimate
	// toaError - the offset of the channel estaimte from its expected position, negative for early

	// TOA error should not exceed +/-2 for a normal burst.
	// It indicates a failure of the closed loop timing control.
	// Continue processing, but log the event.
	if (s_normalBurst) {
	  if (toaError>2 || toaError<-2)  {
	  //  Debug(someLevel,"Excessive TOA error %d, C%dT%d, peak/mean %f", toaError, carrierIndex, slotNumber, peakOverMean);
	  	Debug(DebugAll,"Excessive TOA error %d, peak/mean %f", toaError, peakOverMean);
	  }
	}

	// Shift the received signal to compensate for TOA error.
	// This shifts the signal so that the max power image is aligned to the expected position.
	// The alignment error is +/- 1/2 symbol period;
	// fractional alignment is corrected by correlation with channel estaimte.
	ComplexVector xf2(xf.length());
	for (int i=0; i<xf.length(); i++) {
		int j = i + toaError;
		if (j<0) 
		    continue;
		if (j>=xf.length()) 
		    continue;
		xf2[i]=xf[j];
	}

	// Trim and center the channel estimate.
	// This shifts the channel estimate to put the max power peak in the center, +/- 1/2 symbol period.
	// The alignment error of heTrim is exactly the opposiate of the alignment error of xf2.
	ComplexVector heTrim(11);
	for (unsigned i=0; i<11; i++) {
		int j = maxIndex - 5 + i;
		if (j<0 || j>= (he->length())) continue;
		heTrim[i] = (*he)[j];
	}

	for (unsigned i=0; i<11; i++) {
		heTrim[i] = (*he)[maxIndex-5+i];
		int j = maxIndex - 5 + i;
		if (j<0 || j>= (he->length())) 
		    continue;
		heTrim[i] = (*he)[j];
	}
	
	
	String arfcnPrefix(arfcnIndex);
	arfcnPrefix << ".";
	NamedList* dhe = cfg.getSection("demod-he");
	while (dhe) {
		DataComparator* cHe = getDC("demod-he",arfcnPrefix,*dhe);
		if (cHe)
			cHe->dump(heTrim);
		TelEngine::destruct(cHe);
		break;
	}

	// Calculate delay spread, an important channel statistic.
	float sumTSq = 0;
	float sumP = 0;
	for(int i=5; i<11; i++) {
		float p = heTrim[i].mulConj();
		sumTSq += (i-5)*(i-5)*p;
		sumP += p;
	}
	float delaySpread = sqrt(sumTSq/sumP);
	Debug(DebugAll,"Delay spread %f",delaySpread);

	// The scope below is the "equalizer".
	// It is the part of the system most likely to change in the future.
	// It will most defintely change when we go to EDGE.
	// and should be isolated through an API.
	// The inputs are xf2 and he.
	// The output is u.
	FloatVector u(xf2.length());
	{
		// This correlation against the channel model serves as our equalizer.
		ComplexVector* w = correlate(xf2,heTrim);
		delete he;
	
		// The imaginary part is just noise at this point.
		// All of the decodable signal power is in the real part.
		for (unsigned i = 0; i < u.length();i++) {
			u[i] = (*w)[i].real();
		}
		delete w;
	}

	NamedList* dv = cfg.getSection("demod-u");
	while (dv) {
		DataComparator* cv = getDC("demod-u",arfcnPrefix,*dv);
		if (cv)
			cv->dump(u);
		TelEngine::destruct(cv);
		break;
	}

	// RMS power estimate from middle 5 symbols
	float psum = 0;
	for (unsigned int i = 72; i <= 76;i++)
		psum += u[i] * u[i];
	float p = 0.2 * psum;
	float sqrtP = ::sqrtf(p);

	
	// Normalize amplitude to +1..-1.
	// and trim off the gaurd periods
	// NOTE: The 4-sample offset might change depending on how data
	// are frame-aligned when they come into the receiver.
	FloatVector v(148);
	for (unsigned int i = 0; i < 148;i++)
		v[i] = u[i+4] / sqrtP;
   
	// Map the -1..+1 range to 0..1 with hard limits.
	FloatVector wf(v.length());
	for (unsigned int i = 0; i < v.length();i++)
		if (v[i]>1.0F) wf[i]=1.0F;
		else if (v[i]<-1.0F) wf[i]=0.0F;
		else wf[i] = (v[i] + 1.0F) * 0.5;
	
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
	s_tsc = dataIn->getIntValue(YSTRING("tsc"),s_tsc);
	// modeled delay of the input
	s_delay = dataIn->getIntValue(YSTRING("delay"),s_delay);
	// modeled multipath images
	s_imageGain = dataIn->getIntValue(YSTRING("image-gain"),s_imageGain);
	s_imageDelay = dataIn->getIntValue(YSTRING("image-delay"),s_imageDelay);
	s_imagePhase = dataIn->getIntValue(YSTRING("image-phase"),s_imagePhase);
	s_image2Gain = dataIn->getIntValue(YSTRING("image2-gain"),s_image2Gain);
	s_image2Delay = dataIn->getIntValue(YSTRING("image2-delay"),s_image2Delay);
	s_image2Phase = dataIn->getIntValue(YSTRING("image2-phase"),s_image2Phase);
	// maximum RACH delay to try to track
	s_maxPropDelay = dataIn->getIntValue(YSTRING("max-prop-delay"),s_maxPropDelay);
	
	//DataComparator* indc =  new DataComparator("In_data");
	DataComparator* indc =  new DataComparator("in_data");
	indc->initialize(*dataIn);
	if (indc->m_compare.length() <= 0) {
		Debug("main",DebugNote,"Unable to load input data!");
		return 0;
	}
	if (indc->m_compare.length() < 1250) {
		Debug("main",DebugNote,"Short input data! Read %d expected 1250",indc->m_compare.length());
		return 0;
	}
	
	ComplexVector ind(indc->m_compare.length());
	// Pad the data with zeros
	// For testing, shift the input data by a given number of samples.
	// And and add an optional multipath image at a level of imageGain/10.
	float imagePhi = s_imagePhase * PI / 180.0F;
	float image2Phi = s_image2Phase * PI / 180.0F;

	Complex cImageGain;
	cImageGain.real(s_imageGain * 0.1F * ::cosf(imagePhi));
	cImageGain.imag(s_imageGain * 0.1F * ::sinf(imagePhi));

	Complex cImage2Gain;
	cImage2Gain.real(s_image2Gain * 0.1F * ::cosf(image2Phi));
	cImage2Gain.imag(s_image2Gain * 0.1F * ::sinf(image2Phi));

	for (int i = 0;i < indc->m_compare.length();i++) {
/*		if ((i-(int)s_delay)>=0)
			ind[i] = indc->m_compare[i-(int)s_delay];

		if ((i-(int)s_delay-(int)s_imageDelay)>=0)
			ind[i] += indc->m_compare[i-(int)s_delay-(int)s_imageDelay] * cImageGain;

		if ((i-(int)s_delay-(int)s_image2Delay)>=0)
			ind[i] += indc->m_compare[i-(int)s_delay-(int)s_image2Delay] * cImage2Gain;*/
	    //ind[i].set(indc->m_compare[i].real() / 2048,indc->m_compare[i].imag() / 2048);
	    ind[i].set(indc->m_compare[i].real(),indc->m_compare[i].imag());
	}

	Debug(DebugAll,"Image delay %d samples, gain %f+%f",s_imageDelay,cImageGain.real(),cImageGain.imag());
	Debug(DebugAll,"Image2 delay %d samples, gain %f+%f",s_image2Delay,cImage2Gain.real(),cImage2Gain.imag());
	TelEngine::destruct(indc);
	// Run the data trough qmf filter
	qmf(s_qmfs[0],ind);
	
	if (!s_qmfs[7]->m_lowData.length()) {
	    Debug(DebugWarn,"failed to apply QMF filter for Arfcn 0");
	    return 0;
	}
	
	demodulate(s_qmfs[7],s_cfg,0);
	
	
	Output("RX_Test terminated");
	return 0;
}

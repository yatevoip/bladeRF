/**
 * test.cpp
 * This file is part of the Yate-BTS Project http://www.yatebts.com
 *
 * Transceiver test implemetation
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

#include "test.h"

using namespace TelEngine;


//
// TestIface
//
TestIface::TestIface()
    : m_powerOn(false)
{
}

void printComplexArray(const ComplexArray& array, unsigned int debugLevel, unsigned int numbersPerLine)
{
    unsigned int startPrinting = 1;
    String tmpDump;
    for (unsigned int i = 0; i < array.length();i++,startPrinting++) {
	array[i].dump(tmpDump);
	if (startPrinting != numbersPerLine) 
	    continue;
	Debug(debugLevel,"%s",tmpDump.c_str());
	startPrinting = 0;
	tmpDump = "";
    }
    Debug(debugLevel,"%s",tmpDump.c_str());
}

void TestIface::testTx(NamedList& params)
{
    String burstData = params.getValue(YSTRING("data"));
    if (TelEngine::null(burstData))
	return;
    XDebug(DebugTest,"Starting txTest with oversample 8 and initial data : %s",burstData.c_str());
    // First obtain the frequencyShift array

    ComplexArray frequencyShift;
    SignalProcessing::fillFrequencyShift(&frequencyShift,8);

    XDebug(DebugAll,"Build frequency shift array for oversample 8 has %d elements :",frequencyShift.length());
    printComplexArray(frequencyShift,DebugAll,8);

    char inData[148];
    for (int i = 0;i < 148;i++) {
	inData[i] = burstData[i] - '0';
    }

    ComplexArray modulated(BITS_PER_TIMESLOT * 8);
    unsigned int parseSize = 148 * 8;
    if (parseSize > modulated.length())
	parseSize = modulated.length();
    for (unsigned int i = 0; i < parseSize; i++)
	Complex::multiplyF(modulated[i],frequencyShift[i],inData[i / 8] ? 1.0 : -1.0);

    XDebug(DebugInfo,"Modulated frequency shift array has %d elements :",modulated.length());
    printComplexArray(modulated,DebugInfo,8);
    
    
    ComplexArray laurentPulseAproximation;
    SignalProcessing::fillLaurentPulseAproximation(&laurentPulseAproximation,8);

    XDebug(DebugNote,"laurentPulseAproximation array has %d elements :",laurentPulseAproximation.length());
    printComplexArray(laurentPulseAproximation,DebugNote,8);
    

    ComplexArray* convolution = SignalProcessing::getConvolution(modulated,laurentPulseAproximation);

    XDebug(DebugMild,"Convolution array has %d elements :",convolution->length());
    printComplexArray(*convolution,DebugMild,8);
    
}

unsigned int s_energizer = 32767; // 2^15 - 1

short trimComplex(float cv)
{
    if (cv >= 1)
	return s_energizer;
    if (cv <= -1)
	return -s_energizer;
    return (short)(cv * s_energizer);
}

bool TestIface::sendData(const ComplexArray& data)
{
    unsigned int outLength = data.length() * 2;
    short* out = new short[outLength];
    for (unsigned int i = 0;i < outLength;i += 2) {
	out[i] = trimComplex(data[i / 2].real());
	out[i + 1] = trimComplex(data[i / 2].imag());
    }

    String tmp;
    for (unsigned int i = 0; i < outLength && i < 100;i+=2)
	tmp << '(' << out[i] << ',' << out[i+1] << "),";
    delete[] out;
    //Debug(this,DebugAll,"Send Data %s",tmp.c_str());
    return true;
}

// Set Rx/Tx frequency
int TestIface::tune(bool rx, double freq, double adj)
{
    Debug(this,DebugAll,"%sTEST set %s freq=%g adj=%g [%p]",
	prefix(),(rx ? "Rx" : "Tx"),freq,adj,this);
    return 0;
}

// Read data from radio
bool TestIface::readRadio(RadioIOData& data, unsigned int* n)
{
    int16_t* b = data.buffer();
    unsigned int samples = n ? *n : 0;
    if (!samples || samples > data.free())
	samples = data.free();
    if (!(b && samples))
	return true;
#if 1
    // Simulate read
    if (samples > 5000)
	samples = 5000;
    unsigned int p = data.pos();
    data.advance(samples);
    for (; samples; samples--) {
	*b++ = p++;
	*b++ = p++;
    }
    Thread::idle();
#endif
    return true;
}

// Write data to radio
bool TestIface::writeRadio(RadioIOData& data)
{
    int16_t* b = data.data();
    unsigned int samples = data.pos();
    if (!(b && samples))
	return true;
    // NOTE: Simulate write
    data.consumed(samples);
    return true;
}

// Start the radio device
bool TestIface::radioPowerOn()
{
    if (!m_powerOn)
	Debug(this,DebugNote,"%sTEST power ON [%p]",prefix(),this);
    m_powerOn = true;
    return true;
}

// Stop the radio device
void TestIface::radioPowerOff()
{
    if (m_powerOn)
	Debug(this,DebugNote,"%sTEST power OFF [%p]",prefix(),this);
    m_powerOn = false;
}

// Set Rx/Tx power gain
int TestIface::radioSetGain(bool rx, double dB, double* setVal)
{
    Debug(this,DebugAll,"%sTEST set %s gain %g [%p]",
	prefix(),(rx ? "Rx" : "Tx"),dB,this);
    if (setVal)
	*setVal = dB;
    return 0;
}

/* vi: set ts=8 sw=4 sts=4 noet: */

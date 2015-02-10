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

void TestIface::testTx(NamedList& params)
{
    Debug(DebugStub,"TestIface::testTx() not implemented");
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

bool TestIface::sendData(const ComplexVector& data)
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

// Set frequency correction offset
bool TestIface::setFreqCorr(int val)
{
    Debug(this,DebugAll,"%sTEST set freq correction %d [%p]",
	prefix(),val,this);
    return true;
}

/* vi: set ts=8 sw=4 sts=4 noet: */

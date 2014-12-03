/**
 * gsmutil.cpp
 * This file is part of the Yate-BTS Project http://www.yatebts.com
 *
 * GSM Utility classes implementation
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

#include "gsmutil.h"
#include "math.h"

using namespace TelEngine;

// GSM Training Sequence Code for Normal Burst (See TS 100 908 - GSM 05.02 Section 5.2.3)
static const int8_t s_nbTscTable[8][GSM_NB_TSC_LEN] = {
    {0,0,1,0,0,1,0,1,1,1,0,0,0,0,1,0,0,0,1,0,0,1,0,1,1,1},
    {0,0,1,0,1,1,0,1,1,1,0,1,1,1,1,0,0,0,1,0,1,1,0,1,1,1},
    {0,1,0,0,0,0,1,1,1,0,1,1,1,0,1,0,0,1,0,0,0,0,1,1,1,0},
    {0,1,0,0,0,1,1,1,1,0,1,1,0,1,0,0,0,1,0,0,0,1,1,1,1,0},
    {0,0,0,1,1,0,1,0,1,1,1,0,0,1,0,0,0,0,0,1,1,0,1,0,1,1},
    {0,1,0,0,1,1,1,0,1,0,1,1,0,0,0,0,0,1,0,0,1,1,1,0,1,0},
    {1,0,1,0,0,1,1,1,1,1,0,1,1,0,0,0,1,0,1,0,0,1,1,1,1,1},
    {1,1,1,0,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,0,1,1,1,1,0,0}
};

// GSM Sync Bits for Access Burst (See TS 100 908 - GSM 05.02 Section 5.2.7)
static const int8_t s_abSyncTable[GSM_AB_SYNC_LEN] =
    {0,1,0,0,1,0,1,1,0,1,1,1,1,1,1,1,1,0,0,1,1,0,0,1,1,0,1,0,1,0,1,0,0,0,1,1,1,1,0,0,0};


//
// GSMUtils
//
const int8_t* GSMUtils::nbTscTable()
{
    return (const int8_t*)s_nbTscTable;
}

const int8_t* GSMUtils::abSyncTable()
{
    return s_abSyncTable;
}

GSMTxBurst GSMTxBurst::s_zeroBurst;

//
// GSMTxBurst
//
GSMTxBurst::~GSMTxBurst()
{
    TelEngine::destruct(m_transformed);
}

GSMTxBurst* GSMTxBurst::get(const DataBlock& data, unsigned int length)
{
#ifdef XDEBUG
    String tmp;
    tmp.hexify(data.data(),length);
    Debug(DebugAll,"GSMTxBurst::get() data in: %s",tmp.c_str());
#endif
    // We have:
    // data[0] Time slot number
    // data[1-4] Frame number
    // data[5] power level
    // The rest should be burst data of size GSM_BURST_LENGTH

    // The data size should be GSM_BURST_LENGTH + 1 (time slot number) + 4 (Frame number) + 1 (power level)
    if (length != GSM_BURST_LENGTH + 6) {
	Debug(DebugMild,"Received invalid data length %u for GSMTxBurst! Expected %u",
	    length,GSM_BURST_LENGTH + 6);
	return 0;
    }
    GSMTxBurst* burst = new GSMTxBurst();
    burst->m_time.assign(data.at(1) << 24 | data.at(2) << 16 | data.at(3) << 8 | data.at(4),data.at(0) & 0x7f);
    burst->m_isFillerBurst = (data.at(0) & 0x80) != 0;
    // The power is received in decibels.
    // power = amplitude * 2
    // amplitude = 10^(decibels/20)
    // power = 10 ^ (decibels / 10)
    // Maximum decibels level is 0 so the power level received will be <= 0
    // In conclusion  powerLevel = 10^(-receivedPowerLevel/10)
    burst->m_powerLevel = ::pow(10, -data.at(5) / 10); // This should be between 0 and 1! See how should be transformed!

    burst->assign(0,4);
    burst->append((void*)data.data(6),GSM_BURST_LENGTH);
    uint8_t a[4];
    a[0] = a[1] = a[2] = a[3] = 0;
    burst->append(a,4);
    return burst;
}

GSMTxBurst* GSMTxBurst::getZeroBurst()
{ 
    if (!GSMTxBurst::s_zeroBurst.m_transformed) {
	s_zeroBurst.m_isDummy = true;
    }
    return &s_zeroBurst;
}

const ComplexArray* GSMTxBurst::transform(const SignalProcessing& sigproc)
{
    if (m_transformed)
	return m_transformed;
    if (this == &s_zeroBurst) {
	m_transformed = new ComplexArray(sigproc.getModulatedLength());
	return m_transformed;
    }
    m_transformed = sigproc.modulate((unsigned char*)data(),length());
    if (!m_transformed)
	return 0;
    
    
/*    
    float amp = (1 - 0.01) / 16;
    for (unsigned int i = 0;i < 24;i++) {
	if (i < 8)
	    (*m_transformed)[i].set(0.01,0);
	else
	    Complex::multiplyF((*m_transformed)[i],(*m_transformed)[i],amp * (i + 1 - 8));
    }
    
    unsigned int startSample = 1208;
    for (unsigned int i = startSample;i < 1246;i++) {
	if (i < startSample + 16)
	    Complex::multiplyF((*m_transformed)[i],(*m_transformed)[i],amp * (startSample + 16 - i));
	else
	    Complex::multiplyF((*m_transformed)[i],(*m_transformed)[i],0.01);
    }
*/   
    // NOTE Start
    // This is a test variant for power ramp
    float amp = (1 - 0.01) / 4;
    for (unsigned int i = 0;i < 24;i++) {
	if (i < 20)
	    (*m_transformed)[i].set(0.01,0);
	else
	    Complex::multiplyF((*m_transformed)[i],(*m_transformed)[i],amp * (i + 1 - 20));
    }
    

    unsigned int startSample = 1208;
    for (unsigned int i = startSample;i < 1246;i++) {
	if (i < startSample + 4)
	    Complex::multiplyF((*m_transformed)[i],(*m_transformed)[i],amp * (startSample + 4 - i));
	else
	    (*m_transformed)[i].set(0.01,0);
    }
    // NOTE End
    
    const ComplexArray* sinusoid = sigproc.getSinusoid(m_arfcnNumber);
    if (!sinusoid || sinusoid->length() != m_transformed->length()) {
	TelEngine::destruct(m_transformed);
	return 0;
    }
    m_transformed->multiply(*sinusoid);
    // Apply the power level
    for (unsigned i = 0;i < m_transformed->length();i++)
	Complex::multiplyF((*m_transformed)[i],(*m_transformed)[i],m_powerLevel);
    return m_transformed;
}


//
// GSMRxBurst
//
// Setup a buffer from burst
unsigned int GSMRxBurst::buildEstimatesBuffer(int8_t* buf, unsigned int len)
{
    if (!buf || len < 8)
	return 0;
    // Buffer format:
    //   1 byte timeslot index
    //   4 bytes GSM frame number, big endian
    //   1 byte power level
    //   2 bytes correlator timing offset (timing advance error)
    //   Symbol estimates, 0 -> definite "0", 255 -> definite "1"
    *buf++ = m_time.tn();
    *buf++ = (int8_t)(m_time.fn() >> 24);
    *buf++ = (int8_t)(m_time.fn() >> 16);
    *buf++ = (int8_t)(m_time.fn() >> 8);
    *buf++ = (int8_t)m_time.fn();
    *buf++ = (int8_t)m_powerLevel;
    *buf++ = (int8_t)((int)m_timingError >> 8);
    *buf++ = (int8_t)m_timingError;
    len -= 8;
    unsigned int n = len ? m_bitEstimate.length() : 0;
    if (n > len)
	n = len;
    const float* c = m_bitEstimate.data();
    for (unsigned int i = 0; i < n; i++, c++)
	*buf++ = (int8_t)::round(*c * 255.0);
    return n + 8;
}

/* vi: set ts=8 sw=4 sts=4 noet: */

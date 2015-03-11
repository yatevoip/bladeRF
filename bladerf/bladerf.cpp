/**
 * bladerf.cpp
 * This file is part of the Yate-BTS Project http://www.yatebts.com
 *
 * bladeRF device interface implemetation
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

#include "bladerf.h"
#include <stdio.h>
#include <string.h>

using namespace TelEngine;

//#define BRF_DEBUG_RECV
//#define BRF_DEBUG_SEND
//#define BRF_PRINT_IO 0                 // Print I/O buffer timestamps 0: both, >0: RECV, <0 SEND

// Maximum values for Rx/Tx DC offset I and Q
#define BRF_RX_DC_OFFSET_MAX 63
#define BRF_TX_DC_OFFSET_MAX 63
// Shift values for Rx/Tx DC offset I and Q
#define BRF_RX_DC_OFFSET_SHIFT 5
#define BRF_TX_DC_OFFSET_SHIFT 4

// Calculate Rx DC offset correction
#define BRF_RX_DC_OFFSET_ERROR 10
#define BRF_RX_DC_OFFSET_COEF 1.5
#define BRF_RX_DC_OFFSET_AVG_DAMPING 1024
#define BRF_RX_DC_OFFSET_DEF (BRF_RX_DC_OFFSET_ERROR * BRF_RX_DC_OFFSET_AVG_DAMPING)
#define BRF_RX_DC_OFFSET(val) (((double)val * BRF_RX_DC_OFFSET_COEF + BRF_RX_DC_OFFSET_ERROR) * BRF_RX_DC_OFFSET_AVG_DAMPING)

#define BANDWIDTH 1500000

const TokenDict s_loopbackMode[] = {
    { "txlpf-rxvga2",       BLADERF_LB_BB_TXLPF_RXVGA2 },
    { "txvga1-rxvga2",      BLADERF_LB_BB_TXVGA1_RXVGA2 },
    { "txlpf-rxlpf",        BLADERF_LB_BB_TXLPF_RXLPF },
    { "txvga1-rxlpf",       BLADERF_LB_BB_TXVGA1_RXLPF },
    { "lna1",               BLADERF_LB_RF_LNA1 },
    { "lna2",               BLADERF_LB_RF_LNA2 },
    { "lna3",               BLADERF_LB_RF_LNA3 },
    { "none",               BLADERF_LB_NONE },
    { 0, 0 },
};

const TokenDict s_sampling[] = {
    { "unknown",            BLADERF_SAMPLING_UNKNOWN },
    { "internal",           BLADERF_SAMPLING_INTERNAL },
    { "external",           BLADERF_SAMPLING_EXTERNAL },
    { 0, 0 },
};

const TokenDict s_lnaGain[] = {
    { "unknown",            BLADERF_LNA_GAIN_UNKNOWN },
    { "bypass",             BLADERF_LNA_GAIN_BYPASS },
    { "mid",                BLADERF_LNA_GAIN_MID },
    { "max",                BLADERF_LNA_GAIN_MAX },
    { 0, 0 },
};

const TokenDict s_lpfMode[] = {
    { "normal",            BLADERF_LPF_NORMAL },
    { "bypassed",          BLADERF_LPF_BYPASSED },
    { "dissabled",         BLADERF_LPF_DISABLED },
    { 0, 0 },
};

const TokenDict s_correction[] = {
    { "i",                 BLADERF_CORR_LMS_DCOFF_I },
    { "q",                 BLADERF_CORR_LMS_DCOFF_Q },
    { "phase",             BLADERF_CORR_FPGA_PHASE },
    { "gain",              BLADERF_CORR_FPGA_GAIN },
    { 0, 0 },
};

const TokenDict s_dumpType[] = {
    { "raw",               BladeRFDump::Raw },
    { "samples",           BladeRFDump::Samples },
    { "complex",           BladeRFDump::Complex },
    { 0, 0 },
};

enum BrfFlags
{
    BrfRxOn =      0x00000001,
    BrfTxOn =      0x00000002,
};

static inline uint32_t swapBytes(uint32_t val)
{
#ifdef LITTLE_ENDIAN
    return val;
#else
    return ((val >> 24) & 0x000000ff) |
        ((val << 24) & 0xff000000) |
	((val >> 8) & 0x0000ff00) |
        ((val << 8) & 0x00ff0000);
#endif
}

static inline bool isIntervalInt(int val, int minVal, int maxVal)
{
    return minVal <= val && val <= maxVal;
}

static inline void clampInt(int& val, int minVal, int maxVal)
{
    if (val < minVal)
	val = minVal;
    else if (val > maxVal)
	val = maxVal;
}

static inline const char* rxtx(bool rx)
{
    return (rx ? "Rx" : "Tx");
}

static inline bladerf_module rxtxmod(bool rx)
{
    return rx ? BLADERF_MODULE_RX : BLADERF_MODULE_TX;
}

static inline bool thShouldExit(BrfIface* ifc)
{
    return Thread::check(false) ||
	(ifc->transceiver() && ifc->transceiver()->shouldStop());
}

static inline unsigned int getUInt(const NamedList& p, const String& param,
    unsigned int defVal = 1, unsigned int minVal = 1,
    unsigned int maxVal = (unsigned int)INT_MAX)
{
    return (unsigned int)p.getIntValue(param,defVal,minVal,maxVal);
}

// Utilities used to display device info
static inline void addVersion(String& dest, struct bladerf_version& ver,
    const char* prefix = 0)
{
    dest << prefix << ver.major << "." << ver.minor << "." << ver.patch <<
	" (" << ver.describe << ")";
}
static inline int addError(String& dest, int& level, int e, const char* prefix = 0)
{
    if (e < 0) {
	dest << (prefix ? prefix : "Error: ") << e << " " << ::bladerf_strerror(e);
	level = DebugNote;
    }
    return e;
}
static inline int brfAddVer(String& dest, int& level, int e, struct bladerf_version& ver)
{
    if (addError(dest,level,e) >= 0)
	addVersion(dest,ver);
    return e;
}

static inline int tuneCmdReturn(bool ret)
{
    return ret ? Transceiver::CmdEOk : Transceiver::CmdEFailure;
}

// Initialize data used to wait for interface Tx busy
// Clear the flag when destroyed
class BrfSerialize
{
public:
    inline BrfSerialize(BrfIface* ifc, Mutex& mtx, bool& flag)
	: m_ifc(ifc), m_mutex(mtx), m_flag(flag), m_set(false)
	{}
    inline ~BrfSerialize()
	{ drop(); }
    inline void drop() {
	    if (m_set)
		m_flag = false;
	    m_set = false;
	}
    inline bool wait() {
	    while (true) {
		Lock lck(m_mutex,Thread::idleUsec());
		if (thShouldExit(m_ifc))
		    return false;
		if (!m_flag && lck.locked()) {
		    m_set = m_flag = true;
		    break;
		}
	    }
	    return true;
	}
    BrfIface* m_ifc;
    Mutex& m_mutex;
    bool& m_flag;
    bool m_set;
};
#define BRF_TX_SERIALIZE_ON_RET(isSafe,retVal) \
    BrfSerialize txSerialize(this,m_txIO.m_mutex,m_txIO.m_busy); \
    if (!(isSafe || txSerialize.wait())) \
	return retVal
#define BRF_TX_SERIALIZE_ON_NONE(isSafe) \
    BrfSerialize txSerialize(this,m_txIO.m_mutex,m_txIO.m_busy); \
    if (!(isSafe || txSerialize.wait())) \
	return
#define BRF_RX_SERIALIZE_ON_RET(isSafe,retVal) \
    BrfSerialize rxSerialize(this,m_rxIO.m_mutex,m_rxIO.m_busy); \
    if (!(isSafe || rxSerialize.wait())) \
	return retVal
#define BRF_RX_SERIALIZE_ON_NONE(isSafe) \
    BrfSerialize rxSerialize(this,m_rxIO.m_mutex,m_rxIO.m_busy); \
    if (!(isSafe || rxSerialize.wait())) \
	return
#define BRF_RX_SERIALIZE_DROP rxSerialize.drop()

static inline void computeMinMax(int& minVal, int& maxVal, int val)
{
    if (minVal > val)
	minVal = val;
    if (maxVal < val)
	maxVal = val;
}

static inline void setBrfLibLog(const String& value)
{
    if (!value)
	return;
    int l = value.toInteger(BLADERF_LOG_LEVEL_WARNING);
    ::bladerf_log_set_verbosity((bladerf_log_level)l);
}

static inline void printIO(BrfIface* ifc, bool read, BrfIO& rx, BrfIO& tx, unsigned int nBufs)
{
#ifdef BRF_PRINT_IO
    if ((BRF_PRINT_IO) && read != ((BRF_PRINT_IO) > 0))
	return;
    BrfIO& io = (read ? rx : tx);
    const char* what = (read ? "RECV" : "SEND");
    for (unsigned int i = 0; i < nBufs; i++) {
	BrfTsHeader& hdr = io.header(i);
	uint64_t hi = swapBytes(hdr.timeHi);
	uint64_t lo = swapBytes(hdr.timeLo);
	uint64_t ts = (hi << 31) | (lo >> 1);
	String tmp;
	if (!read)
	    tmp << " future: " << ((int64_t)tx.m_timestamp - (int64_t)rx.m_timestamp);
	Output("%s ts=" FMT64U "%s",what,ts,tmp.safe());
    }
#endif
}

//
// BrfIO
//
void BrfIO::reset(bool superSpeed)
{
    clear();
    m_buffers = 0;
    m_current = 0;
    m_pos = 0;
    m_bufSamples = superSpeed ? BRF_SUPERSPEED_SAMPLES : BRF_HIGHSPEED_SAMPLES;
    m_useSamples = superSpeed ? BRF_SUPERSPEED_SAMPLES_USE : BRF_HIGHSPEED_SAMPLES_USE;
    if (!m_count)
	m_count = 1;
    if (superSpeed)
	m_super = new BrfSuperSpeedTsBuffer[m_count];
    else
	m_high = new BrfHighSpeedTsBuffer[m_count];
}

void BrfIO::clear()
{
    if (m_super) {
	delete[] m_super;
	m_super = 0;
    }
    if (m_high) {
	delete[] m_high;
	m_high = 0;
    }
}

// Test input data
static int16_t s_tenTen[BRF_SUPERSPEED_SAMPLES_USE * 2];
static int16_t s_zeroZero[BRF_SUPERSPEED_SAMPLES_USE * 2];

static void initTestData()
{
    if (s_tenTen[0] != s_tenTen[2])
	return;
    for (unsigned int i = 0;i < BRF_SUPERSPEED_SAMPLES_USE * 2;i += 4)
	s_tenTen[i] = (i % 8) ? BRF_ENERGYPEAK : -BRF_ENERGYPEAK;
}

DataDumper::DataDumper(BrfIface* iface, u_int64_t samples, unsigned int spt)
    : m_iface(iface), m_samples(samples), m_written(0), m_samplesPerTimeslot(spt)
{}

bool DataDumper::open(const String& fileName)
{
    if (!m_file.openPath(fileName,true,false,true)) {
	Debug(m_iface,DebugWarn,"Failed to open file %s error %s [%p] %p",fileName.c_str(),strerror(errno),&m_file,this);
	return false;
    }
    DDebug(m_iface,DebugNote,"File %s oppened![%p] %p",fileName.c_str(),&m_file,this);
    return true;
}

void DataDumper::write(void* buf,unsigned int len, u_int64_t timestamp)
{
    if (m_samples < m_written) {
	BrfIface* i  = m_iface;
	m_iface = 0;
	if (i)
	    i->resetDumper(this,true);
	return;
    }
    if (len % 2)
	Debug(m_iface,DebugMild,"Odd buf length! Input data not in samples?[%p] %p ",&m_file,this);

    String ts("\nTime Stamp: FN: ");
    ts << timestamp / (8 * m_samplesPerTimeslot);
    ts << " TN: ";
    ts << (timestamp % (8 * m_samplesPerTimeslot)) / m_samplesPerTimeslot;
    ts << "\n";
    m_file.writeData(ts.c_str(),ts.length());
    String tmp;
    tmp.hexify(buf,len,' ');
    int w = m_file.writeData(tmp.c_str(),tmp.length());
    if (w != (int)tmp.length())
	Debug(m_iface,DebugWarn,"DataDumper: data write requested %d written %d Left "FMT64" %s",tmp.length(),w,m_samples - m_written,w < 0 ? ::strerror(m_file.error()) : "");

    m_written += len / 4;
}

const char* s_dumpMutexName = "BrfIfaceDataDump";

//
// BrfIface
//
BrfIface::BrfIface()
    : m_flags(0),
    m_dev(0),
    m_superSpeed(false),
    m_rxResyncCandidate(0),
    m_rxShowDcInfo(0),
    m_rxDcAuto(true),
    m_rxDcOffsetMax(BRF_RX_DC_OFFSET_DEF),
    m_rxDcOffsetI(BRF_RX_DC_OFFSET_MAX + 1),
    m_rxDcOffsetQ(BRF_RX_DC_OFFSET_MAX + 1),
    m_rxDcAvgI(0),
    m_rxDcAvgQ(0),
    m_rxVga1(BLADERF_RXVGA1_GAIN_MAX),
    m_txShowInfo(0),
    m_modulated(false),
    m_predefinedData(0),
    m_dumpMutex(true,s_dumpMutexName),
    m_txDump(0),
    m_loopbackFreq(0),
    m_dumpTxTime(false),
    m_txDumper(0),
    m_rxDumper(0)
{
    setSpeed(false);
    initTestData();
}

BrfIface::~BrfIface()
{
    doClose(true,true);
}

// Initialize radio
bool BrfIface::init(const NamedList& params)
{
    setBrfLibLog(params[YSTRING("bladerf_debug_level")]);
    return RadioIface::init(params) && open(params);
}

// Set Rx/Tx frequency
int BrfIface::tune(bool rx, double freq, double adj)
{
    return setFreq(rx,freq + 600000,adj,false) ? 0 : -1;
}

// Set Rx/Tx power gain
int BrfIface::radioSetGain(bool rx, double dB, double* setVal)
{
    int rVal = 0;
    int ret = setGain(rx,dB,true,false,&rVal);
    if (!ret && setVal)
	*setVal = rVal;
    return ret;
}

// Execute a command
int BrfIface::command(const String& cmd, String* rspParam, String* reason)
{
    int status = Transceiver::CmdEOk;
    if (!cmd)
	return status;
    DDebug(this,DebugAll,"%sHandling command '%s' [%p]",prefix(),cmd.c_str(),this);
    if (cmd.startsWith("rx ")) {
	String s = cmd.substr(3);
	status = handleRxCmds(s,rspParam,reason);
    }
    else if (cmd.startsWith("tx ")) {
	String s = cmd.substr(3);
	status = handleTxCmds(s,rspParam,reason);
    }
    else if (cmd.startsWith("bladerf_debug_level "))
	setBrfLibLog(cmd.substr(20));
    else if (cmd.startsWith("unmodulated 10-10")) {
	// unmodulated 10-10 command will replace the data that should be sent 
	// to bladeRF with an energized form of the (1,0i),(0,0i),(-1,0i),(0,0i)...
	m_modulated = false;
	m_predefinedData = s_tenTen;
    } else if (cmd.startsWith("unmodulated 0000") || 
	    cmd.startsWith("unmodulated") || cmd.startsWith("off")) {
	// unmodulated <0000> command will replace the data that should be sent 
	// to bladeRF with (0,0i),(0,0i),(0,0i),(0,0i)...
	m_modulated = false;
	m_predefinedData = s_zeroZero;
    } else if (cmd.startsWith("modulated") || cmd.startsWith("normal")) {
	m_modulated = true;
	m_predefinedData = 0;
    } else if (cmd.startsWith("VCTCXO ",false,true)) {
	String s = cmd.substr(7);
	setVCTCXO(s.toInteger(),false);
    } else if (cmd.startsWith("loopback ")) {
	String s = cmd.substr(9);
	int l = lookup(s,s_loopbackMode,BLADERF_LB_NONE);
	return tuneCmdReturn(setLoopback(l));
    } else if (cmd.startsWith("loopback")) {
	int l = 0;
	bool ret = getLoopback(l);
	if (ret)
	    rspParam->append(lookup(l,s_loopbackMode));
	return tuneCmdReturn(ret);
    } else if (cmd.startsWith("sampling ")) {
	String s = cmd.substr(9);
	int l = lookup(s,s_sampling,BLADERF_SAMPLING_UNKNOWN);
	return tuneCmdReturn(setSampling(l));
    } else if (cmd.startsWith("sampling")) {
	int s = 0;
	bool ret = getSampling(s);
	if (ret)
	    rspParam->append(lookup(s,s_sampling));
	return tuneCmdReturn(ret);
    } else if (cmd.startsWith("lna_gain ")) {
	String s = cmd.substr(9);
	int l = lookup(s,s_lnaGain,BLADERF_LNA_GAIN_UNKNOWN);
	return tuneCmdReturn(setLnaGain(l));
    } else if (cmd.startsWith("lna_gain")) {
	int l = 0;
	bool ret = getLnaGain(l);
	if (ret)
	    rspParam->append(lookup(l,s_lnaGain));
	return tuneCmdReturn(ret);
    } else if (cmd.startsWith("tx-time")) {
	m_dumpTxTime = true;
    } else if (cmd.startsWith("board-dump")) {
	unsigned int frames = 0;
	if (::sscanf(cmd.c_str(),"board-dump %u",&frames) != 1) {
	    frames = 100;
	}
	u_int64_t samples = frames * 8 * BITS_PER_TIMESLOT * m_samplesPerSymbol;
	Time t;
	DataDumper* drx = new DataDumper(this,samples,8 * BITS_PER_TIMESLOT);
	String rxn("rx_");
	rxn << t.sec();
	drx->open(rxn);
	setRxDump(drx);

	DataDumper* dtx = new DataDumper(this,samples,8 * BITS_PER_TIMESLOT);
	String txn("tx_");
	txn << t.sec();
	dtx->open(txn);
	setTxDump(dtx);
    } else
	return RadioIface::command(cmd,rspParam,reason);
    return status;
}

// Destroy the object
void BrfIface::destruct()
{
    doClose(true,true);
    RadioIface::destruct();
}

// Read data from radio
bool BrfIface::readRadio(RadioIOData& data, unsigned int* samples)
{
    BRF_RX_SERIALIZE_ON_RET(false,true);
    if (!flag(BrfRxOn)) {
	Debug(this,DebugNote,"%sCan't read: Rx module not enabled [%p]",
		prefix(),this);
	return true;
    }
#ifdef BRF_DEBUG_RECV
    Debugger rd(DebugNote,"BrfIface::read"," [%p]",this);
#endif
    unsigned int len = samples ? *samples : 0;
    if (!len || len > data.free()) {
	len = data.free();
	if (samples)
	    *samples = len;
    }
    int16_t* startBuf = data.buffer();
    while (len) {
#ifdef BRF_DEBUG_RECV
	Debug(this,DebugAll,
	    "%sREAD LOOP DATA: ts=" FMT64U " free=%u BUF: ts=" FMT64U " buffer=%u/%u pos=%u [%p]",
	    prefix(),data.timestamp(),len,
	    m_rxIO.m_timestamp,m_rxIO.current(),m_rxIO.m_buffers,m_rxIO.m_pos,this);
#endif
	if (m_rxIO.m_timestamp > data.timestamp()) {
	    // Upper layer timestamp is older then our timestamp
	    // Fill recv buffer with 0 for the difference
	    unsigned int delta = (unsigned int)(m_rxIO.m_timestamp - data.timestamp());
	    delta = data.fill(delta <= len ? delta : len);
	    len -= delta;
#ifdef BRF_DEBUG_RECV
	    Debug(this,DebugNote,"%sRx filled %u samples until ts=" FMT64U " [%p]",
		prefix(),delta,m_rxIO.m_timestamp,this);
#endif
	    if (!len)
		break;
	}
	else if (m_rxIO.m_timestamp < data.timestamp()) {
	    // Overrun: upper layer timestamp is newer then our timestamp
	    int discard = m_rxIO.available();
	    if (discard > 0) {
		// Discard our data
		unsigned int delta = (unsigned int)(data.timestamp() - m_rxIO.m_timestamp);
		if ((unsigned int)discard > delta)
		    discard = delta;
		m_rxIO.advance(discard);
#ifdef BRF_DEBUG_RECV
		Debug(this,DebugNote,"%sRx discarded %d samples at ts=" FMT64U " [%p]",
		    prefix(),discard,m_rxIO.m_timestamp,this);
#endif
		if (m_rxResyncCandidate)
		    m_rxResyncCandidate += discard;
	    }
	}
	int avail = m_rxIO.available();
	if (avail > 0) {
	    int16_t* p = m_rxIO.currentSamples() + (m_rxIO.m_pos * 2);
	    unsigned int cp = (avail <= (int)len) ? (unsigned int)avail : len;
	    cp = data.copy(p,cp);
#ifdef BRF_DEBUG_RECV
	    Debug(this,DebugAll,"%sRx copy %u samples at ts=" FMT64U " [%p]",
		prefix(),cp,m_rxIO.m_timestamp,this);
#endif
	    m_rxIO.advance(cp);
	    len -= cp;
	    if (m_rxResyncCandidate)
		m_rxResyncCandidate += cp;
	    if (!len)
		break;
	}
	if (!m_rxIO.advanceCurrent()) {
	    // No more data in buffer, read from device
	    unsigned int n = len;
	    n = (n + m_rxIO.useSamples() - 1) / m_rxIO.useSamples();
	    if (n > m_rxIO.count())
		n = m_rxIO.count();
	    unsigned int rd = m_rxIO.bufSamples() * n;
	    int ok = ::bladerf_sync_rx(m_dev,m_rxIO.buffer(),rd,0,data.syncIOWait());
	    if (thShouldExit(this))
		return true;
	    if (ok < 0) {
		Debug(this,DebugNote,
		    "%sRead failed (%u samples at ts=" FMT64U "): %d %s [%p]",
		    prefix(),rd,m_rxIO.m_timestamp,ok,::bladerf_strerror(ok),this);
		return updateError(true,false);
	    }
	    printIO(this,true,m_rxIO,m_txIO,n);
	    updateError(true,true);
	    m_rxIO.m_buffers = n;
	    computeRx();
	}
	m_rxIO.m_pos = 0;
	BrfTsHeader& hdr = m_rxIO.currentHeader();
	uint64_t hi = swapBytes(hdr.timeHi);
	uint64_t lo = swapBytes(hdr.timeLo);
	// bladeRF timestamp is given in int16_t elements count,
	//  our timestamp is in samples
	uint64_t ts = (hi << 31) | (lo >> 1);
	m_lastBoardTs = ts;
#ifdef BRF_DEBUG_RECV
	Debug(this,DebugAll,"%sCurrent buffer %u ts=" FMT64U " (%u/%u) [%p]",
	    prefix(),m_rxIO.current(),ts,(unsigned int)hi,(unsigned int)lo,this);
#endif
	if (ts == m_rxIO.m_timestamp)
	    continue;
	if (m_rxIO.m_timestamp == initialReadTs()) {
	    // First read gave us a different timestamp: use it as initial read ts
	    m_rxIO.m_timestamp = ts;
	    data.timestamp(ts);
	    m_rxResyncCandidate = 0;
	    continue;
	}
	int64_t delta = (int64_t)(ts - m_rxIO.m_timestamp);
	if (ts == m_rxResyncCandidate || abs(delta) < 1000) {
	    Debug(this,DebugNote,
		"%sRx timestamp adjusted by " FMT64 " to " FMT64U " [%p]",
		prefix(),delta,ts,this);
	    m_rxIO.m_timestamp = ts;
	    m_rxResyncCandidate = 0;
	    // NOTE: delta > 0 means overrun
	}
	else {
	    Debug(this,DebugWarn,
		"%sRx timestamp jumped by " FMT64 " at " FMT64U " in buffer %u/%u [%p]",
		prefix(),delta,m_rxIO.m_timestamp,m_rxIO.current(),m_rxIO.m_buffers,this);
	    m_rxResyncCandidate = ts;
        }
        // NOTE: See if we can use the header flags to detect data underrun or RSSI
    }
#ifdef BRF_DEBUG_RECV
    Debug(this,DebugAll,
	"%sreadRadio() exiting DATA: ts=" FMT64U " free=%u BUF: ts=" FMT64U " buf=%u/%u pos=%u [%p]",
	prefix(),data.timestamp(),len,m_rxIO.m_timestamp,
	m_rxIO.current(),m_rxIO.m_buffers,m_rxIO.m_pos,this);
#endif
    if (m_rxDumper)
	m_rxDumper->write(startBuf,*samples * 4,m_lastBoardTs);

    return true;
}

// Write data to radio
bool BrfIface::writeRadio(RadioIOData& data)
{
    BRF_TX_SERIALIZE_ON_RET(false,false);
    if (!flag(BrfTxOn)) {
	Debug(this,DebugNote,"%sCan't write: Tx module not enabled [%p]",
	    prefix(),this);
	return true;
    }
    int16_t* buf = data.data();
    unsigned int len = data.pos();

    if (!(buf && len))
	return true;
    if (m_txDumper)
	m_txDumper->write((void*)buf,len * 4,data.timestamp());
    if (m_txShowInfo) {
        if (m_txShowInfo > 0)
            m_txShowInfo--;
	if (debugAt(DebugAll)) {
	    int minVal = 0;
	    int maxVal = 0;
	    int16_t* b = buf;
	    for (unsigned int n = len * 2; n; n--, b++)
		computeMinMax(minVal,maxVal,*b);
	    Debug(this,DebugAll,"%sTx min=%d max=%d [%p]",prefix(),minVal,maxVal,this);
	}
    }
    unsigned int consumed = 0;
    uint64_t dataTs = data.timestamp() - data.pos();
#ifdef BRF_DEBUG_SEND
    Debugger wr(DebugNote,"BrfIface::write"," samples=%u ts=" FMT64U " [%p]",
	len,dataTs,this);
#endif
    // Drop the buffer and resync if timestamp is not matching
    if (dataTs != m_txIO.m_timestamp) {
	Debug(this,DebugWarn,
	    "%sTx timestamp difference our=" FMT64U " input=" FMT64U " [%p]",
	    prefix(),m_txIO.m_timestamp,dataTs,this);
	m_txIO.m_pos = 0;
	m_txIO.m_timestamp = dataTs;
    }
    unsigned int& nBuf = m_txIO.m_buffers;
    u_int64_t lastTimestamp = 0;
    while (consumed < len) {
#ifdef BRF_DEBUG_SEND
	Debugger debug(DebugAll,"BrfIface SEND LOOP",
	    " consumed=%u BUF: ts=" FMT64U " pos=%u [%p]",
	    consumed,m_txIO.m_timestamp,m_txIO.m_pos,this);
#endif
	// Fill the buffers
	for (; consumed < len && nBuf < m_txIO.count(); nBuf++, m_txIO.m_pos = 0) {
	    int avail = m_txIO.available();
	    if (avail <= 0)
		continue;
	    unsigned int cp = len - consumed;
	    if (cp > (unsigned int)avail)
		cp = avail;
	    // Don't fill a partial buffer if not the first one
	    // We will fill it in the next loop
	    if (m_txIO.m_buffers && (m_txIO.m_pos + cp) < m_txIO.useSamples())
		break;
	    int16_t* p = m_txIO.samples(m_txIO.m_buffers) + (m_txIO.m_pos * 2);
	    if (!m_modulated && m_predefinedData)
		::memcpy(p,m_predefinedData,cp * sizeof(int16_t) * 2);
	    else
		::memcpy(p,buf,cp * sizeof(int16_t) * 2);
	    
#ifdef BRF_DEBUG_SEND
	    Debug(this,DebugAll,"%sTx copied %u samples buf=%u at ts=" FMT64U " [%p]",
		prefix(),cp,m_txIO.m_buffers,m_txIO.m_timestamp,this);
#endif
	    // New buffer: set the timestamp
	    if (!m_txIO.m_pos) {
		// bladeRF timestamp is given in int16_t elements count,
		//  our timestamp is in samples
		BrfTsHeader& hdr = m_txIO.header(m_txIO.m_buffers);
		hdr.timeLo = swapBytes((uint32_t)(m_txIO.m_timestamp << 1));
		hdr.timeHi = swapBytes((uint32_t)(m_txIO.m_timestamp >> 31));
		hdr.reserved = 0xdeadbeef;
		hdr.flags = (uint32_t)-1;
		lastTimestamp = m_txIO.m_timestamp;
	    }
	    m_txIO.advance(cp);
	    buf += cp * 2;
	    consumed += cp;
	    // Don't advance the buffers count if partially filled
	    if (m_txIO.m_pos < m_txIO.useSamples())
		break;
	}
	if (!m_txIO.m_buffers)
	    break;
	unsigned int n = m_txIO.m_buffers;
	m_txIO.m_buffers = 0;

	if (m_txDump) {
	    Lock myLock(m_dumpMutex);
	    if (m_txDump)
		m_txDump->dump(m_txIO.buffer(),m_txIO.bufSamples());
	}

	int ok = ::bladerf_sync_tx(m_dev,m_txIO.buffer(),n * m_txIO.bufSamples(),
	    0,data.syncIOWait());
	if (thShouldExit(this))
	    return true;
	if (ok < 0)
	    Debug(this,DebugNote,"%sWrite failed %u samples at " FMT64U ": %d %s [%p]",
		prefix(),n * m_txIO.useSamples(),m_txIO.m_timestamp,
		ok,::bladerf_strerror(ok),this);
#ifdef BRF_DEBUG_SEND
	else
	    Debug(this,DebugInfo,"%sWrote %u samples at ts=" FMT64U " [%p]",
		prefix(),n * m_txIO.useSamples(),m_txIO.m_timestamp,this);
#endif
	if (!updateError(false,ok >= 0))
	    return false;
    }
#ifdef BRF_DEBUG_SEND
    Debug(this,DebugAll,
	"%swriteRadio() exiting DATA: ts=" FMT64U " consumed=%u/%u BUF: ts=" FMT64U " pos=%u [%p]",
	prefix(),data.timestamp(),consumed,data.pos(),m_txIO.m_timestamp,m_txIO.m_pos,this);
#endif
    data.consumed(consumed);
    if (m_dumpTxTime) {
	m_dumpTxTime = false;
	GSMTime lt,bt,diff,rc,diffr;
	uint fd = 1250 * 8;
	lt.assign(lastTimestamp / fd,(lastTimestamp % fd) / 1250);
	bt.assign(data.timestamp() / fd,(data.timestamp() % fd) / 1250);
	GSMTime::diff(diff,bt,lt);
	getRadioClock(rc);
	GSMTime::diff(diffr,data.m_time,rc);
	Debug(this,DebugNote,"\nTxBoard time FN %d TN %d unused %d\nTxRadio time FN %d TN %d unused %d\nTxDiff %d %d\nTxRadio Clock FN %d TN %d, radioClock FN %d TN %d diff %d %d",
		    lt.fn(),lt.tn(),(int)(lastTimestamp % 1250),bt.fn(),bt.tn(),(int)(data.timestamp() % 1250),diff.fn(),diff.tn(),data.m_time.fn(),data.m_time.tn(),rc.fn(),rc.tn(),diffr.fn(),diffr.tn());

    }
    return true;
}

// Start the radio device
bool BrfIface::radioPowerOn()
{
    BRF_RX_SERIALIZE_ON_RET(false,false);
    BRF_TX_SERIALIZE_ON_RET(false,false);
    if (!m_dev)
	return false;
    doPowerOff(true,true);
    if (!(initSyncIO() && initTimestampsGPIO() && enableModules())) {
	doPowerOff(true,true);
	return false;
    }
    m_rxErrors.reset();
    m_txErrors.reset();
    m_rxIO.m_timestamp = initialReadTs();
    m_rxIO.m_buffers = 0;
    m_rxIO.m_pos = m_rxIO.useSamples();
    m_txIO.m_timestamp = initialWriteTs();
    m_txIO.m_pos = 0;
    m_txIO.m_buffers = 0;
    m_rxResyncCandidate = 0;
    m_rxDcAvgI = 0;
    m_rxDcAvgQ = 0;
    return true;
}

bool BrfIface::initTimestampsGPIO()
{
    // Enable timestamps
    // 0x10000: enable timestamps, clears and resets everything on write
    uint32_t val = 0;
    ::bladerf_config_gpio_read(m_dev,&val);
    val |= 0x10000;
    ::bladerf_config_gpio_write(m_dev,val);
    val = 0;
    ::bladerf_config_gpio_read(m_dev,&val);
    if ((val & 0x10000) != 0)
	return true;
    Debug(this,DebugNote,"%sFailed to enable timestamps [%p]",prefix(),this);
    return false;
}

// Stop the radio device
void BrfIface::radioPowerOff()
{
    doPowerOff(false,false);
}

// Open the device
bool BrfIface::open(const NamedList& params)
{
    BRF_RX_SERIALIZE_ON_RET(false,false);
    BRF_TX_SERIALIZE_ON_RET(false,false);
    doClose(true,true);
    int code = 0;
    String what;
    const char* dev = params.getValue(YSTRING("device_args"));
    String fpga;
    unsigned int rxRate = 0;
    unsigned int txRate = 0;
    unsigned int rxBw = 0;
    unsigned int txBw = 0;
#define BRF_SET_OPER_BREAK(oper) { what = oper; break; }
    Lock lck(m_mutex);
    while (true) {
	// Open the device
	code = ::bladerf_open(&m_dev,dev);
	if (code < 0)
	    BRF_SET_OPER_BREAK("Failed to open");
	// Load FPGA
	if (!loadFPGA(params,code,what,fpga))
	    break;
	// Update device speed
	bladerf_dev_speed speed = ::bladerf_device_speed(m_dev);
	if (speed != BLADERF_DEVICE_SPEED_HIGH && speed != BLADERF_DEVICE_SPEED_SUPER) {
	    code = 0;
	    BRF_SET_OPER_BREAK("Unknown device speed");
	}
	setSpeed(speed == BLADERF_DEVICE_SPEED_SUPER,params);
	// Set Rx/Tx sampling rate
	if (!(setSampleRate(true,code,what,&rxRate) &&
	    setSampleRate(false,code,what,&txRate)))
	    break;
	// Set bandwith
	if (!(setBandwith(true,code,what,BANDWIDTH,&rxBw) &&
	    setBandwith(false,code,what,BANDWIDTH,&txBw)))
	    break;
	m_loopbackFreq = params.getIntValue(YSTRING("loopback-freq"),0);
	// Init sync I/O timeouts
	unsigned int tout = getUInt(params,YSTRING("io_timeout"),500,Thread::idleMsec());
	m_rxData.syncIOWait(tout);
	m_txData.syncIOWait(tout);
	m_rxShowDcInfo = 0;
	m_txShowInfo = 0;
	m_rxVga1 = BLADERF_RXVGA1_GAIN_MAX;
	m_rxDcAuto = params.getBoolValue(YSTRING("rx_dc_autocorrect"),true);
	m_rxDcOffsetMax = BRF_RX_DC_OFFSET_DEF;
	m_rxDcOffsetI = m_rxDcOffsetQ = BRF_RX_DC_OFFSET_MAX + 1;
	setCorrection(true,params.getIntValue(YSTRING("RX.OffsetI")),
	    params.getIntValue(YSTRING("RX.OffsetQ")),true);
	setCorrection(false,params.getIntValue(YSTRING("TX.OffsetI")),
	    params.getIntValue(YSTRING("TX.OffsetQ")),true);
	setGain(true,BLADERF_RXVGA2_GAIN_MIN);
	setGain(false,BLADERF_TXVGA2_GAIN_MIN);
	break;
    }
    bool done = thShouldExit(this);
    if (what || done) {
	if (!done) {
	    String s;
	    if (code)
		s << ": " << code << " " << ::bladerf_strerror(code);
	    Debug(this,DebugWarn,"%s%s device=%s%s [%p]",
		prefix(),what.c_str(),dev,s.safe(),this);
	}
	doClose(true,true);
	return false;
    }
    // Print data
    int level = DebugInfo;
    struct bladerf_version ver;
    ::bladerf_version(&ver);
    addVersion(what,ver,"\r\nlibrary=");
    struct bladerf_devinfo info;
    ::bladerf_init_devinfo(&info);
    what << "\r\ndevice=";
    if (addError(what,level,::bladerf_get_devinfo(m_dev,&info)) >= 0) {
	what << "usb_bus:" << info.usb_bus << " usb_addr:" << info.usb_addr;
	what << " instance:" << info.instance;
	String tmp(info.serial,sizeof(info.serial));
	what << " serial:" << tmp;
    }
    what << "\r\nfirmware=";
    brfAddVer(what,level,::bladerf_fw_version(m_dev,&ver),ver);
    what << "\r\nFPGA=";
    if (fpga)
	what << fpga <<  " - ";
    what << "version ";
    brfAddVer(what,level,::bladerf_fpga_version(m_dev,&ver),ver);
    what << "\r\nsuperspeed=" << String::boolText(m_superSpeed);
    what << "\r\nrx_sample_rate=" << rxRate;
    what << "\r\ntx_sample_rate=" << txRate;
    what << "\r\nrx_bandwith=" << rxBw;
    what << "\r\ntx_bandwith=" << txBw;
    what << "\r\nrx_buffers=" << m_rxIO.count() << "/" << m_rxIO.libBuffers();
    what << "\r\ntx_buffers=" << m_txIO.count() << "/" << m_txIO.libBuffers();
    Debug(this,level,"%sOpened bladeRF: [%p]\r\n-----%s\r\n-----",
	prefix(),this,what.c_str());
    return true;
}

void BrfIface::doPowerOff(bool safeRx, bool safeTx)
{
    BRF_RX_SERIALIZE_ON_NONE(safeRx);
    BRF_TX_SERIALIZE_ON_NONE(safeTx);
    enableModule(true,false);
    enableModule(false,false);
}

// Close the device
void BrfIface::doClose(bool safeRx, bool safeTx)
{
    BRF_RX_SERIALIZE_ON_NONE(safeRx);
    BRF_TX_SERIALIZE_ON_NONE(safeTx);
    doPowerOff(true,true);
    struct bladerf* dev = m_dev;
    m_dev = 0;
    if (!dev)
	return;
    ::bladerf_close(dev);
    Debug(this,DebugInfo,"%sClosed bladeRF device [%p]",prefix(),this);
}

bool BrfIface::enableModule(bool rx, bool on)
{
    if (!m_dev)
	return false;
    uint8_t flg = rx ? BrfRxOn : BrfTxOn;
    if (on == isFlag(flg)) {
	// Just to make sure: always disable the module
	if (!on)
	    ::bladerf_enable_module(m_dev,rxtxmod(rx),false);
	return true;
    }
    int ok = ::bladerf_enable_module(m_dev,rxtxmod(rx),on);
    if (ok >= 0) {
	setFlag(flg,on);
	Debug(this,DebugAll,"%s%s %s module [%p]",
	    prefix(),on ? "Enabled" : "Disabled",rxtx(rx),this);
	return true;
    }
    Debug(this,DebugWarn,"%sFailed to %s %s module: %d %s [%p]",
	prefix(),on ? "enable" : "disable",rxtx(rx),ok,::bladerf_strerror(ok),this);
    return false;
}

bool BrfIface::loadFPGA(const NamedList& params, int& code, String& what, String& fpga)
{
    const String& oper = params[YSTRING("bladerf_load_fpga")];
    bool load = false;
    if (!oper)
	load = true;
    else if (oper.isBoolean())
	load = oper.toBoolean();
    else if (oper == YSTRING("check")) {
	code = ::bladerf_is_fpga_configured(m_dev);
	load = (code != 1);
	if (load && code < 0)
	    Debug(this,DebugNote,"%sForcing FPGA load (check failure: %d %s) [%p]",
		prefix(),code,::bladerf_strerror(code),this);
    }
    if (!load)
	return true;
    bladerf_fpga_size sz;
    code = ::bladerf_get_fpga_size(m_dev,&sz);
    if (code < 0) {
	what << "Failed to retrieve FPGA size";
	return false;
    }
    if (sz == BLADERF_FPGA_40KLE)
	fpga = params.getValue(YSTRING("bladerf_fpga_40k"),"hostedx40.rbf");
    else if (sz == BLADERF_FPGA_115KLE)
	fpga = params.getValue(YSTRING("bladerf_fpga_115k"),"hostedx115.rbf");
    else {
	what << "Unknown FPGA size " << (int)sz;
	code = 0;
	return false;
    }
    code = ::bladerf_load_fpga(m_dev,fpga);
    if (code < 0)
	what << "Failed to load FPGA from file '" << fpga << "'";
    return code >= 0;
}

bool BrfIface::setSampleRate(bool rx, int& code, String& what, unsigned int* actual)
{
#define GSM_SAMPLE_RATE (13e6 / 48) + 1
    unsigned int rate = m_samplesPerSymbol * GSM_SAMPLE_RATE;
    unsigned int result = 0;
    if (!actual)
	actual = &result;

    code = ::bladerf_set_sample_rate(m_dev,rxtxmod(rx),rate,actual);
    if (code < 0) {
	what << "Failed to set " << rxtx(rx) << " sampling rate to " << rate;
	return false;
    }
    if (rate == *actual)
	Debug(this,DebugAll,"%sSet %s sampling rate to %u [%p]",
	    prefix(),rxtx(rx),*actual,this);
    else
	Debug(this,DebugGoOn,
	    "%s%s sampling rate set failed requested=%u current=%u [%p]",
	    prefix(),rxtx(rx),rate,*actual,this);
    return true;
}

bool BrfIface::setBandwith(bool rx, int& code, String& what, unsigned int value,
    unsigned int* actual)
{
    unsigned int bw = 0;
    if (!actual)
	actual = &bw;
    code = ::bladerf_set_bandwidth(m_dev,rxtxmod(rx),value,actual);
    if (code >= 0) {
	if (*actual >= value)
	    Debug(this,DebugAll,"%sSet %s bandwith %u [%p]",
		prefix(),rxtx(rx),*actual,this);
	else
	    Debug(this,DebugGoOn,
		"%s%s bandwith failed requested=%u current=%u [%p]",
		prefix(),rxtx(rx),value,*actual,this);
	return true;
    }
    if (updateError(rx,false)) {
	Debug(this,DebugNote,"%sFailed to set %s bandwith to %u: %d %s [%p]",
	    prefix(),rxtx(rx),value,code,::bladerf_strerror(code),this);
	return true;
    }
    what << "Failed to set " << rxtx(rx) << " bandwith";
    return false;
}

bool BrfIface::initSyncIO(bool rx, unsigned int numBuffers, unsigned int bufSizeSamples,
    unsigned int numTransfers, int* code, String* what)
{
    unsigned int tout = rx ? m_rxData.syncIOWait() : m_txData.syncIOWait();
    int ok = ::bladerf_sync_config(m_dev,rxtxmod(rx),BLADERF_FORMAT_SC16_Q11,
	numBuffers,bufSizeSamples,numTransfers,tout);
    if (ok >= 0) {
	Debug(this,DebugAll,
	   "%sInitialized sync %s buffers=%u size=%u transfers=%u tout=%u [%p]",
	    prefix(),rxtx(rx),numBuffers,bufSizeSamples,numTransfers,tout,this);
	if (code)
	    *code = ok;
	return true;
    }
    String tmp;
    if (!what)
	what = &tmp;
    *what << "Failed to init " << rxtx(rx) << " sync operation";
    if (!code)
	Debug(this,DebugWarn,"%s%s: %d %s [%p]",
	    prefix(),what->c_str(),ok,::bladerf_strerror(ok),this);
    else
	*code = ok;
    return false;
}

bool BrfIface::setCorrection(bool rx, int offsI, int offsQ, bool safe)
{
    int i = rx ? BRF_RX_DC_OFFSET_MAX : BRF_TX_DC_OFFSET_MAX;
    String error;
    if (!isIntervalInt(offsI,-i,i))
	error.append("I="," ") << offsI;
    if (!isIntervalInt(offsQ,-i,i))
	error.append("Q="," ") << offsQ;
    if (error) {
	Debug(this,DebugNote,"%sInvalid %s DC offset %s [%p]",
	    prefix(),rxtx(rx),error.c_str(),this);
	return false;
    }
    BRF_TX_SERIALIZE_ON_RET(safe,false);
    if (rx && offsI == m_rxDcOffsetI && offsQ == m_rxDcOffsetQ)
	return true;
    if (!(setCorrection(rx,true,offsI) && setCorrection(rx,false,offsQ)))
	return false;
    if (rx) {
	m_rxDcOffsetI = offsI;
	m_rxDcOffsetQ = offsQ;
    }
    int shift = (rx ? BRF_RX_DC_OFFSET_SHIFT : BRF_TX_DC_OFFSET_SHIFT);
    Debug(this,DebugAll,"%sSet %s DC offset I=%d Q=%d [%p]",
	prefix(),rxtx(rx),(offsI << shift),(offsQ << shift),this);
    return true;
}

bool BrfIface::setCorrection(bool rx, bool offsI, int val)
{
    int16_t tmp = val << (rx ? BRF_RX_DC_OFFSET_SHIFT : BRF_TX_DC_OFFSET_SHIFT);
    int ok = ::bladerf_set_correction(m_dev,rxtxmod(rx),
	offsI ? BLADERF_CORR_LMS_DCOFF_I : BLADERF_CORR_LMS_DCOFF_Q,tmp);
    if (ok >= 0)
	return true;
    Debug(this,DebugNote,"%sFailed to set %s DC offset %s value %d: %d %s [%p]",
	prefix(),rxtx(rx),(offsI ? "I" : "Q"),tmp,ok,::bladerf_strerror(ok),this);
    return false;
}

bool BrfIface::setVCTCXO(unsigned int val, bool safe)
{
    BRF_TX_SERIALIZE_ON_RET(safe,false);
    int ok = ::bladerf_dac_write(m_dev,val << 8);
    if (ok >= 0) {
	Debug(this,DebugAll,"%sVCTCXO set to %u [%p]",prefix(),val,this);
	return true;
    }
    Debug(this,DebugGoOn,"%sFailed to set VCTCXO to %u: %d %s [%p]",
	prefix(),val,ok,::bladerf_strerror(ok),this);
    return false;
}

bool BrfIface::setFreq(bool rx, unsigned int hz, unsigned int vctcxo, bool safe)
{
    BRF_TX_SERIALIZE_ON_RET(safe,false);
    if (m_loopbackFreq)
	hz = m_loopbackFreq;
    int ok = ::bladerf_set_frequency(m_dev,rxtxmod(rx),hz);
    if (ok < 0) {
	Debug(this,DebugGoOn,"%sFailed to set %s frequency %uHz: %d %s [%p]",
	    prefix(),rxtx(rx),hz,ok,::bladerf_strerror(ok),this);
	return false;
    }
    Debug(this,DebugAll,"%s%s frequency set to %gMHz (%uHz) [%p]",
	prefix(),rxtx(rx),((double)hz / 1000000),hz,this);
    return setVCTCXO(vctcxo,true);
}

// Set Rx/TX power gain.
// Return 0 on success, negative on fatal error, positive on fatal error
int BrfIface::setGain(bool rx, int vga2, bool updError, bool safe, int* setVal)
{
    BRF_TX_SERIALIZE_ON_RET(safe,2);
    int vga1 = 0;
    if (rx) {
	clampInt(vga2,BLADERF_RXVGA2_GAIN_MIN,BLADERF_RXVGA2_GAIN_MAX);
	vga1 = m_rxVga1;
	if (setVal)
	    *setVal = vga2;
    }
    else {
	// TODO: Check if this is correct
	Debug(DebugStub,"Please check BrfIface::setGain() for TX");
	vga1 = BLADERF_TXVGA1_GAIN_MAX;
	vga2 += BLADERF_TXVGA2_GAIN_MAX;
	clampInt(vga2,BLADERF_TXVGA2_GAIN_MIN,BLADERF_TXVGA2_GAIN_MAX);
	if (setVal)
	    *setVal = vga2 - BLADERF_TXVGA2_GAIN_MAX;
    }
    int ok = setGainVga(true,rx,vga1,updError,true);
    if (ok > 0)
	return ok;
    return setGainVga(false,rx,vga2,updError,true);
}

// Set Rx/TX VGA1 (pre-LPF) or VGA2 (post-LPF) power gain.
// Return 0 on success, negative on library error, positive on fatal error
int BrfIface::setGainVga(bool pre, bool rx, int value, bool updError, bool safe)
{
    BRF_TX_SERIALIZE_ON_RET(safe,2);
    int ok = 0;
    if (pre) {
	if (rx) {
	    if (!isIntervalInt(value,BLADERF_RXVGA1_GAIN_MIN,BLADERF_RXVGA1_GAIN_MAX))
		ok = -2;
	}
	else if (!isIntervalInt(value,BLADERF_TXVGA1_GAIN_MIN,BLADERF_TXVGA1_GAIN_MAX))
	    ok = -2;
    }
    else if (rx) {
	if (!isIntervalInt(value,BLADERF_RXVGA2_GAIN_MIN,BLADERF_RXVGA2_GAIN_MAX))
	    ok = -2;
    }
    else if (!isIntervalInt(value,BLADERF_TXVGA2_GAIN_MIN,BLADERF_TXVGA2_GAIN_MAX))
	ok = -2;
    if (ok) {
	Debug(this,DebugNote,"%s%s VGA%s gain value %d out of range [%p]",
	    prefix(),rxtx(rx),(pre ? "1" : "2"),value,this);
	return ok;
    }
    if (pre) {
	if (rx)
	    ok = ::bladerf_set_rxvga1(m_dev,value);
	else
	    ok = ::bladerf_set_txvga1(m_dev,value);
    }
    else if (rx)
	ok = ::bladerf_set_rxvga2(m_dev,value);
    else
	ok = ::bladerf_set_txvga2(m_dev,value);
    if (ok >= 0) {
	DDebug(this,DebugAll,"%s%s VGA%s gain value set to %d [%p]",
	    prefix(),rxtx(rx),(pre ? "1" : "2"),value,this);
	if (pre) {
	    if (rx && m_rxVga1 != value) {
		Debug(this,DebugAll,"%s%s VGA1 gain value changed %d -> %d [%p]",
		    prefix(),rxtx(rx),m_rxVga1,value,this);
		m_rxVga1 = value;
	    }
	}
	else if (rx)
	    m_rxDcOffsetMax = (int)BRF_RX_DC_OFFSET(value);
	return 0;
    }
    Debug(this,DebugAll,"%sFailed to set %s VGA%s gain value %d: %d %s [%p]",
	prefix(),rxtx(rx),(pre ? "1" : "2"),value,ok,::bladerf_strerror(ok),this);
    if (updError && !updateError(rx,false))
	return 1;
    return ok;
}

// Set device speed and related data
void BrfIface::setSpeed(bool super, const NamedList& params)
{
    m_superSpeed = super;
    // Set buffers
    unsigned int libBuffs = getUInt(params,YSTRING("rx_lib_buffers"),8,1);
    unsigned int nBuffs = getUInt(params,YSTRING("rx_buffers"),4,1,libBuffs);
    m_rxIO.setBuffers(libBuffs,nBuffs,m_superSpeed);
    libBuffs = getUInt(params,YSTRING("tx_lib_buffers"),8,1);
    nBuffs = getUInt(params,YSTRING("tx_buffers"),1,1,libBuffs);
    m_txIO.setBuffers(libBuffs,nBuffs,m_superSpeed);
}

bool BrfIface::setLoopback(int loopback)
{
    return ::bladerf_set_loopback(m_dev, (bladerf_loopback)loopback) >= 0;
}

bool BrfIface::getLoopback(int& loopback)
{
    bladerf_loopback l;
    if (::bladerf_get_loopback(m_dev, &l) < 0)
	return false;
    loopback = l;
    return true;
}

bool BrfIface::setSampling(int sampling)
{
    return ::bladerf_set_sampling(m_dev, (bladerf_sampling)sampling) >= 0;
}

bool BrfIface::getSampling(int& sampling)
{
    bladerf_sampling s;
    if (::bladerf_get_sampling(m_dev, &s) < 0)
	return false;
    sampling = s;
    return true;
}

bool BrfIface::setLnaGain(int lnaGain)
{
    return ::bladerf_set_lna_gain(m_dev, (bladerf_lna_gain)lnaGain) >= 0;
}

bool BrfIface::getLnaGain(int& lna_gain)
{
    bladerf_lna_gain lg;
    if (::bladerf_get_lna_gain(m_dev, &lg) < 0)
	return false;
    lna_gain = lg;
    return true;
}

bool BrfIface::getSampleRate(bool rx, unsigned int& sampleRate)
{
    int ret = ::bladerf_get_sample_rate(m_dev,rxtxmod(rx),&sampleRate);
    if (ret < 0)
	return false;
    return true;
}

bool BrfIface::getBandwidth(bool rx, unsigned int &bandwidth)
{
    return ::bladerf_get_bandwidth(m_dev,rxtxmod(rx),&bandwidth) >= 0;
}

bool BrfIface::setLpfMode(bool rx, int lpfMode)
{
    return ::bladerf_set_lpf_mode(m_dev, rxtxmod(rx), (bladerf_lpf_mode)lpfMode) >= 0;
}

bool BrfIface::getLpfMode(bool rx, int &lpfMode)
{
    bladerf_lpf_mode lm;
    if (::bladerf_get_lpf_mode(m_dev, rxtxmod(rx), &lm) < 0)
	return false;
    lpfMode = lm;
    return true;
}

bool BrfIface::getVga(bool rx, bool one, int& gain)
{
    if (rx) {
	if (one)
	    return ::bladerf_get_rxvga1(m_dev,&gain) >= 0;
	return::bladerf_get_rxvga2(m_dev,&gain) >= 0;
    }
    if (one)
	return ::bladerf_get_txvga1(m_dev,&gain) >= 0;
    return ::bladerf_get_txvga2(m_dev,&gain) >= 0;
}

bool BrfIface::getFrequency(bool rx, unsigned int &freq)
{
    return ::bladerf_get_frequency(m_dev,rxtxmod(rx),&freq) >= 0;
}

bool BrfIface::setRxDump(DataDumper* data)
{
    BRF_RX_SERIALIZE_ON_RET(false,false);
    DataDumper* d = m_rxDumper;
    m_rxDumper = data;
    if (d)
	delete d;
    return true;
}

bool BrfIface::setTxDump(DataDumper* data)
{
    BRF_TX_SERIALIZE_ON_RET(false,false);
    DataDumper* d = m_txDumper;
    m_txDumper = data;
    if (d)
	delete d;
    return true;
}

bool BrfIface::resetDumper(DataDumper* d, bool safe)
{
    if (!d)
	return true;
    if (m_txDumper && d == m_txDumper) {
	BRF_TX_SERIALIZE_ON_RET(safe,false);
	delete m_txDumper;
	m_txDumper = 0;
    } else if (m_rxDumper && d == m_rxDumper) {
	BRF_RX_SERIALIZE_ON_RET(safe,false);
	if (m_rxDumper)
	    delete m_rxDumper;
	m_rxDumper = 0;
    }
    return true;
}

static inline void computeRxAdjustPeak(int& p, int val, uint64_t& peakTs, uint64_t& ts)
{
    if (p >= val)
	return;
    p = val;
    peakTs = ts;
}

static inline void computeRxAdjustDC(int& corr, int& avg, int dcOffsMax)
{
    if ((avg > dcOffsMax) && (corr < BRF_RX_DC_OFFSET_MAX))
	corr++;
    else if ((avg < -dcOffsMax) && (corr > -BRF_RX_DC_OFFSET_MAX))
	corr--;
    else
	return;
    avg = 0;
}

// Compute Rx avg values, autocorrect offsets if configured
void BrfIface::computeRx()
{
    bool debug = m_rxShowDcInfo != 0;
    if (!(m_rxDcAuto || debug))
	return;
    // Compute averages and peak values
    int dcIMin = 32767;
    int dcIMax = -32767;
    int dcIAvg = 0;
    int dcQMin = 32767;
    int dcQMax = -32767;
    int dcQAvg = 0;
    int peak = 0;
    uint64_t ts = m_rxIO.m_timestamp;
    uint64_t peakTs = 0;
    for (unsigned int i = 0; i < m_rxIO.m_buffers; i++) {
	int16_t* b = m_rxIO.samples(i);
	for (unsigned int j = m_rxIO.useSamples(); j; j--, ts++) {
	    int dcI = *b++;
	    int dcQ = *b++;
	    dcIAvg += dcI;
	    dcQAvg += dcQ;
	    if (!debug)
		continue;
	    computeMinMax(dcIMin,dcIMax,dcI);
	    computeMinMax(dcQMin,dcQMax,dcQ);
	    computeRxAdjustPeak(peak,dcIMax,peakTs,ts);
	    computeRxAdjustPeak(peak,-dcIMin,peakTs,ts);
	    computeRxAdjustPeak(peak,dcQMax,peakTs,ts);
	    computeRxAdjustPeak(peak,-dcQMin,peakTs,ts);
	}
    }
    int div = m_rxIO.m_buffers * m_rxIO.useSamples();
    dcIAvg /= div;
    dcQAvg /= div;
    if (debug) {
	if (m_rxShowDcInfo > 0)
	    m_rxShowDcInfo--;
	Debug(this,DebugInfo,
	    "%sRx DC values min/avg/max I=%d/%d/%d Q=%d/%d/%d peak %d TS=" FMT64U " [%p]",
	    prefix(),dcIMin,dcIAvg,dcIMax,dcQMin,dcQAvg,dcQMax,peak,peakTs,this);
    }
    if (!m_rxDcAuto)
	return;
    // DC offsets compensation feedback using an exponential moving average
    m_rxDcAvgI = ((BRF_RX_DC_OFFSET_AVG_DAMPING - 1) * m_rxDcAvgI /
	BRF_RX_DC_OFFSET_AVG_DAMPING) + dcIAvg;
    m_rxDcAvgQ = ((BRF_RX_DC_OFFSET_AVG_DAMPING - 1) * m_rxDcAvgQ /
	BRF_RX_DC_OFFSET_AVG_DAMPING) + dcQAvg;
    int corrI = m_rxDcOffsetI;
    int corrQ = m_rxDcOffsetQ;
    computeRxAdjustDC(corrI,m_rxDcAvgI,m_rxDcOffsetMax);
    computeRxAdjustDC(corrQ,m_rxDcAvgQ,m_rxDcOffsetMax);
    if (corrI == m_rxDcOffsetI && corrQ == m_rxDcOffsetQ)
	return;
    DDebug(this,DebugInfo,"%sAdjusting Rx DC offsets I=%d Q=%d [%p]",
	prefix(),corrI,corrQ,this);
    setCorrection(true,corrI,corrQ,false);
}

int BrfIface::handleRxCmds(String& s, String* rspParam, String* reason)
{
    // rx offsets auto
    // rx offsets I Q
    if (s.startSkip("offsets ",false)) {
	if (s == YSTRING("auto"))
	    m_rxDcAuto = true;
	else {
	    int corrI = 0;
	    int corrQ = 0;
	    if (!s || ::sscanf(s.c_str(),"%d %d",&corrI,&corrQ) != 2)
		return Transceiver::CmdEInvalidParam;
	    // Serialize Rx to avoid autocorrect
	    BRF_RX_SERIALIZE_ON_RET(false,false);
	    bool tmp = m_rxDcAuto;
	    m_rxDcAuto = false;
	    BRF_RX_SERIALIZE_DROP;
	    if (!setCorrection(true,corrI,corrQ,false)) {
		m_rxDcAuto = tmp;
		return Transceiver::CmdEFailure;
	    }
	}
    }
    // rx gain1
    else if (s.startSkip("gain1 ",false)) {
        int gain = s.toInteger(BLADERF_RXVGA1_GAIN_MAX + 1);
	if (!isIntervalInt(gain,BLADERF_RXVGA1_GAIN_MIN,BLADERF_RXVGA1_GAIN_MAX))
	    return Transceiver::CmdEInvalidParam;
	if (!setGainVga(true,true,gain,false,false))
	    return Transceiver::CmdEFailure;
    }
    // rx show [on|off|value]
    else if (s.startSkip("show ",false)) {
	BRF_RX_SERIALIZE_ON_RET(false,false);
	if (s == YSTRING("on"))
	    m_rxShowDcInfo = -1;
	else if (s == YSTRING("off"))
	    m_rxShowDcInfo = 0;
	else {
	    int val = s.toInteger(-1);
	    if (val < 0)
		return Transceiver::CmdEInvalidParam;
	    m_rxShowDcInfo = val;
	}
    } else if (s.startSkip("vga1",false)) {
	int vga1;
	bool ret = getVga(true,true, vga1);
	if (ret) {
	    *rspParam << "current: " << vga1 << " min: " <<
	    BLADERF_TXVGA1_GAIN_MIN << " max: " << BLADERF_TXVGA1_GAIN_MAX;
	}
	return tuneCmdReturn(ret);
    } else if (s.startSkip("vga2",false)) {
	int vga2;
	bool ret = getVga(true,false, vga2);
	if (ret) {
	    *rspParam << "current: " << vga2 << " min: " <<
	    BLADERF_TXVGA2_GAIN_MIN << " max: " << BLADERF_TXVGA2_GAIN_MAX;
	}
	return tuneCmdReturn(ret);
    }
    else
	return Transceiver::CmdEUnkCmd;
    return Transceiver::CmdEOk;
}

int BrfIface::handleTxCmds(String& s, String* rspParam, String* reason)
{
    // tx offsets I Q
    if (s.startSkip("offsets ",false)) {
	int corrI = 0;
	int corrQ = 0;
	if (!s || ::sscanf(s.c_str(),"%d %d",&corrI,&corrQ) != 2)
	    return Transceiver::CmdEInvalidParam;
	if (!setCorrection(false,corrI,corrQ,false))
	    return Transceiver::CmdEFailure;
    }
    // tx show [on|off|value]
    else if (s.startSkip("show ",false)) {
	BRF_TX_SERIALIZE_ON_RET(false,false);
	if (s == YSTRING("on"))
	    m_txShowInfo = -1;
	else if (s == YSTRING("off"))
	    m_txShowInfo = 0;
	else {
	    int val = s.toInteger(-1);
	    if (val < 0)
		return Transceiver::CmdEInvalidParam;
	    m_txShowInfo = val;
	}
    } else if (s.startSkip("sample_rate ",false)) {
	int sampleRate = s.toInteger(-1);
	if (sampleRate < 0)
	    return Transceiver::CmdEInvalidParam;
	unsigned int actual = 0;
	String what;
	return tuneCmdReturn(setSampleRate(false,sampleRate,what,&actual));
    } else if (s.startSkip("sample_rate",false)) {
	unsigned int sampleRate = 0;
	bool ret = getSampleRate(false,sampleRate);
	if (ret)
	    *rspParam << sampleRate;
	return tuneCmdReturn(ret);
    } else if (s.startSkip("bandwidth ",false)) {
	int bandwidth = s.toInteger(-1);
	Debug(DebugTest,"bandwidth is %d",bandwidth);
	if (bandwidth < 0)
	    return Transceiver::CmdEInvalidParam;
	String what;
	return tuneCmdReturn(setBandwith(false,bandwidth,what,0));
    } else if (s.startSkip("bandwidth",false)) {
	unsigned int bandwidth = 0;
	bool ret = getBandwidth(false,bandwidth);
	if (ret)
	    *rspParam << bandwidth;
	return tuneCmdReturn(ret);
    } else if (s.startSkip("lpf_mode ",false)) {
	int lpfMode = lookup(s,s_lpfMode);
	if (lpfMode < 0)
	    return Transceiver::CmdEInvalidParam;
	return tuneCmdReturn(setLpfMode(false,lpfMode));
    } else if (s.startSkip("lpf_mode",false)) {
	int lpfMode = 0;
	bool ret = getLpfMode(false,lpfMode);
	if (ret)
	    *rspParam << lookup(lpfMode,s_lpfMode);
	return tuneCmdReturn(ret);
    } else if (s.startSkip("vga1 ",false)) {
	int vga1 = s.toInteger();
	if (vga1 < BLADERF_TXVGA1_GAIN_MIN || vga1 > BLADERF_TXVGA1_GAIN_MAX)
	    return Transceiver::CmdEInvalidParam;
	return tuneCmdReturn(setGainVga(true, false, vga1, false, true) >= 0);
    } else if (s.startSkip("vga2 ",false)) {
	int vga2 = s.toInteger();
	if (vga2 < BLADERF_TXVGA2_GAIN_MIN || vga2 > BLADERF_TXVGA2_GAIN_MAX)
	    return Transceiver::CmdEInvalidParam;
	return tuneCmdReturn(setGainVga(false, false, vga2, false, true) >= 0);
    } else if (s.startSkip("vga1",false)) {
	int vga1;
	bool ret = getVga(false,true, vga1);
	if (ret) {
	    *rspParam << "current: " << vga1 << " min: " << 
		    BLADERF_TXVGA1_GAIN_MIN << " max: " << BLADERF_TXVGA1_GAIN_MAX;
	}
	return tuneCmdReturn(ret);
    } else if (s.startSkip("vga2",false)) {
	int vga2;
	bool ret = getVga(false,false, vga2);
	if (ret) {
	    *rspParam << "current: " << vga2 << " min: " << 
		    BLADERF_TXVGA2_GAIN_MIN << " max: " << BLADERF_TXVGA2_GAIN_MAX;
	}
	return tuneCmdReturn(ret);
    } else if (s.startSkip("frequency",false)) {
	unsigned int freq;
	bool ret = getFrequency(false,freq);
	if (ret)
	    *rspParam << freq;
	return tuneCmdReturn(ret);
    } else if (s.startSkip("correction ",false)) {
	int index = s.find(' ');
	if (index < 1) {
	    *rspParam << "unable to determine correction type!";
	    return Transceiver::CmdEInvalidParam;
	}
	int cType = lookup(s.substr(0,index),s_correction,-1);
	if (cType < 0) {
	    *rspParam << "unable to determine correction type!";
	    return Transceiver::CmdEInvalidParam;
	}
	s = s.substr(index + 1);
	int corr = s.toInteger(-50000);
	if (corr == -50000) {
	    *rspParam << "unable to determine correction value!";
	    return Transceiver::CmdEInvalidParam;
	}
	if (cType == BLADERF_CORR_LMS_DCOFF_I)
	    return tuneCmdReturn(setCorrection(false,true,corr));
	if (cType == BLADERF_CORR_LMS_DCOFF_Q)
	    return tuneCmdReturn(setCorrection(false,false,corr));
	return tuneCmdReturn(::bladerf_set_correction(m_dev,rxtxmod(false),
		(bladerf_correction)cType,corr) >= 0);
    } else if (s.startSkip("correction",false)) {
	int16_t i,q,phase,gain;
	::bladerf_get_correction(m_dev,rxtxmod(false),BLADERF_CORR_LMS_DCOFF_Q,&q);
	::bladerf_get_correction(m_dev,rxtxmod(false),BLADERF_CORR_LMS_DCOFF_I,&i);
	::bladerf_get_correction(m_dev,rxtxmod(false),BLADERF_CORR_FPGA_GAIN,&gain);
	::bladerf_get_correction(m_dev,rxtxmod(false),BLADERF_CORR_FPGA_PHASE,&phase);
	*rspParam << "i: " << i << " q: " << q << " gain: " << gain << " phase: " << phase;
	return tuneCmdReturn(true);
    } else if (s.startSkip("dump ",false)) {
	ObjList* split = s.split(' ',false);
	if (!split)
	    return Transceiver::CmdEInvalidParam;
	Lock myLock(m_dumpMutex);
	if (m_txDump)
	    TelEngine::destruct(m_txDump);
	m_txDump = new BladeRFDump(this);
	int count = split->count();
	while (true) {
	    String* type = static_cast<String*>((*split)[0]);
	    if (type)
		m_txDump->m_type = lookup(*type,s_dumpType);
	    if (count == 1)
		break;

	    String* samples = static_cast<String*>((*split)[1]);
	    if (samples)
		m_txDump->m_samples = samples->toInteger(1250);
	    if (count == 2)
		break;
    
	    String* file = static_cast<String*>((*split)[2]);
	    if (file)
		m_txDump->m_fileName = *file;

	    if (count == 3)
		break;

	    String* append = static_cast<String*>((*split)[3]);
	    if (append)
		m_txDump->m_fileAppend = String::boolText(*append);
	    break;
	}
	TelEngine::destruct(split);
	return tuneCmdReturn(true);
    } else if (s.startSkip("dump")) {
	*rspParam << "dump type[raw|samples|complex] samples[integer] file[file name] append[boolean]";
	return tuneCmdReturn(true);
    } else
	return Transceiver::CmdEUnkCmd;
    return Transceiver::CmdEOk;
}

/**
 * class BladeRFDump
 */
void BladeRFDump::dump(const void* data, unsigned int samples)
{
    if (m_samples <= 0) {
	m_interface->stopDump(false);
	return;
    }
    switch (m_type) {
	case Raw:
	    if ((int)samples > m_samples)
		samples = m_samples;
	    m_data.append((void*)data,samples * 4 + 16);
	    m_samples -= samples;
	    break;
	case Samples:
	{
	    if ((int)samples > m_samples)
		samples = m_samples;
	    int8_t* buf = (int8_t*)data;
	    m_data.append(buf + 16,samples * 4);
	    m_samples -= samples;
	    break;
	}
	case Complex:
	{
	    int16_t* buf = (int16_t*)data;
	    buf += 8;
	    char tmp[100];
	    for (unsigned int i = 0; i < samples * 2 && m_samples > 0;i += 2, m_samples--) {
		int len = ::sprintf(tmp,"%+d%+di, ",buf[i],buf[i + 1]);
		m_dump.append(tmp,len);
	    }
	    break;
	}
	default:
	    m_interface->stopDump(false);
	    return;
    }
    if (m_samples > 0)
	return; // Data Not Completed
    // All samples dump the data
    if (!TelEngine::null(m_fileName)) {
	File f; //(const char* name, bool canWrite = false, bool canRead = true, bool create = false, bool append = false, bool binary = false, bool pubReadable = false, bool pubWritable = false) 
	if (!f.openPath(m_fileName,true,false,true,m_fileAppend)) {
	    Debug("BladeRFDump",DebugMild,"Unable to open file '%s'",m_fileName.c_str());
	    m_interface->stopDump(false);
	    return;
	}
	if (m_type != Complex)
	    f.writeData(m_data.data(),m_data.length());
	else
	    f.writeData(m_dump.c_str(),m_dump.length());
	f.detach();
	m_interface->stopDump(false);
	return;
    }
    if (m_type != Complex)
	m_dump.hexify(m_data.data(),m_data.length(),' ');
    ::printf("BladeRFDump:\n%s\n",m_dump.c_str());
    m_interface->stopDump(false);
}

/* vi: set ts=8 sw=4 sts=4 noet: */

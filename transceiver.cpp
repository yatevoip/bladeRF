/**
 * tranceiver.cpp
 * This file is part of the Yate-BTS Project http://www.yatebts.com
 *
 * Tranceiver implementation
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
#include <stdio.h>
#include <string.h>

// Socket read/write
#ifdef XDEBUG
#define TRANSCEIVER_DEBUG_SOCKET
#else
//#define TRANSCEIVER_DEBUG_SOCKET
#endif

#ifdef XDEBUG
//#define TRANSCEIVER_DEBUGGER_FUNC
#endif
#ifdef TRANSCEIVER_DEBUGGER_FUNC
#define DBGFUNC_TRXOBJ(func,obj) \
    Debugger debugger(DebugAll,func," %s[%p]",TelEngine::c_safe(obj->prefix()),obj)
#else
#define DBGFUNC_TRXOBJ(func,obj) {}
#endif

#define FILLER_FRAMES_MIN 102
#define AVEREGE_NOISE_LENGTH 102
#define BOARD_SHIFT 4

// Dump data when entering in qmf() function
//#define TRANSCEIVER_DUMP_QMF_IN
// Dump data after applying QMF frequency shifting
//#define TRANSCEIVER_DUMP_QMF_FREQSHIFTED
// Dump QMF half band filter data
//#define TRANSCEIVER_DUMP_QMF_HALFBANDFILTER
// Dump input data on ARFCN process start / exit
//#define TRANSCEIVER_DUMP_ARFCN_PROCESS_IN
//#define TRANSCEIVER_DUMP_ARFCN_PROCESS_OUT
// Dump the RX data for testing.
//#define TRANSCEIVER_DUMP_RX_DEBUG
// Dump only the input vector and the output data for RX
//#define TRANSCEIVER_DUMP_RX_INPUT_OUTPUT
// Dump Demodulator performance
//#define TRANSCEIVER_DUMP_DEMOD_PERF

// Minimum data length for radio Rx burst to be processed by an ARFCN
#define ARFCN_RXBURST_LEN 156

namespace TelEngine {

class TrxWorker : public Thread, public GenObject
{
public:
    enum Type {
	RadioRead = 0x0001,
	TrxReadCtrl = 0x0002,
	TrxRadioIn = 0x0004,
	ARFCNData = 0x0008,
	ARFCNRx = 0x0010,
	TrxRadioOut = 0x0020,
	RadioMask = RadioRead | TrxRadioIn | TrxRadioOut | ARFCNData | ARFCNRx,
    };
    TrxWorker(unsigned int type, TransceiverObj* obj)
	: Thread(lookup(type,s_name)), m_type(type), m_obj(obj)
	{}
    ~TrxWorker()
	{ notify(); }
    virtual void cleanup()
	{ notify(); }
    static bool create(Thread*& th, int type, TransceiverObj* o);
    static void cancelThreads(TransceiverObj* o, unsigned int waitMs1, Thread** th1,
	unsigned int waitMs2 = 0, Thread** th2 = 0);
    static void softCancel(unsigned int mask = 0xffffffff);
    static const TokenDict s_name[];
    static const TokenDict s_info[];
    static Mutex s_mutex;
protected:
    virtual void run();
    void notify(bool final = true);

    static ObjList s_threads;

    unsigned int m_type;
    TransceiverObj* m_obj;
};

/** TODO NOTE 
 * The following code is for testing purpose
 */
static ObjList s_indata;
static Mutex s_inDataLoker;

class RxInData : public GenObject
{
public:
    inline RxInData(const ComplexVector& v, const GSMTime& t)
	: m_data(v), m_time(t)
    {}
    ComplexVector m_data;
    GSMTime m_time;

    static void add(const ComplexVector& v, const GSMTime& t)
    {
	Lock myLock(s_inDataLoker);
	RxInData* in = new RxInData(v,t);
	s_indata.append(in);
	if (s_indata.count() > 100) {
	    ObjList* o = s_indata.skipNull();
	    if (o)
		o->remove();
	}
    }

    static void dump(const GSMTime& t) {
	RxInData* d = 0;
	Lock myLock(s_inDataLoker);
	for (ObjList* o = s_indata.skipNull(); o;) {
	    RxInData* rx = static_cast<RxInData*>(o->get());
	    if (t > rx->m_time) {
		o->remove();
		o = o->skipNull();
		continue;
	    }
	    if (rx->m_time == t) {
		TelEngine::destruct(d);
		d = static_cast<RxInData*>(o->remove(false));
		o = o->skipNull();
		continue;
	    }
	    o = o->skipNext();
	}
	if (!d) {
	    Debug(DebugWarn,"Unable to find Input Data!!");
	    return;
	}
	String dump;
	Complex::dump(dump,d->m_data.data(),d->m_data.length());
	::printf("input-data:%s",dump.c_str());
	TelEngine::destruct(d);
    }
};

class FileDataDumper : public Thread
{
public:
    FileDataDumper(const String& fileName);
    void addData(const ComplexVector& v);
    static bool start(FileDataDumper*& ptr, const String& fileName);
    static inline void stop(FileDataDumper*& ptr) {
	    if (!ptr)
		return;
	    ptr->cancel();
	    ptr = 0;
	}
protected:
    virtual void run();
private:
    ObjList m_list;
    String m_fileName;
    Mutex m_mutex;
};

FileDataDumper::FileDataDumper(const String& fileName)
    : Thread("TrxFileDump"),
    m_fileName(fileName),
    m_mutex(false,"TrxFileDump")
{
}

void FileDataDumper::addData(const ComplexVector& v)
{
    static int s_counter = 0;
    static GSMTime t;
    s_counter++;
    if ((s_counter % 1001) != 0)
	return;
    s_counter = 0;
    Lock myLock(m_mutex);
    m_list.append(new RxInData(v,t));
}

bool FileDataDumper::start(FileDataDumper*& ptr, const String& fileName)
{
    stop(ptr);
    ptr = new FileDataDumper(fileName);
    if (!ptr->startup()) {
	delete ptr;
	ptr = 0;
    }
    return ptr != 0;
}

void FileDataDumper::run()
{
    File f;
    if (!f.openPath(m_fileName,true,false,true)) {
	Debug(DebugNote,"Unable to open dump file %s",m_fileName.c_str());
	return;
    }
    while (!Thread::check(false)) {
	Lock myLock(m_mutex);
	ObjList* o = m_list.skipNull();
	RxInData* in = o ? static_cast<RxInData*>(o->remove(false)) : 0;
	myLock.drop();
	if (!in) {
	    Thread::idle();
	    continue;
	}
	String s;
	Complex::dump(s,in->m_data.data(),in->m_data.length());
	s << "\r\n";
	f.writeData(s.c_str(),s.length());
	TelEngine::destruct(in);
    }
    Debug(DebugTest,"Data write finished!");
}

/** TODO NOTE 
 * End test code */


ObjList TrxWorker::s_threads;
Mutex TrxWorker::s_mutex(false,"TrxWorkers");
static FileDataDumper* s_dumper = 0;

const TokenDict TrxWorker::s_name[] = {
    {"ARFCNData",     ARFCNData},
    {"ARFCNRx",       ARFCNRx},
    {"TrxReadCtrl",   TrxReadCtrl},
    {"TrxRadioIn",    TrxRadioIn},
    {"TrxRadioOut",   TrxRadioOut},
    {"RadioRead",     RadioRead},
    {0,0},
};

const TokenDict TrxWorker::s_info[] = {
    {"Data socket read",       ARFCNData},
    {"Radio input process",    ARFCNRx},
    {"Control socket read",    TrxReadCtrl},
    {"Radio input process",    TrxRadioIn},
    {"Radio device send",      TrxRadioOut},
    {"Radio device read",      RadioRead},
    {0,0},
};

bool TrxWorker::create(Thread*& th, int type, TransceiverObj* o)
{
    if (!o)
	return 0;
    if (th)
	cancelThreads(o,20,&th);
    Lock lck(s_mutex);
    TrxWorker* tmp = new TrxWorker(type,o);
    if (tmp->startup()) {
	th = tmp;
	return true;
    }
    th = 0;
    lck.drop();
    delete tmp;
    Alarm(o,"system",DebugWarn,"%sFailed to start '%s' worker thread [%p]",
	(o ? o->prefix() : ""),lookup(type,s_name),o);
    return false;
}

void TrxWorker::softCancel(unsigned int mask)
{
    Lock lck(s_mutex);
    for (ObjList* o = s_threads.skipNull(); o; o = o->skipNext()) {
	TrxWorker* t = static_cast<TrxWorker*>(o->get());
	if (!t->alive())
	    continue;
	if ((t->m_type & mask) != 0)
	    t->cancel(false);
    }
}

static inline void setDebugLevel(DebugEnabler* enabler, const String* str)
{
    if (enabler && str)
	enabler->debugLevel(str->toInteger(enabler->debugLevel()));
}

// Calculate a number of thread idle intervals from a period
// Return a non 0 value
static inline unsigned int threadIdleIntervals(unsigned int intervalMs)
{
    intervalMs = (unsigned int)(intervalMs / Thread::idleMsec());
    return intervalMs ? intervalMs : 1;
}

static inline void hardCancel(Thread*& t, TransceiverObj* o, Mutex* mtx = 0)
{
    if (!t)
	return;
    Lock lck(mtx);
    if (!t)
	return;
    Debug(o,DebugWarn,"%sHard cancelling worker thread (%p) '%s' [%p]",
	(o ? o->prefix() : ""),t,t->name(),o);
    t->cancel(true);
    t = 0;
}

static inline void checkHardCancel(Thread*& t, unsigned int& wait, TransceiverObj* dbg,
    Mutex* mtx)
{
    if (!(wait && t))
	return;
    wait--;
    if (!wait)
	hardCancel(t,dbg,mtx);
}

// Cancel threads, wait for them to terminate
// waitMs: Interval to wait before hard cancelling them
void TrxWorker::cancelThreads(TransceiverObj* o, unsigned int waitMs1, Thread** th1,
    unsigned int waitMs2, Thread** th2)
{
    DBGFUNC_TRXOBJ("cancelThreads()",o);
    Lock lck(s_mutex);
    Thread* dummy = 0;
    Thread*& t1 = th1 ? *th1 : dummy;
    Thread*& t2 = th2 ? *th2 : dummy;
    if (!(t1 || t2))
	return;
    if (t1)
	t1->cancel(false);
    if (t2)
	t2->cancel(false);
    if (waitMs1)
	waitMs1 = threadIdleIntervals(waitMs1);
    if (waitMs2)
	waitMs2 = threadIdleIntervals(waitMs2);
    bool stop = false;
    while ((t1 || t2) && !stop) {
	lck.drop();
	Thread::idle();
	stop = Thread::check(false);
	if (!stop && (waitMs1 || waitMs2)) {
	    checkHardCancel(t1,waitMs1,o,&s_mutex);
	    checkHardCancel(t2,waitMs2,o,&s_mutex);
	}
	lck.acquire(s_mutex);
    }
    hardCancel(t1,o);
    hardCancel(t2,o);
}

void TrxWorker::run()
{
    if (!m_obj)
	return;
    s_mutex.lock();
    s_threads.append(this)->setDelete(false);
    s_mutex.unlock();
    DDebug(m_obj,DebugAll,"%s%s thread (%p) started [%p]",
	m_obj->prefix(),lookup(m_type,s_info),this,m_obj);
    switch (m_type) {
	case ARFCNData:
	    (static_cast<ARFCNSocket*>(m_obj))->runReadDataSocket();
	    break;
	case ARFCNRx:
	    (static_cast<ARFCN*>(m_obj))->runRadioDataProcess();
	    break;
	case TrxReadCtrl:
	    (static_cast<Transceiver*>(m_obj))->runReadCtrlSocket();
	    break;
	case TrxRadioIn:
	    (static_cast<Transceiver*>(m_obj))->runRadioDataProcess();
	    break;
	case TrxRadioOut:
	    (static_cast<Transceiver*>(m_obj))->runRadioSendData();
	    break;
	case RadioRead:
	    (static_cast<RadioIface*>(m_obj))->runReadRadio();
	    break;
	default:
	    Debug(m_obj,DebugStub,"TrxWorker::run() type=%d not handled",m_type);
    }
    notify(false);
}

void TrxWorker::notify(bool final)
{
    s_mutex.lock();
    s_threads.remove(this,false);
    s_mutex.unlock();
    TransceiverObj* obj = m_obj;
    m_obj = 0;
    if (!obj)
	return;
    const char* s = lookup(m_type,s_info);
    if (!final)
	Debug(obj,DebugAll,"%s%s thread (%p) terminated [%p]",obj->prefix(),s,this,obj);
    else {
	Alarm(obj,"system",DebugWarn,"%s%s thread (%p) abnormally terminated [%p]",
	    obj->prefix(),s,this,obj);
	if (obj->transceiver())
	    obj->transceiver()->fatalError();
    }
    obj->workerTerminated(this);
}

}; // namespace TelEngine

using namespace TelEngine;

enum Command
{
    CmdUnknown = 0,
    CmdHandover,
    CmdNoHandover,
    CmdSetSlot,
    CmdSetPower,
    CmdAdjPower,
    CmdSetTxAtten,
    CmdRxTune,
    CmdTxTune,
    CmdReadFactory,
    CmdSetTsc,
    CmdSetMaxDelay,
    CmdSetRxGain,
    CmdPowerOn,
    CmdPowerOff,
    CmdCustom,
    CmdManageTx,
    CmdNoise,
    CmdFreqOffset
};

static const TokenDict s_cmdName[] = {
    {"HANDOVER",    CmdHandover},
    {"NOHANDOVER",  CmdNoHandover},
    {"SETSLOT",     CmdSetSlot},
    {"SETPOWER",    CmdSetPower},
    {"ADJPOWER",    CmdAdjPower},
    {"SETTXATTEN",  CmdSetTxAtten},
    {"RXTUNE",      CmdRxTune},
    {"TXTUNE",      CmdTxTune},
    {"READFACTORY", CmdReadFactory},
    {"SETTSC",      CmdSetTsc},
    {"SETMAXDLY",   CmdSetMaxDelay},
    {"SETRXGAIN",   CmdSetRxGain},
    {"POWERON",     CmdPowerOn},
    {"POWEROFF",    CmdPowerOff},
    {"CUSTOM",      CmdCustom},
    {"MANAGETX",    CmdManageTx},
    {"NOISELEV",    CmdNoise},
    {"SETFREQOFFSET",CmdFreqOffset},
    {0,0}
};

static const TokenDict s_cmdErrorName[] = {
    {"Unknown command",                Transceiver::CmdEUnkCmd},
    {"Failure",                        Transceiver::CmdEFailure},
    {"Invalid/missing parameter(s)",   Transceiver::CmdEInvalidParam},
    {"Invalid state",                  Transceiver::CmdEInvalidState},
    {"Invalid ARFCN",                  Transceiver::CmdEInvalidARFCN},
    {0,0}
};

static const TokenDict s_stateName[] = {
    {"Invalid",  Transceiver::Invalid},
    {"Idle",     Transceiver::Idle},
    {"PowerOff", Transceiver::PowerOff},
    {"PowerOn",  Transceiver::PowerOn},
    {0,0}
};

static const TokenDict s_gsmSlotType[] = {
    {"I",      ARFCN::ChanI},
    {"II",     ARFCN::ChanII},
    {"III",    ARFCN::ChanIII},
    {"IV",     ARFCN::ChanIV},
    {"V",      ARFCN::ChanV},
    {"VI",     ARFCN::ChanVI},
    {"VII",    ARFCN::ChanVII},
    {"Fill",   ARFCN::ChanFill},
    {"None",   ARFCN::ChanNone},
    {"IGPRS",  ARFCN::ChanIGPRS},
    {0,0}
};

static const TokenDict s_gsmBurstType[] = {
    {"Normal",   ARFCN::BurstNormal},
    {"Access",   ARFCN::BurstAccess},
    {"Idle",     ARFCN::BurstIdle},
    {"None",     ARFCN::BurstNone},
    {"Unknown",  ARFCN::BurstUnknown},
    {"Check",    ARFCN::BurstCheck},
    {0,0}
};

struct QMFBlockDesc
{
    const char* name;
    unsigned int arfcn;
    float freqShiftValue;
};

static const QMFBlockDesc s_qmfBlock4[15] = {
    {"Q",    0xffffffff, (float)(PI / 2)},
    {"QL",   0xffffffff, (float)(0.749149017394489)},
    {"QH",   0xffffffff, (float)(2.3924436361953)},
    {"QLL",  0xffffffff, (float)(-0.821647309400407)},
    {"QLH",  0xffffffff, (float)(0.821647309400407)},
    {"QHL",  0xffffffff, (float)(-0.821647309400407)},
    {"QHH",  0xffffffff, (float)(0.821647309400407)},
    {"QLLL", 0,          (float)(-PI / 4)},
    {"QLLH", 0xffffffff, 0},
    {"QLHL", 1,          (float)(-PI / 4)},
    {"QLHH", 0xffffffff, 0},
    {"QHLL", 2,          (float)(-PI / 4)},
    {"QHLH", 0xffffffff, 0},
    {"QHHL", 3,          (float)(-PI / 4)},
    {"QHHH", 0xffffffff, 0}
};

static inline const char* trxStateName(int stat)
{
    return lookup(stat,Transceiver::dictStateName());
}

static inline bool trxShouldExit(Transceiver* trx)
{
    return trx && trx->shouldStop();
}

static inline bool thShouldExit(Transceiver* trx)
{
    return Thread::check(false) || trxShouldExit(trx);
}

static inline unsigned int getUInt(const NamedList& p, const String& param,
    unsigned int defVal = 1, unsigned int minVal = 1,
    unsigned int maxVal = (unsigned int)INT_MAX)
{
    return (unsigned int)p.getIntValue(param,defVal,minVal,maxVal);
}

static inline bool setReason(String* dest, const char* src, bool ok = false)
{
    if (dest)
	*dest = src;
    return ok;
}

static inline int getCommand(String& buf, String& cmd)
{
    int c = buf.find(" ");
    if (c >= 0) {
	cmd = buf.substr(0,c);
	buf = buf.substr(c + 1);
    }
    else {
	cmd = buf;
	buf.clear();
    }
    return lookup(cmd,s_cmdName);
}

static inline bool buildCmdRsp(String* rsp, const char* cmd, int status,
    const char* param = 0)
{
    if (rsp) {
	*rsp << "RSP " << cmd << " " << status;
	rsp->append(param," ");
    }
    return status == 0;
}


//
// GenQueue
//
GenQueue::GenQueue(unsigned int maxLen, const char* name)
    : m_name(name),
    m_mutex(false,m_name.safe("GenQueue")),
    m_free(maxLen,"GenQueueFree"),
    m_used(maxLen,"GenQueueUsed"),
    m_empty(true),
    m_count(0),
    m_max(maxLen)
{
    clear();
}

bool GenQueue::add(GenObject* obj, Transceiver* trx)
{
    if (!obj)
	return false;
    m_mutex.lock();
    if (m_max <= m_count) {
	m_mutex.unlock();
	while (!m_free.lock(Thread::idleUsec()))
	    if (thShouldExit(trx))
		return false;
	m_mutex.lock();
    }
    m_data.append(obj);
    m_count++;
    m_empty = false;
    m_mutex.unlock();
    if (m_max)
	m_used.unlock();
    return true;
}

// Wait for an object to be put in the queue.
// Returns when there is something in the queue or the calling thread was
//  cancelled or the given transceiver is exiting
bool GenQueue::waitPop(GenObject*& obj, Transceiver* trx)
{
    if (m_max)
	while (true) {
	    obj = internalPop();
	    if (obj) {
		m_free.unlock();
		return true;
	    }
	    if (m_used.lock(Thread::idleUsec())) {
		obj = internalPop();
		if (obj) {
		    m_free.unlock();
		    return true;
		}
	    }
	    if (thShouldExit(trx))
		return false;
	}
    else
	while (true) {
	    if (!empty()) {
		obj = internalPop();
		if (obj)
		    return true;
	    }
	    Thread::idle();
	    if (thShouldExit(trx))
		return false;
	}
    return false;
}

// Clear the queue
void GenQueue::clear()
{
    m_mutex.lock();
    m_empty = true;
    m_count = 0;
    m_data.clear();
    m_mutex.unlock();
    if (!m_max)
	return;
    // Adjust semaphores
    while (m_used.lock(0))
	;
    for (unsigned int n = m_max + 1; n; n--)
	m_free.unlock();
}

GenObject* GenQueue::internalPop()
{
    Lock lck(m_mutex);
    GenObject* ret = m_data.remove(false);
    if (!ret)
	return 0;
    m_count--;
    if (!m_count)
	m_empty = true;
    return ret;
}


//
// TransceiverObj
//
// Set the transceiver
void TransceiverObj::setTransceiver(Transceiver& t, const char* name)
{
    m_transceiver = &t;
    m_prefix.clear();
    bool noTrx = ((&t) != this);
    if (noTrx) {
	m_name = t.debugName();
	m_prefix << name << ": ";
    }
    else
        m_name = name;
    debugName(m_name);
}

// Dump to output a received burst
void TransceiverObj::dumpRecvBurst(const char* str, const GSMTime& time,
    const Complex* c, unsigned int len)
{
    if (!debugAt(DebugAll))
	return;
    String tmp;
    Complex::dump(tmp,c,len);
    Debug(this,DebugAll,"%s%s TN=%u FN=%u len=%u:%s [%p]",
	    prefix(),TelEngine::c_safe(str),time.tn(),time.fn(),len,
	    tmp.safe(),this);
}

void demodulatorTestPoint(const String& name, const ComplexVector& out, int arfcn, int start = -1, int len = -1)
{
#ifdef TRANSCEIVER_DUMP_RX_DEBUG
    if (start == -1)
	start = 0;
    int end = start;
    if (len == -1 || len > (int)out.length())
	end += out.length();
    else
	end += len;
    String dump;
    for (int i = start;i < end;i++)
	out[i].dump(dump);
    ::printf("\n%s:%d:%s\n",name.c_str(),arfcn,dump.c_str());
#endif
}


void TransceiverObj::dumpRxData(const char* prefix, unsigned int index, const char* postfix,
	const Complex* c, unsigned int len)
{
    if (!debugAt(DebugAll))
	return;
#ifdef TRANSCEIVER_DUMP_RX_DEBUG
    String tmp;
    Complex::dump(tmp,c,len);
    ::printf("\n%s%d%s:%s\n",prefix,index,postfix,tmp.safe());
#endif
}

void TransceiverObj::dumpRxData(const char* prefix, unsigned int index, const char* postfix,
	const float* f, unsigned int len)
{
    if (!debugAt(DebugAll))
	return;
#ifdef TRANSCEIVER_DUMP_RX_DEBUG
    String tmp;
    FloatVector::dump(tmp,f,len,SigProcUtils::appendFloat);
    ::printf("\n%s%d%s:%s\n",prefix,index,postfix,tmp.safe());
#endif
}


//
// TransceiverSockIface
//
// Worker terminated notification
void TransceiverObj::workerTerminated(Thread* th)
{
    Debug(this,DebugStub,"TransceiverObj::workerTerminated() [%p]",this);
}


//
// TransceiverSockIface
//
// Set a socket address
bool TransceiverSockIface::setAddr(bool local, const char* addr, int port,
    TransceiverObj& dbg)
{
    String reason;
    SocketAddr& sa = local ? m_local : m_remote;
    int family = SocketAddr::IPv4;
    if (sa.assign(family)) {
	if ((!addr || sa.host(addr)) && (local || !sa.isNullAddr())) {
	    sa.port(port);
	    DDebug(&dbg,DebugAll,"%sInitialized %s %s address: '%s' [%p]",
		dbg.prefix(),name().c_str(),(local ? "local" : "remote"),
		sa.addr().c_str(),&dbg);
	    return true;
	}
	reason = "invalid address";
    }
    else
	reason << "failed to assign family " << family <<
	    " (" << SocketAddr::lookupFamily(family) << ")";
    Alarm(&dbg,"config",DebugNote,
	"%sFailed to initialize %s %s address (addr='%s' port=%d): %s [%p]",
	dbg.prefix(),name().c_str(),(local ? "local" : "remote"),
	addr,port,reason.c_str(),&dbg);
    return false;
}

// Prepare the socket: create, set reuse, bind, set unblocking mode
// Don't terminate the socket on failure
bool TransceiverSockIface::initSocket(TransceiverObj& dbg)
{
    terminate();
    const char* error = 0;
    while (true) {
	if (!m_socket.create(m_local.family(),SOCK_DGRAM,IPPROTO_UDP)) {
	    error = "create failed";
	    break;
	}
	m_socket.setReuse();
	if (!m_socket.bind(m_local)) {
	    error = "failed to bind";
	    break;
	}
	if (!m_socket.setBlocking(false)) {
	    error = "failed to set non blocking mode";
	    break;
	}
	break;
    }
    if (!error) {
	Debug(&dbg,DebugInfo,"%sSocket(%s) bound on %s [%p]",
	    dbg.prefix(),name().c_str(),m_local.addr().c_str(),&dbg);
	return true;
    }
    String s;
    Thread::errorString(s,m_socket.error());
    Alarm(&dbg,"socket",DebugWarn,"%sSocket(%s) %s (addr=%s): %d %s [%p]",
	dbg.prefix(),name().c_str(),error,m_local.addr().c_str(),
	m_socket.error(),s.c_str(),&dbg);
    return false;
}

// Read from socket. Call Thread::idle() if nothing is read
int TransceiverSockIface::readSocket(TransceiverObj& dbg)
{
    const char* error = 0;
    bool sel = m_socket.canSelect();
    if (sel) {
	bool canRead = false;
	bool ok = m_socket.select(&canRead,0,0,Thread::idleUsec());
	if (thShouldExit(dbg.transceiver()))
	    return 0;
	if (ok) {
	    if (!canRead)
		return 0;
	}
	else if (m_socket.canRetry()) {
	    Thread::idle();
	    return 0;
	}
	else
	    error = "select";
    }
    if (!error) {
	int r = m_socket.recvFrom((void*)m_readBuffer.data(),m_readBuffer.length());
	if (r > 0) {
#ifdef TRANSCEIVER_DEBUG_SOCKET
	    String tmp;
	    if (m_text)
		tmp.assign((const char*)m_readBuffer.data(),m_readBuffer.length());
	    else
		tmp.hexify((void*)m_readBuffer.data(),r,' ');
	    Debug(&dbg,DebugAll,"%sSocket(%s) read %d bytes: %s [%p]",
		dbg.prefix(),name().c_str(),r,tmp.c_str(),&dbg);
#endif
	    return r;
	}
	if (!r || m_socket.canRetry()) {
	    if (!sel)
		Thread::idle();
	    return 0;
	}
	error = "read";
    }
    String s;
    Thread::errorString(s,m_socket.error());
    Alarm(&dbg,"socket",DebugWarn,"%sSocket(%s) %s failed: %d %s [%p]",
	dbg.prefix(),name().c_str(),error,m_socket.error(),s.c_str(),&dbg);
    return -1;
}

// Write to socket
int TransceiverSockIface::writeSocket(const void* buf, unsigned int len,
    TransceiverObj& dbg)
{
    if (!(buf && len))
	return 0;

    // NOTE: Handle partial write for stream sockets
    for (unsigned int i = 0; i < m_writeAttempts; i++) {
	int r = m_socket.sendTo(buf,len,m_remote);
	if (r > 0) {
#ifdef TRANSCEIVER_DEBUG_SOCKET
	    bool dump = true;
#else
	    bool dump = m_printOne;
#endif
	    if (dump) {
		m_printOne = false;
		String tmp;
		if (m_text)
		    tmp.assign((const char*)buf,len);
		else
		    tmp.hexify((void*)buf,len,' ');
		Debug(&dbg,DebugAll,"%sSocket(%s) sent %u bytes: %s [%p]",
		    dbg.prefix(),name().c_str(),len,tmp.c_str(),&dbg);
	    }
	    return len;
	}
	if (thShouldExit(dbg.transceiver()))
	    return 0;
	if (m_socket.canRetry() && i < (m_writeAttempts - 1)) {
	    Thread::idle();
	    continue;
	}
	break;
    }
    String s;
    Thread::errorString(s,m_socket.error());
    Alarm(&dbg,"socket",DebugWarn,"%sSocket(%s) send failed: %d %s [%p]",
	dbg.prefix(),name().c_str(),m_socket.error(),s.c_str(),&dbg);
    return m_socket.canRetry() ? 0 : -1;
}

//
// Transceiver
//
Transceiver::Transceiver(const char* name)
    : m_state(Invalid),
    m_stateMutex(false,"TrxState"),
    m_ctrlReadThread(0),
    m_radio(0),
    m_radioInThread(0),
    m_radioOutThread(0),
    m_rxQueue(24,"TrxRxQueue"),
    m_oversamplingRate(1),
    m_burstMinPower(0),
    m_snrThreshold(2),
    m_wrtFullScaleThreshold(-2),
    m_peakOverMeanThreshold(3),
    m_upPowerThreshold(-2),
    m_maxPropDelay(63),
    m_arfcn(0),
    m_arfcnCount(0),
    m_arfcnConf(0),
    m_clockIface("clock"),
    m_startTime(0),
    m_clockUpdMutex(false,"TrxClockUpd"),
    m_txLatency(2),
    m_tsc(0),
    m_rxFreq(0),
    m_txFreq(0),
    m_freqOffset(0),
    m_txPower(-10),
    m_txAttnOffset(0),
    m_sendArfcnFS(0),
    m_txTestBurst(0),
    m_testMutex(false,"txTest"),
    m_dumpOneTx(false),
    m_dumpOneRx(false),
    m_toaShift(0),
    m_error(false),
    m_exiting(false),
    m_shutdown(false)
{
    setTransceiver(*this,name);
    DDebug(this,DebugAll,"Transceiver() [%p]",this);
}

Transceiver::~Transceiver()
{
    DDebug(this,DebugAll,"~Transceiver() [%p]",this);
    stop();
    // NOTE: If you need to use m_arfcnCount use it before this line!
    // resetARFCNs will set it to 0.
    resetARFCNs();
    TelEngine::destruct(m_radio);
    TelEngine::destruct(m_txTestBurst);
}

// Initialize the transceiver. This method should be called after construction
bool Transceiver::init(RadioIface* radio, const NamedList& params)
{
    if (m_state != Invalid) {
	reInit(params);
	return false;
    }
    m_error = false;
    m_radio = radio;
    if (!m_radio) {
	Debug(this,DebugFail,"No radio interface [%p]",this);
	return false;
    }
    m_radio->setTransceiver(*this,"RADIO");
    m_radio->debugChain(this);
    reInit(params);
    bool ok = false;
    while (true) {
	m_oversamplingRate = getUInt(params,"oversampling");
	unsigned int arfcns = getUInt(params,"arfcns");
	m_arfcnConf = getUInt(params,"conf_arfcns");
	const String* rAddr = params.getParam(YSTRING("remoteaddr"));
	unsigned int nFillers = params.getIntValue(YSTRING("filler_frames"),
	    FILLER_FRAMES_MIN,FILLER_FRAMES_MIN);
	m_signalProcessing.initialize(m_oversamplingRate,arfcns);
	int port = rAddr ? params.getIntValue(YSTRING("port")) : 0;
	const char* lAddr = rAddr ? params.getValue(YSTRING("localaddr"),*rAddr) : 0;
	if (rAddr &&
	    !(m_clockIface.setAddr(true,lAddr,port,*this) &&
	    m_clockIface.setAddr(false,*rAddr,port + 100,*this)))
	    break;
	if (!resetARFCNs(arfcns,port,rAddr,lAddr,nFillers))
	    break;
	if (debugAt(DebugAll)) {
	    String tmp;
	    tmp << "\r\nARFCNs=" << m_arfcnCount;
	    tmp << "\r\noversampling=" << m_oversamplingRate;
	    Debug(this,DebugAll,"Initialized [%p]\r\n-----%s\r\n-----",this,tmp.c_str());
	}
	NamedList rParams(params);
	rParams.setParam("samples_per_symbol",String(m_oversamplingRate));
	if (!m_radio->init(rParams))
	    break;
	ok = true;
	// TODO test
	String* dumpFile = params.getParam(YSTRING("dump_file"));
	if (dumpFile)
	    FileDataDumper::start(s_dumper,*dumpFile);
	// TODO end test
	break;
    }
    if (!ok) {
	TelEngine::destruct(m_radio);
	return false;
    }
    m_nextClockUpdTime = m_startTime;
    m_txTime = m_startTime;
    changeState(Idle);
    return true;
}

// Re-initialize the transceiver
void Transceiver::reInit(const NamedList& params)
{
    m_txAttnOffset = params.getIntValue(YSTRING("TxAttenOffset"),0,0,100);
    m_freqOffset = params.getIntValue(YSTRING("RadioFrequencyOffset"),128,64,192);
    setDebugLevel(this,params.getParam(YSTRING("debug_level")));
    setDebugLevel(m_radio,params.getParam(YSTRING("debug_radio_level")));

    m_snrThreshold = params.getIntValue(YSTRING("snr_threshold"),m_snrThreshold);
    m_wrtFullScaleThreshold = params.getIntValue(YSTRING("wrt_full_scale"),m_wrtFullScaleThreshold);
    m_peakOverMeanThreshold = params.getIntValue(YSTRING("peak_over_mean"),m_peakOverMeanThreshold);
    m_maxPropDelay = params.getIntValue(YSTRING("max_prop_delay"),m_maxPropDelay);
    setBurstMinPower(params.getIntValue(YSTRING("burst_min_power"),-50));
    m_upPowerThreshold = params.getIntValue(YSTRING("up_power_warn"),m_upPowerThreshold);
}

// Wait for PowerOn state
void Transceiver::waitPowerOn()
{
    while (m_state < PowerOn && !thShouldExit(this))
	Thread::idle();
}

// Process received radio data
void Transceiver::recvRadioData(RadioRxData* d)
{
    if (!d)
	return;
    XDebug(this,DebugAll,"Enqueueing radio data (%p) TN=%u FN=%u len=%u [%p]",
	d,d->m_time.tn(),d->m_time.fn(),d->m_data.length(),this);

    if (m_dumpOneRx) { // For TEST purposes
	m_dumpOneRx = false;
	String x;
	for (unsigned int i = 0;i < d->m_data.length();i++) {
	    d->m_data[i].dump(x);
	    x << " ";
	}
	printf("\n%s\n",x.c_str());
    }

#ifdef TRANSCEIVER_DUMP_RX_DEBUG
    if (debugAt(DebugAll)) {
	String dumprx;
	for (unsigned int i = 0;i < d->m_data.length();i++)
	    d->m_data[i].dump(dumprx);
	::printf("\ninput-data:%s\n",dumprx.c_str());
    }
#endif
    if (m_rxQueue.add(d,this))
	return;
    if (!thShouldExit(this))
	Debug(this,DebugWarn,"Dropping radio data (%p) TN=%u FN=%u: queue full [%p]",
	    d,d->m_time.tn(),d->m_time.fn(),this);
    TelEngine::destruct(d);
}

void Transceiver::runRadioDataProcess()
{
    waitPowerOn();
    GenObject* gen = 0;
    while (m_rxQueue.waitPop(gen,this)) {
	if (!gen)
	    continue;
	processRadioData(static_cast<RadioRxData*>(gen));
	gen = 0;
    }
    TelEngine::destruct(gen);
}

#ifndef CALLGRIND_CHECK
void Transceiver::runRadioSendData()
{
    if (!m_radio)
	return;
    waitPowerOn();
    GSMTime latencyLastUpdate(m_txTime);
    GSMTime radioTime;
    GSMTime endRadioTime;
    GSMTime txBuf = m_radio->getTxBufSpace();
    bool wait = true;
    while (true) {
	if (wait) {
	    if (!m_radio->waitSendTx())
		break;
	    m_radio->getRadioClock(radioTime);
	}
	else {
	    if (thShouldExit(this))
		break;
	    wait = true;
	}
	GSMTime rTime = radioTime;
	rTime.advance(m_txLatency.fn(),m_txLatency.tn());
	if (m_txTime > rTime)
	    continue;
	// TODO Adjust Latency
	// The underrun may be unreliable here.
	// Calculate the latency after the data is actualy sent?
	GSMTime txTime = m_txTime;
	GSMTime maxSendTime = m_txTime;
	maxSendTime.advance(txBuf.fn(),txBuf.tn());
	if (rTime > maxSendTime) {
	    GSMTime diff;
	    GSMTime::diff(diff,rTime,maxSendTime);
	    Debug(this,DebugMild,
		"Oversleep in runRadioSendData() with fn %u tn %u, adjusting! (rtime=(%u,%u) max=(%u,%u))",
		diff.fn(),diff.tn(),rTime.fn(),rTime.tn(),maxSendTime.fn(),maxSendTime.tn());
	    //txTime.advance(diff.fn(),diff.tn());
	}
	while (rTime > txTime) {
	    if (sendBurst(txTime)) {
		if (m_radio->loopback())
		    m_radio->getRadioClock(txTime);
		else
		    txTime.incTn();
		continue;
	    }
	    fatalError();
	    break;
	}
	if (thShouldExit(this))
	    break;
	Lock lck(m_clockUpdMutex);
	m_txTime = txTime;
	bool upd = m_txTime > m_nextClockUpdTime;
	lck.drop();
	if (upd && !syncGSMTime())
	    break;
	m_radio->getRadioClock(endRadioTime);
	wait = (radioTime == endRadioTime);
	if (!wait)
	    radioTime = endRadioTime;
    }
}

#else

void Transceiver::runRadioSendData()
{
    if (!m_radio)
	return;
    waitPowerOn();
    m_radio->waitSendTx();
    m_radio->getRadioClock(m_txTime);
    for (unsigned int i = 0; i < 102;i++) {
	sendBurst(m_txTime);
	m_txTime.incTn();
    }
    m_radio->getRadioClock(m_txTime);
    GSMTime tmpTime;
    while (true) {
	if (thShouldExit(this))
	    break;
	if (m_txTime > m_nextClockUpdTime && !syncGSMTime())
	    break;

	if (sendBurst(m_txTime)) {
	    if (m_radio->loopback()) {
		m_radio->getRadioClock(m_txTime);
		continue;
	    }
	    m_txTime.incTn();
	    if (m_txTime.tn() != 9)
		continue;
	    int fnoff = GSMTime::fnOffset(m_txTime.fn(),tmpTime.fn());
	    m_radio->getRadioClock(tmpTime);
	    if (fnoff >= 3) {
		m_txTime.decFn();
		Debug(this,DebugMild,"Big Difference between Rx and Tx decFN (%d %d) (%d %d) fnoff %d",tmpTime.fn(),tmpTime.tn(),m_txTime.fn(),m_txTime.tn(),fnoff);
	    } else if (fnoff < 1) {
		m_txTime.incFn();
		Debug(this,DebugMild,"Big Difference between Rx and Tx incFN (%d %d) (%d %d) fnoff %d",tmpTime.fn(),tmpTime.tn(),m_txTime.fn(),m_txTime.tn(),fnoff);
	    }
	    continue;
	}
	fatalError();
	break;
    }
}
#endif

bool shouldBeSync(GSMTime& time)
{
    if (time.tn() != 0)
	return false;
    int t3 = time.fn() % 51;
    return t3 == 1 || t3 == 11 || t3 == 21 || t3 == 31 || t3 == 41;
}

void Transceiver::checkType(const GSMTxBurst& burst, const GSMTime& time, bool isFiller)
{
    int t3 = time.fn() % 51;
    switch (t3) {
	case 0:
	case 10:
	case 20:
	case 30:
	case 40:
	    if (burst.type() == 2)
		break;
	    Debug(this,DebugNote,"Sending Burst type %d on FCCH t3=%d FN=%d TN=%d burst FN=%d TN=%d filler=%u",burst.type(),t3,time.fn(),time.tn(),burst.time().fn(),burst.time().tn(),isFiller);
	    break;
	case 1:
	case 11:
	case 21:
	case 31:
	case 41:
	    if (burst.type() == 1)
		break;
	    Debug(this,DebugNote,"Sending Burst type %d on SCH t3 %d FN=%d TN=%d burst FN=%d TN=%d filler=%u",burst.type(),t3,time.fn(),time.tn(),burst.time().fn(),burst.time().tn(),isFiller);
	    break;
	default:
	    if (burst.type() != 0)
		Debug(this,DebugNote,"Sending Burst type %d on unknow channel t3 %d FN=%d TN=%d burst FN=%d TN=%d filler=%u",burst.type(),t3,time.fn(),time.tn(),burst.time().fn(),burst.time().tn(),isFiller);
	    break;
    }
}

bool Transceiver::sendBurst(GSMTime& time)
{
    int t3 = time.fn() % 51;
    int t2 = time.fn() % 26;
#ifdef DEBUG
    if (t3 == 0 && time.tn() == 0)
	Debug(this,DebugAll,"Sending Time FN=%d",time.fn());
#endif

    XDebug(this,DebugAll,"sendBurst() FN=%u TN=%u T2 %d T3 %d[%p]",time.fn(),time.tn(),t2,t3,this);
    // Build send data
    bool first = true;
    for (unsigned int i = 0; i < m_arfcnCount; i++) {
	ARFCN* a = m_arfcn[i];
	a->moveBurstsToFillers(time,m_radio && !m_radio->loopback());
	// Get the burst to send
	bool addFiller = true;
	bool sync = (i == 0) && shouldBeSync(time);
	bool delBurst = false;
	GSMTxBurst* burst = a->m_shaper.getShaped(time);
	if (!burst)
	    burst = a->getBurst(time);
	else
	    delBurst = true;
	if (burst && a->m_slots[time.tn()].type == ARFCN::ChanIGPRS)
	    delBurst = true;
	if (!burst) {
	    if (sync) {
		m_sendBurstBuf.resize(m_sendBurstBuf.length());
		Debug(this,DebugNote,"Requested to get SYNC burst from filler table! ARFCN %d FN %d TN %d T2 %d T3 %d",a->arfcn(),time.fn(),time.tn(),t2,t3);
		continue;
	    }
	    burst = a->m_fillerTable.get(time);
	    addFiller = false;
	}
#ifdef CALLGRIND_CHECK
	if (!addFiller) {
	    addFiller = true;
	    burst = GSMTxBurst::buildFiller();
	    burst->buildTxData(m_signalProcessing);
	}
#endif
	if (!burst)
	    continue;
	if (time.tn() == 0 && i == 0 && burst->type() != 0)
	    checkType(*burst,time,!addFiller);
	if (first) {
	    if (m_sendBurstBuf.length() != burst->txData().length())
		m_sendBurstBuf.resize(burst->txData().length());
	    m_sendBurstBuf.copy(burst->txData());
	    Complex::multiply(m_sendBurstBuf.data(),m_sendBurstBuf.length(),m_signalProcessing.arfcnFS(i)->data(),m_signalProcessing.arfcnFS(i)->length());
	    first = false;
	} else
	    Complex::sumMul(m_sendBurstBuf.data(),m_sendBurstBuf.length(),burst->txData().data(),burst->txData().length(),m_signalProcessing.arfcnFS(i)->data(),m_signalProcessing.arfcnFS(i)->length());

	if (delBurst) {
	    TelEngine::destruct(burst);
	    continue;
	}
	if (addFiller)
	    a->m_fillerTable.set(burst);
    }
    // Reset TX buffer if nothing was set there
    if (first)
	m_sendBurstBuf.resize(m_signalProcessing.gsmSlotLen(),true);
    // Send frequency shifting vectors only?
    if (m_sendArfcnFS)
	m_signalProcessing.sumFreqShift(m_sendBurstBuf,m_sendArfcnFS);
    if (m_dumpOneTx) {
	m_dumpOneTx = false;
	String d;
	m_sendBurstBuf.dump(d,SigProcUtils::appendComplex);
	::printf("TX %u at fn=%u tn=%u:\r\n%s\r\n",
	    m_sendBurstBuf.length(),time.fn(),time.tn(),d.c_str());
    }
    return m_radio->sendData(m_sendBurstBuf,time);
}

// Process a received radio burst
bool Transceiver::processRadioBurst(unsigned int arfcn, ArfcnSlot& slot, GSMRxBurst& b)
{
    Debug(this,DebugStub,"Transceiver::processRadioBurst() not implemented");
    return false;
}

// Start the transceiver, stop it if already started
bool Transceiver::start()
{
    DBGFUNC_TRXOBJ("Transceiver::start()",this);
    Lock lck(m_stateMutex);
    if (!m_radio || m_state == Invalid)
	return false;
    if (m_state > Idle)
	return true;
    Debug(this,DebugAll,"Starting [%p]",this);
    while (true) {
	if (!m_clockIface.initSocket(*this))
	    break;
	unsigned int n = 0;
	for (; n < m_arfcnCount; n++)
	    if (!m_arfcn[n]->start())
		break;
	if (n < m_arfcnCount)
	    break;
	if (m_arfcnCount && m_arfcn[0]->getObject(YATOM("ARFCNSocket")) &&
	    !TrxWorker::create(m_ctrlReadThread,TrxWorker::TrxReadCtrl,this))
	    break;
	changeState(PowerOff);
	return true;
    }
    lck.drop();
    stop();
    return false;
}

// Stop the transceiver
void Transceiver::stop()
{
    DBGFUNC_TRXOBJ("Transceiver::stop()",this);
    TrxWorker::softCancel();
    m_stateMutex.lock();
    if (m_exiting) {
	m_stateMutex.unlock();
	return;
    }
    m_exiting = true;
    if (m_state > Idle) {
	Debug(this,DebugAll,"Stopping [%p]",this);
	changeState(Idle);
    }
    TrxWorker::cancelThreads(this,0,&m_ctrlReadThread,0,&m_radioInThread);
    m_stateMutex.unlock();
    radioPowerOff();
    m_stateMutex.lock();
    stopARFCNs();
    m_exiting = false;
    m_stateMutex.unlock();
}

// Execute a command
bool Transceiver::command(const char* str, String* rsp, unsigned int arfcn)
{
    String s = str;
    if (!s)
	return false;
    if (!s.startSkip("CMD ",false)) {
	Debug(this,DebugNote,"Invalid command '%s' [%p]",str,this);
	syncGSMTime();
	return false;
    }
    Debug(this,DebugAll,"Handling command '%s' arfcn=%u [%p]",str,arfcn,this);
    String cmd;
    String reason;
    String rspParam;
    int status = CmdEOk;
    int c = getCommand(s,cmd);
    switch (c) {
	case CmdHandover:
	case CmdNoHandover:
	    status = handleCmdHandover(c == CmdHandover,arfcn,s,&rspParam);
	    break;
	case CmdSetSlot:
	    status = handleCmdSetSlot(arfcn,s,&rspParam);
	    break;
	case CmdSetPower:
	    status = handleCmdSetPower(arfcn,s,&rspParam);
	    break;
	case CmdAdjPower:
	    status = handleCmdAdjustPower(arfcn,s,&rspParam);
	    break;
	case CmdSetTxAtten:
	    status = handleCmdSetTxAttenuation(arfcn,s,&rspParam);
	    break;
	case CmdRxTune:
	case CmdTxTune:
	    status = handleCmdTune(c == CmdRxTune,arfcn,s,&rspParam);
	    break;
	case CmdSetTsc:
	    status = handleCmdSetTsc(arfcn,s,&rspParam);
	    break;
	case CmdSetMaxDelay:
	    status = 0;
	    rspParam = s;
	    break;
	case CmdSetRxGain:
	    status = handleCmdSetGain(c == CmdSetRxGain,s,&rspParam);
	    break;
	case CmdReadFactory:
	    // NOTE: Should we return the requested parameter name?
	    if (m_radio)
		status = m_radio->getFactoryValue(s,rspParam);
	    break;
	case CmdCustom:
	    status = handleCmdCustom(s,&rspParam,&reason);
	    break;
	case CmdPowerOn:
	    status = (radioPowerOn(&reason) ? CmdEOk : CmdEFailure);
	    break;
	case CmdPowerOff:
	    radioPowerOff();
	    break;
	case CmdManageTx: // NOTE: this command is just for test
	    handleCmdManageTx(arfcn,s,&rspParam);
	    break;
	case CmdNoise:
	{
	    String dump;
	    for (unsigned int i = 0;i < m_arfcnCount;i++) {
		dump << "ARFCN: ";
		dump <<  i;
		dump << ": noise: ";
		dump << (int)m_arfcn[i]->getAverageNoiseLevel();
		dump << " ; ";
	    }
	    Debug(this,DebugNote,"%s",dump.c_str());
	    rspParam << (int)-m_arfcn[0]->getAverageNoiseLevel();
	    break;
	}
	case CmdFreqOffset:
	    status = handleCmdFreqCorr(s,&rspParam);
	    break;
	default:
	    status = CmdEUnkCmd;
    }
    if (status)
	Debug(this,DebugNote,"Command '%s' (ARFCN=%u) failed in state %s: %d %s [%p]",
	    str,arfcn,trxStateName(state()),status,
	    reason.safe(lookup(status,s_cmdErrorName,"failure")),this);
    else
	Debug(this,DebugAll,"Command '%s' (ARFCN=%u) RSP '%s'",str,arfcn,rspParam.c_str());
    syncGSMTime();
    return buildCmdRsp(rsp,cmd,status,rspParam);
}

// Set the minimum power to accept radio bursts
void Transceiver::setBurstMinPower(int val)
{
    if (m_burstMinPower == val)
	return;
    m_burstMinPower = val;
    Debug(this,DebugInfo,"Burst minimum power level set to %d [%p]",val,this);
}

// Internal fatal error
void Transceiver::fatalError()
{
    m_error = true;
}

// Run read control socket loop
void Transceiver::runReadCtrlSocket()
{
    if (!(m_arfcnCount && m_arfcn[0]->getObject(YATOM("ARFCNSocket"))))
	return;
    while (true) {
	unsigned int i = 0;
	for (; i < m_arfcnCount; i++) {
	    ARFCNSocket* a = static_cast<ARFCNSocket*>(m_arfcn[i]);
	    int r = a->m_ctrl.readSocket(*a);
	    if (thShouldExit(this))
		break;
	    if (r < 0)
		break;
	    if (!r)
		continue;
	    String rsp;
	    String s((const char*)a->m_ctrl.m_readBuffer.data(),r);
	    if (s)
		command(s,&rsp,a->arfcn());
	    if (!rsp)
		continue;
	    r = a->m_ctrl.writeSocket((const void*)rsp.c_str(),rsp.length(),*this);
	    if (r <= 0)
		break;
	}
	if (i < m_arfcnCount)
	    break;
    }
    if (!thShouldExit(this))
	fatalError();
}

// Worker terminated notification
void Transceiver::workerTerminated(Thread* th)
{
    if (!th)
	return;
    Lock lck(TrxWorker::s_mutex);
    if (m_ctrlReadThread == th)
	m_ctrlReadThread = 0;
    else if (m_radioInThread == th)
	m_radioInThread = 0;
    else if (m_radioOutThread == th)
	m_radioOutThread = 0;
}

// Destroy the object
void Transceiver::destruct()
{
    stop();
    GenObject::destruct();
}

// Starting radio power on notification
void Transceiver::radioPowerOnStarting()
{
    m_rxQueue.clear();
    m_txPower = -20;
}

// Set channel (slot) type
void Transceiver::setChanType(unsigned int arfcn, unsigned int slot, int chanType)
{
    if (arfcn < m_arfcnCount)
	m_arfcn[arfcn]->initTimeslot(slot,chanType);
}

// ARFCN list changed notification
void Transceiver::arfcnListChanged()
{
}

// Retrieve the state name dictionary
const TokenDict* Transceiver::dictStateName()
{
    return s_stateName;
}

// Power on the radio if not already done
bool Transceiver::radioPowerOn(String* reason)
{
    DBGFUNC_TRXOBJ("Transceiver::radioPowerOn()",this);
    Lock lck(m_stateMutex);
    if (m_state == PowerOn)
	return true;
    if (!m_radio || m_state < PowerOff || !m_arfcnCount)
	return setReason(reason,"invalid state");
    if (!m_rxFreq)
	return setReason(reason,"Rx frequency not set");
    if (!m_txFreq)
	return setReason(reason,"Tx frequency not set");
    radioPowerOnStarting();
    const char* reasonStr = "failure";
    while (true) {
	Debug(this,DebugInfo,"Starting radio [%p]",this);
	if (!m_radio->powerOn()) {
	    reasonStr = "radio failure";
	    break;
	}
	if (!TrxWorker::create(m_radioInThread,TrxWorker::TrxRadioIn,this))
	    break;
	if (!TrxWorker::create(m_radioOutThread,TrxWorker::TrxRadioOut,this))
	    break;
	unsigned int i = 0;
	for (; i < m_arfcnCount; i++)
	    if (!m_arfcn[i]->radioPowerOn(reason))
		break;
	if (i < m_arfcnCount)
	    break;
	changeState(PowerOn);
	return true;
    }
    lck.drop();
    radioPowerOff();
    return setReason(reason,reasonStr);
}

// Power off the radio if not already done
void Transceiver::radioPowerOff()
{
    DBGFUNC_TRXOBJ("Transceiver::radioPowerOff()",this);
    TrxWorker::softCancel(TrxWorker::RadioMask);
    m_stateMutex.lock();
    unsigned int waitThOut = 0;
    if (m_radio) {
	if (m_state == PowerOn)
	    Debug(this,DebugInfo,"Stopping radio [%p]",this);
	waitThOut = m_radio->syncWriteWaitMs();
	if (waitThOut)
	    waitThOut += 50;
	// Signal Tx ready: this will stop us waiting for Tx ready
	m_radio->signalSendTx();
	m_radio->powerOff();
    }
    TrxWorker::cancelThreads(this,0,&m_radioInThread,waitThOut,&m_radioOutThread);
    for (unsigned int i = 0; i < m_arfcnCount; i++)
	m_arfcn[i]->radioPowerOff();
    if (m_state == PowerOn)
	changeState(PowerOff);
    m_stateMutex.unlock();
    m_rxQueue.clear();
}

void Transceiver::changeState(int newState)
{
    if (m_state == newState)
	return;
    Debug(this,DebugNote,"State changed %s -> %s [%p]",
	trxStateName(m_state),trxStateName(newState),this);
    m_state = newState;
}

static inline void clearARFCNs(ARFCN**& ptr, unsigned int& n)
{
    if (!ptr)
	return;
    for (unsigned int i = 0; i < n; i++)
	TelEngine::destruct(ptr[i]);
    delete[] ptr;
    ptr = 0;
    n = 0;
}

// Initialize the ARFCNs list (set or release)
bool Transceiver::resetARFCNs(unsigned int arfcns, int port, const String* rAddr,
    const char* lAddr, unsigned int nFillers)
{
    ARFCN** old = m_arfcn;
    if (m_arfcn) {
	stopARFCNs();
	clearARFCNs(m_arfcn,m_arfcnCount);
    }
    m_arfcnCount = arfcns;
    if (!m_arfcnCount) {
	if (old)
	    arfcnListChanged();
	return true;
    }
    m_arfcn = new ARFCN*[m_arfcnCount];
    for (unsigned int i = 0; i < m_arfcnCount; i++) {
	if (rAddr)
	    m_arfcn[i] = new ARFCNSocket(i);
	else
	    m_arfcn[i] = new ARFCN(i);
	m_arfcn[i]->setTransceiver(*this,"ARFCN[" + String(i) + "]");
	m_arfcn[i]->debugChain(this);
	GSMTxBurst* filler = 0;
	if (m_arfcn[i]->arfcn() == 0) {
	    filler = GSMTxBurst::buildFiller();
	    if (filler) {
		filler->buildTxData(m_signalProcessing);
		if (!filler->txData().length())
		    TelEngine::destruct(filler);
	    }
	}
	m_arfcn[i]->m_fillerTable.init(nFillers,filler);
    }
    if (rAddr)
	for (unsigned int i = 0; i < m_arfcnCount; i++) {
	    ARFCNSocket* a = static_cast<ARFCNSocket*>(m_arfcn[i]);
	    int p = port + 2 * i + 1;
	    if (a->initUDP(false,p + 100,*rAddr,p,lAddr) &&
		a->initUDP(true,p + 100 + 1,*rAddr,p + 1,lAddr))
		continue;
	    clearARFCNs(m_arfcn,m_arfcnCount);
	    if (old)
		arfcnListChanged();
	    return false;
	}
    arfcnListChanged();
    return true;
}

// Stop all ARFCNs
void Transceiver::stopARFCNs()
{
    for (unsigned int i = 0; i < m_arfcnCount; i++)
	m_arfcn[i]->stop();
}

// Sync upper layer GSM clock (update time)
bool Transceiver::syncGSMTime()
{
    String tmp("IND CLOCK ");
    Lock lck(m_clockUpdMutex);
    m_nextClockUpdTime = m_txTime;
    m_nextClockUpdTime.advance(216);
    tmp << (m_txTime.fn() + 2);
    if (m_clockIface.m_socket.valid()) {
	if (m_clockIface.writeSocket(tmp.c_str(),tmp.length() + 1,*this) > 0)
	    return true;
    }
    else
	Debug(this,DebugFail,"Clock interface is invalid [%p]",this);
    lck.drop();
    fatalError();
    return false;
}

#define TRX_SET_ERROR_BREAK(e) { code = e; break; }

// Handle (NO)HANDOVER commands. Return status code
int Transceiver::handleCmdHandover(bool on, unsigned int arfcn, String& cmd,
    String* rspParam)
{
    int code = CmdEFailure;
    while (true) {
	if (m_state != PowerOn)
	    TRX_SET_ERROR_BREAK(CmdEInvalidState);
	if (arfcn >= m_arfcnCount)
	    TRX_SET_ERROR_BREAK(CmdEInvalidARFCN);
	int slot = cmd.toInteger(-1);
	if (slot < 0 || slot > 7)
	    TRX_SET_ERROR_BREAK(CmdEInvalidParam);
	m_arfcn[arfcn]->setBurstAccess((unsigned int)slot,on);
	if (rspParam)
	    *rspParam << slot;
	return 0;
    }
    if (rspParam)
	*rspParam = cmd;
    return code;
}

// Handle SETSLOT command. Return status code
int Transceiver::handleCmdSetSlot(unsigned int arfcn, String& cmd, String* rspParam)
{
    int code = CmdEFailure;
    while (true) {
	if (m_state < Idle)
	    TRX_SET_ERROR_BREAK(CmdEInvalidState);
	if (arfcn >= m_arfcnCount)
	    TRX_SET_ERROR_BREAK(CmdEInvalidARFCN);
	int slot = -1;
	int chanType = -1;
	int pos = cmd.find(' ');
	if (pos > 0) {
	    slot = cmd.substr(0,pos).toInteger(-1);
	    chanType = cmd.substr(pos + 1).toInteger(-1);
	}
	if (slot < 0 || slot > 7)
	    TRX_SET_ERROR_BREAK(CmdEInvalidParam);
	setChanType(arfcn,(unsigned int)slot,chanType);
	if (rspParam)
	    *rspParam << slot << " " << chanType;
	return 0;
    }
    if (rspParam)
	*rspParam = cmd;
    return code;
}

int Transceiver::handleCmdSetPower(unsigned int arfcn, String& cmd, String* rspParam)
{
    int code = CmdEFailure;
    while (true) {
	if (m_state != PowerOn)
	    TRX_SET_ERROR_BREAK(CmdEInvalidState);
	// Power can be set for ARFCN 0 only
	if (arfcn)
	    TRX_SET_ERROR_BREAK(CmdEInvalidARFCN);
	if (!cmd)
	    TRX_SET_ERROR_BREAK(CmdEInvalidParam);
	int p = cmd.toInteger();
	int ok = m_radio->setTxPower(p + m_txAttnOffset);
	if (ok)
	    break;
	m_txPower = p;
	if (rspParam)
	    *rspParam << p;
	return 0;
    }
    if (rspParam)
	*rspParam = cmd;
    return code;
}

int Transceiver::handleCmdAdjustPower(unsigned int arfcn, String& cmd, String* rspParam)
{
    int code = CmdEFailure;
    while (true) {
	if (m_state != PowerOn)
	    TRX_SET_ERROR_BREAK(CmdEInvalidState);
	// Power can be set for ARFCN 0 only
	if (arfcn)
	    TRX_SET_ERROR_BREAK(CmdEInvalidARFCN);
	if (!cmd)
	    TRX_SET_ERROR_BREAK(CmdEInvalidParam);
	int p = cmd.toInteger();
	m_txPower += p;
	if (rspParam)
	    *rspParam << m_txPower;
	return 0;
    }
    if (rspParam)
	*rspParam = cmd;
    return code;
}

int Transceiver::handleCmdSetTxAttenuation(unsigned int arfcn, String& cmd, String* rspParam)
{
    int code = CmdEFailure;
    while (true) {
	if (m_state != PowerOn)
	    TRX_SET_ERROR_BREAK(CmdEInvalidState);
	// Power can be set for ARFCN 0 only
	if (arfcn)
	    TRX_SET_ERROR_BREAK(CmdEInvalidARFCN);
	if (!cmd)
	    TRX_SET_ERROR_BREAK(CmdEInvalidParam);
	int p = cmd.toInteger();
	int ok = m_radio->setTxPower(m_txPower + p);
	if (ok)
	    break;
	m_txAttnOffset = p;
	if (rspParam)
	    *rspParam << p;
	return 0;
    }
    if (rspParam)
	*rspParam = cmd;
    return code;
}

// Handle RXTUNE/TXTUNE commands
int Transceiver::handleCmdTune(bool rx, unsigned int arfcn, String& cmd, String* rspParam)
{
    int code = CmdEFailure;
    while (true) {
	if (!m_radio)
	    TRX_SET_ERROR_BREAK(CmdEInvalidState);
	if (arfcn)
	    TRX_SET_ERROR_BREAK(CmdEInvalidARFCN);
	if (!cmd)
	    TRX_SET_ERROR_BREAK(CmdEInvalidParam);
	int freq = cmd.toInteger();
	// NOTE: Should we add some extra value when using multiple ARFCNs?
	// Frequency is expected in kHz
	double val = (double)freq * 1000;
	int ok = m_radio->tune(rx,val,m_freqOffset);
	if (ok)
	    break;
	double& f = rx ? m_rxFreq : m_txFreq;
	f = freq;
	if (rspParam)
	    *rspParam << freq;
	return 0;
    }
    if (rspParam)
	*rspParam = cmd;
    return code;
}

// Handle SETTSC commands. Return status code
int Transceiver::handleCmdSetTsc(unsigned int arfcn, String& cmd, String* rspParam)
{
    int code = CmdEFailure;
    while (true) {
	if (arfcn)
	    TRX_SET_ERROR_BREAK(CmdEInvalidARFCN);
	int tsc = cmd.toInteger(-1);
	if (tsc < 0 || tsc > 7)
	    TRX_SET_ERROR_BREAK(CmdEInvalidParam);
	if (m_tsc != (unsigned int)tsc) {
	    Debug(this,DebugInfo,"%sTSC changed %u -> %d [%p]",
		prefix(),m_tsc,tsc,this);
	    m_tsc = (unsigned int)tsc;
	}
	if (rspParam)
	    *rspParam << m_tsc;
	return 0;
    }
    if (rspParam)
	*rspParam = cmd;
    return code;
}

// Handle SETRXGAIN command. Return status code
int Transceiver::handleCmdSetGain(bool rx, String& cmd, String* rspParam)
{
    int code = CmdEFailure;
    while (true) {
	if (!m_radio)
	    TRX_SET_ERROR_BREAK(CmdEInvalidState);
	if (!cmd)
	    TRX_SET_ERROR_BREAK(CmdEInvalidParam);
	double newVal = 0;
	int ok = m_radio->radioSetGain(rx,cmd.toInteger(),&newVal);
	if (ok)
	    break;
	if (rspParam)
	    *rspParam << (int)newVal;
	return 0;
    }
    if (rspParam)
	*rspParam = cmd;
    return code;
}

// Handle SETRXGAIN command. Return status code
int Transceiver::handleCmdFreqCorr(String& cmd, String* rspParam)
{
    int code = CmdEFailure;
    while (true) {
	if (!m_radio)
	    TRX_SET_ERROR_BREAK(CmdEInvalidState);
	if (!cmd)
	    TRX_SET_ERROR_BREAK(CmdEInvalidParam);
	if (!m_radio->setFreqCorr(cmd.toInteger()))
	    break;
	*rspParam = cmd;
	return 0;
    }
    if (rspParam)
	*rspParam = cmd;
    return code;
}

bool loadFileData(const String& fileName, ComplexVector& data)
{
    File f;
    if (!f.openPath(fileName)) {
	Debug(DebugNote,"Failed to open file '%s'!",fileName.c_str());
	return false;
    }
    DataBlock buf(0,(unsigned int)f.length());
    int read = f.readData(buf.data(),buf.length());
    if (f.length() != read) {
	Debug(DebugInfo,"Failed to read all the data from file %s",fileName.c_str());
	return false;
    }
    String s((char*)buf.data(),buf.length());
    return data.parse(s,&SigProcUtils::callbackParse);
}

// Handle CUSTOM command. Return status code
int Transceiver::handleCmdCustom(String& cmd, String* rspParam, String* reason)
{
    DDebug(this,DebugAll,"Handle custom cmd %s",cmd.c_str());
    if (cmd.startSkip("debug")) {
	DebugEnabler* dbg = this;
	if (cmd.startSkip("radio")) {
	    if (!m_radio) {
		if (reason)
		    *reason = "No radio";
		return CmdEInvalidState;
	    }
	    dbg = m_radio;
	}
	if (cmd.startSkip("level"))
	    setDebugLevel(dbg,&cmd);
	else if (cmd.isBoolean())
	    dbg->debugEnabled(cmd.toBoolean());
	else
	    return CmdEUnkCmd;
	return CmdEOk;
    }
    if (cmd.startSkip("dumprx ",false)) {
	if (cmd == YSTRING("complex")) {
	    m_dumpOneRx= true;
	    return CmdEOk;
	}
	unsigned int arfcn = cmd.toInteger(0);
	if (arfcn >= m_arfcnCount)
	    return CmdEUnkCmd;
	ARFCNSocket* ar = YOBJECT(ARFCNSocket,m_arfcn[arfcn]);
	if (!ar)
	    return CmdEUnkCmd;
	ar->setPrintSocket();
	return CmdEOk;
    }
    if (cmd.startSkip("freqout ",false)) { 
	// Sends to output only the frequency shift vectors
	// "mbts radio freqout 0,1,2,3" summes all ARFCN's frequency shift vectors
	// and send them to output.
	ObjList* split = cmd.split(',');
	if (!split)
	    return CmdEUnkCmd;
	m_sendArfcnFS = 0;
	for (ObjList* o = split->skipNull(); o; o = o->skipNext()) {
	    String* s = static_cast<String*>(o->get());
	    unsigned int shift = (unsigned int)s->toInteger(0,0,0);
	    if (shift < 8)
		m_sendArfcnFS |= (1 << shift);
	}
	TelEngine::destruct(split);
	return CmdEOk;
    }
    if (cmd.startSkip("freqout")) {
	m_sendArfcnFS = 0;
	return CmdEOk;
    }
    if (cmd.startSkip("testtxburst ",false)) {
	int increment = -1;
	if (cmd == YSTRING("10"))
	    increment = 2;
	if (cmd == YSTRING("11"))
	    increment = 1;
	DataBlock d(0,GSM_BURST_TXPACKET);
	uint8_t* testData = (uint8_t*)d.data();
	for (int i = GSM_BURST_TXHEADER; i < GSM_BURST_LENGTH && increment > 0; i += increment)
	    testData[i] = 1;
	Lock myLock(m_testMutex);
	TelEngine::destruct(m_txTestBurst);
	m_txTestBurst = GSMTxBurst::parse(d);
	if (!m_txTestBurst)
	    return CmdEFailure;
	return CmdEOk;
    }
    if (cmd.startSkip("testtxburst")) {
	Lock myLock(m_testMutex);
	TelEngine::destruct(m_txTestBurst);
	return CmdEOk;
    }
    if (cmd == YSTRING("dumponetx")) {
	m_dumpOneTx = true;
	return CmdEOk;
    }
    if (cmd.startSkip(YSTRING("complexrx "),false)) {
	ComplexVector* cv = new ComplexVector();
	if (!loadFileData(cmd,*cv))
	    return CmdEUnkCmd;
	// TODO See how to set burst type
	m_radio->setLoopbackArray(cv);
	return CmdEOk;
    }
    if (cmd == YSTRING("complexrx")) {
	m_radio->setLoopbackArray();
	return CmdEOk;
    }
    if (cmd.startSkip("predefined ",false)) {
	int min = 0, max = 0, inc = 0;
	int ret = ::sscanf(cmd.c_str(),"%d %d %d",&min,&max,&inc);
	if (ret != 3 || min > max)
	    return CmdEUnkCmd;
	if (!m_radio)
	    return CmdEInvalidState;
	m_radio->setTestData(min,max,inc);
	return CmdEOk;
    }
    if (cmd.startSkip("internal-loopback ",false)) {
	if (!m_radio)
	    return CmdEInvalidState;
	m_radio->setLoopback(true,cmd.toInteger());
	return CmdEOk;
    }
    if (cmd.startSkip("internal-loopback",false)) {
	if (!m_radio)
	    return CmdEInvalidState;
	m_radio->setLoopback(false);
	return CmdEOk;
    }
    if (cmd.startSkip("dump-freq-shift ",false)) {
	dumpFreqShift(cmd.toInteger());
	return CmdEOk;
    }
    if (cmd.startSkip("noise",false)) {
	String dump;
	for (unsigned int i = 0;i < m_arfcnCount;i++) {
	    dump << "ARFCN: ";
	    dump <<  i;
	    dump << ": average noise level: ";
	    SigProcUtils::appendFloat(dump,m_arfcn[i]->getAverageNoiseLevel(),"");
	    dump << " ; ";
	}
	*rspParam << dump;
	return CmdEOk;
    }
    if (cmd.startSkip("slots-delay")) {
	if (rspParam) {
	    String dump;
	    for (unsigned int i = 0;i < m_arfcnCount;i++) 
		m_arfcn[i]->dumpSlotsDelay(*rspParam);
	}
	return CmdEOk;
    }
    if (cmd.startSkip("file-dump ",false)) {
	if (FileDataDumper::start(s_dumper,cmd))
	    return CmdEOk;
	return CmdEFailure;
    }
    if (cmd == YSTRING("file-dump")) {
	FileDataDumper::stop(s_dumper);
	return CmdEOk;
    }
    if (cmd == YSTRING("shutdown")) {
	Debug(this,DebugInfo,"Received shutdown command");
	m_shutdown = true;
	fatalError();
	return CmdEOk;
    }
    if (cmd.startSkip("show-mbts-traffic ",false)) {
	ObjList* split = cmd.split(' ',false);
	ObjList* o = split->skipNull();
	if (!o) {
	    TelEngine::destruct(split);
	    return CmdEFailure;
	}
	int arfcn = (static_cast<String*>(o->get()))->toInteger(-1);
	if (arfcn < 0 || arfcn > 4) {
	    Debug(this,DebugNote,"Invalid ARFCN index %d.",arfcn);
	    TelEngine::destruct(split);
	    return CmdEFailure;
	}
	o = o->skipNext();
	if (!o) {
	    Debug(this,DebugNote,"Unable to obtain direction.");
	    TelEngine::destruct(split);
	    return CmdEFailure;
	}
	String* direct = static_cast<String*>(o->get());
	bool direction = *direct == YSTRING("out");
	o = o->skipNext();
	if (!o) {
	    TelEngine::destruct(split);
	    return CmdEFailure;
	}
	m_arfcn[arfcn]->showTraffic(direction,*o);
	TelEngine::destruct(split);
	return CmdEOk;
    }
    if (cmd.startSkip("show-mbts-traffic",false)) {
	Output("mbts radio show-mbts-traffic arfcn[0,1,2-3] in/out enable[true/false] timeslot[0,1,5-7] devider[0 to 51] devider pos[1,11,21,31,41]");
	return CmdEOk;
    }
    if (cmd.startSkip("traffic-shape ",false)) {
	ObjList* split = cmd.split(' ',false);
	ObjList* o = split->skipNull();
	int arfcn = (static_cast<String*>(o->get()))->toInteger(-1);
	if (arfcn < 0 || arfcn > 3) {
	    Debug(this,DebugNote,"Invalid ARFCN index %d",arfcn);
	    return CmdEFailure;
	}
	o = o->skipNext();
	m_arfcn[arfcn]->m_shaper.parse(o);
	TelEngine::destruct(split);
	return CmdEOk;
    }
    if (cmd.startSkip("traffic-shape")) {
	Output("mbts radio traffic-shape arfcn[0,1,2-3] timeslot[0,1,5-7] value");
	return CmdEOk;
    }
    if (cmd.startSkip(YSTRING("toa-shift ")),false) {
	m_toaShift = cmd.toInteger();
	return CmdEOk;
    }
    if (m_radio)
	return m_radio->command(cmd,rspParam,reason);
    return CmdEUnkCmd;
}

int Transceiver::handleCmdManageTx(unsigned int arfcn, String& cmd, String* rspParam)
{
    NamedList params("");
    ObjList* p = cmd.split(' ');
    for (ObjList* o = p->skipNull();o;o = o->skipNext()) {
	String* nvs = static_cast<String*>(o->get());
	ObjList* nvo = nvs->split('=');
	if (nvo->count() != 2) {
	    TelEngine::destruct(nvo);
	    continue;
	}
	params.addParam(*static_cast<String*>((*nvo)[0]),*static_cast<String*>((*nvo)[1]));
	TelEngine::destruct(nvo);
    }
    TelEngine::destruct(p);
    String command = params.getParam(YSTRING("cmd"));
    if (command == YSTRING("dump")) {
	m_arfcn[arfcn]->dumpBursts();
	return 0;
    }
    if (command == YSTRING("test_tx")) {
	if (!m_radio)
	    return 0;
	m_radio->testTx(params);
	return 0;
    }
    DDebug(this,DebugStub,"Received ManageTx command %s %u",command.c_str(),GSM_HYPERFRAME_MAX_HALF);
    return 0; 
}


//
// TransceiverQMF
//
TransceiverQMF::TransceiverQMF(const char* name)
    : Transceiver(name),
    m_halfBandFltCoeffLen(11),
    m_tscSamples(26)
{
    initNormalBurstTSC();
    initAccessBurstSync();
}

static void adjustParam(TransceiverObj* obj, NamedList& p, const String& param,
    unsigned int value)
{
    String* s = p.getParam(param);
    int val = s ? s->toInteger(-1) : -1;
    if (val == (int)value)
	return;
    if (!s) {
	p.setParam(param,String(value));
	return;
    }
    if (obj) {
	int level = (val < (int)value) ? DebugAll : DebugConf;
	Debug(obj,level,"%sAdjusting parameter %s='%s' -> %u [%p]",
	    obj->prefix(),param.c_str(),s->c_str(),value,obj);
    }
    *s = value;
}

// Initialize the transceiver. This method should be called after construction
bool TransceiverQMF::init(RadioIface* radio, const NamedList& params)
{
    if (m_state != Invalid)
	return Transceiver::init(radio,params);
    // First init: adjust ARFCNs and oversampling
    NamedList p(params);
    adjustParam(this,p,YSTRING("oversampling"),8);
    adjustParam(this,p,YSTRING("arfcns"),4);
    p.addParam("conf_arfcns",params.getValue(YSTRING("arfcns"),"1"));
    return Transceiver::init(radio,p);
}

// Re-Initialize the transceiver
void TransceiverQMF::reInit(const NamedList& params)
{
    Transceiver::reInit(params);
    m_tscSamples = getUInt(params,YSTRING("chan_estimator_tsc_samples"),26,2,26);
}

// Process a received radio burst
bool TransceiverQMF::processRadioBurst(unsigned int arfcn, ArfcnSlot& slot, GSMRxBurst& b)
{
    ARFCN* a = this->arfcn(arfcn);
    if (!a)
	return false;

    const GSMTime& t = b.time();
    unsigned int len = b.m_data.length();
    int bType = ARFCN::burstType(slot,t.fn());
    if (bType != ARFCN::BurstNormal && bType != ARFCN::BurstAccess) {
	String tmp;
	int level = -1;
	if (slot.warnRecvDrop) {
	    slot.warnRecvDrop = false;
	    level = (bType != ARFCN::BurstUnknown) ? DebugInfo : DebugNote;
	    tmp = "unhandled";
	    tmp << " burst type (" << bType << "," << ARFCN::burstType(bType) << ")";
	    tmp << " slot_type " << a->chanType(slot.type);
	}
	a->dropRxBurst(tmp,t,len,level);
	return false;
    }
    if (len < ARFCN_RXBURST_LEN)
	return false;

    // Use the center values of the input data to calculate the power level.
    float power = SignalProcessing::computePower(b.m_data.data(),
	b.m_data.length(),5,0.2, (148 - 4) / 2);

    // Use the last 4 samples of the input data (guard period) to calculate the noise level
    float noise = SignalProcessing::computePower(b.m_data.data(),
	b.m_data.length(),2,0.5,b.m_data.length() - 2);

    noise += SignalProcessing::computePower(b.m_data.data(),
	b.m_data.length(),2,0.5,0);

    noise /= 2;

    a->addAverageNoise(10 * ::log10f(noise));

    // Calculate Signal to Noise ratio
    float SNR = power/noise;
    b.m_powerLevel = 10 * ::log10f(power);

#ifdef TRANSCEIVER_DUMP_ARFCN_PROCESS_IN
    a->dumpRecvBurst("processRadioBurst()",t,b.m_data.data(),len);
#else
    XDebug(a,DebugAll,"processRadioBurst ARFCN %d FN %d TN %d T2 %d T3 %d. Level %f, noise %f, SNR %f powerDb %g low SNR %s lowPower %s, burst type %s",
	  arfcn, t.fn(),slot.slot,t.fn() % 26,t.fn() % 51,power, noise, SNR,b.m_powerLevel,String::boolText(SNR < m_snrThreshold),String::boolText(b.m_powerLevel < m_burstMinPower),ARFCN::burstType(bType));
#endif

    if (SNR < m_snrThreshold) {
	// Discard Burst low SNR
#ifndef CALLGRIND_CHECK
	a->dropRxBurst("low SNR",t,len,DebugAll,false);
	return false;
#endif
    }

    if (b.m_powerLevel < m_burstMinPower) {
#ifndef CALLGRIND_CHECK
	a->dropRxBurst("low power",t,len,DebugAll,false);
	return false;
#endif
    }

    static unsigned int s_lastPowerWarn = 0;
    if  (b.m_powerLevel > m_upPowerThreshold && s_lastPowerWarn < t.fn()) {
	Debug(a,DebugInfo, "Receiver clipping on ARFCN %d Slot %d, %f dB", arfcn, slot.slot, b.m_powerLevel);
	s_lastPowerWarn = t.fn() + 217; // 1 sec = 216.(6)
    }

    SignalProcessing::applyMinusPIOverTwoFreqShift(b.m_data);

    FloatVector* trainingSeq = 0;
    int start,heLen,center;

    if (bType == ARFCN::BurstNormal) {
	trainingSeq  = &m_nbTSC[m_tsc];
	start = b.m_data.length() / 2 - GSM_NB_TSC_LEN / 2;
	heLen = GSM_NB_TSC_LEN;
	center = GSM_NB_TSC_LEN / 2 - 1;
    }
    else {
	trainingSeq = &m_abSync;
	start = 8;
	center = 24; // HardCodded Ask David
	heLen = GSM_AB_SYNC_LEN + m_maxPropDelay;
    }

    dumpRxData("correlate-in1",arfcn,"",b.m_data.data() + start,heLen);
    dumpRxData("correlate-in2",arfcn,"",trainingSeq->data(), trainingSeq->length());

    ComplexVector he(heLen);
    SignalProcessing::correlate(he,b.m_data,start,heLen,*trainingSeq);

    dumpRxData("correlate-out",arfcn,"",he.data(), he.length());

    // Find the peak power in the channel estimate.
    float max = 0, rms = 0;
    int maxIndex = -1;
    // GSM standards specififys that we should calculate from center +/- 2
    // We use center +/- 5 to be sure
    for (unsigned i = center - 5; i < he.length(); i++) {
	float pwr = he[i].mulConj();
	rms += pwr;
	if (pwr < max) 
	    continue;
	max = pwr;
	maxIndex = i;
    }
    rms /= (he.length() - (center - 5));
    if (rms == 0) {
	a->dropRxBurst("invalid rms '0'",t,len,DebugNote,true);
	return false;
    }
    float peakOverMean = max / rms;
    XDebug(a,DebugAll,"ARFCN %d Slot %d. chan max %f @ %d, rms %f, peak/mean %f",
		arfcn,slot.slot,max, maxIndex, rms, peakOverMean);
    if (peakOverMean < m_peakOverMeanThreshold) {
#ifndef CALLGRIND_CHECK
	a->dropRxBurst("low peak / min",t,len,DebugInfo,false);
	return false;
#endif
    }


    // TOA error.  Negative for early, positive for late.
    int toaError = maxIndex - center;

    DDebug(a,DebugAll,"ARFCN %d Slot %d. TOA error %d", arfcn,slot.slot,toaError);

    // indexing relationships
    // maxIndex - actual t0 of the received signal channel estimate
    // center - expected t0 of the received signal channel estimate
    // toaError - the offset of the channel estaimte from its expected position, negative for early

    // TOA error should not exceed +/-2 for a normal burst.
    // It indicates a failure of the closed loop timing control.

    static unsigned int s_lastTOAWarn = 0;
    static unsigned int s_toaCount = 0;
    if (bType == ARFCN::BurstNormal && (toaError > 2 || toaError < -2)) {
	s_toaCount++;
	if (s_lastTOAWarn < t.fn()) {
	    Debug(a,DebugAll,"ARFCN %d Slot %d. Excessive TOA error %d, peak/mean %f count %u",arfcn,slot.slot, toaError, peakOverMean,s_toaCount);
	    s_toaCount = 0;
	    s_lastTOAWarn = t.fn() + 217;
	}
    } else if (s_toaCount && s_lastTOAWarn < t.fn()) {
	Debug(a,DebugAll,"ARFCN %d Slot %d. Excessive TOA errors count %u",arfcn,slot.slot,s_toaCount);
	s_toaCount = 0;
    }

    // Shift the received signal to compensate for TOA error.
    // This shifts the signal so that the max power image is aligned to the expected position.
    // The alignment error is +/- 1/2 symbol period;
    // fractional alignment is corrected by correlation with channel estaimte.
    if (toaError < 0) {
	for (unsigned int i = -toaError; i < b.m_data.length(); i++)
	    b.m_data[i] = b.m_data[i + toaError];
	for (int i = 0; i < -toaError; i++)
	    b.m_data[i].set(0,0);
    } else if (toaError != 0) {
	for (unsigned int i = 0; i < b.m_data.length() - toaError; i++)
	    b.m_data[i] = b.m_data[i + toaError];
	for (unsigned int i = b.m_data.length() - toaError; i < b.m_data.length(); i++)
	    b.m_data[i].set(0,0);
    }
#ifdef DEBUG
    toaError += m_toaShift;
#endif
    b.m_timingError = toaError;
    // Trim and center the channel estimate.
    // This shifts the channel estimate to put the max power peak in the center, +/- 1/2 symbol period.
    // The alignment error of heTrim is exactly the opposiate of the alignment error of xf2.
    if (maxIndex < 0) {
	a->dropRxBurst("negative max index",t,len,DebugStub);
	return false;
    }

    // GSM standards specififys 10 
    // To be sure we go 11
    // ASK David for details
    for (unsigned i = 0; i < 11; i++)
	he[i] = he[maxIndex - 5 + i];

    dumpRxData("demod-he",arfcn,"",he.data(), he.length());

    float sumTSq = 0;
    float sumP = 0;
    int m = 0;
    for(int i = 5; i < 11; i++,m++) {
	float p = he[i].mulConj();
	sumTSq += m * m *p;
	sumP += p;
    }
    float delaySpread = sqrt(sumTSq/sumP);
    a->addSlotDelay(slot.slot,delaySpread);

    FloatVector v(b.m_data.length());
    Equalizer::equalize(v,b.m_data,he,11);

    dumpRxData("demod-u",arfcn,"",v.data(), v.length());

    float powerSum = 0;
    // Calculate the middle without guard period
    for (unsigned int i = 72; i <= 76;i++)
	powerSum += v[i] * v[i];

    powerSum = ::sqrtf(powerSum * 0.2);

    for (unsigned int i = 8;i < ARFCN_RXBURST_LEN;i++) {
	float vi = v[i - BOARD_SHIFT] / powerSum;
	if (vi >= 1)
	    b.m_bitEstimate[i] = 255;
	else if (vi  <= -1)
	    b.m_bitEstimate[i] = 0;
	else
	    b.m_bitEstimate[i] = (uint8_t)::round((vi + 1.0F) * 0.5 * 255);
    }

    bool printOutput = false;
#ifdef TRANSCEIVER_DUMP_RX_DEBUG
    printOutput = true;
#endif

#ifdef TRANSCEIVER_DUMP_RX_INPUT_OUTPUT
    printOutput = true;
#endif

    if (debugAt(DebugAll) && printOutput) {
#ifdef TRANSCEIVER_DUMP_RX_INPUT_OUTPUT
	RxInData::dump(t);
#endif
	String dump,db;
	for (int i = 8;i < ARFCN_RXBURST_LEN;i++)
	    dump << b.m_bitEstimate[i] << ",";
	::printf("\noutput-data:%d:%s\n",arfcn,dump.c_str());
    }
#ifdef TRANSCEIVER_DUMP_DEMOD_PERF
    uint8_t burst[148];
    bool notified = false;
    unsigned int count = 0;
    for (int i = 8;i < ARFCN_RXBURST_LEN;i++) {
	burst[i - 8] = (b.m_bitEstimate[i] < 128 ? 0 : 1);
	if (b.m_bitEstimate[i] > 32 && b.m_bitEstimate[i] < 196) {
	    if (!notified)
		notified = true;
	    count ++;
	}
    }
    if (notified)
	Debug(a,DebugNote,"Poor demodulator performance! ARFCN=%d, FN=%d, TN=%d, T2=%d, T3=%d, uncerten values: %d",a->arfcn(),t.fn(),t.tn(),t.fn() % 26,t.fn() % 51,count);

    const int8_t *ts = GSMUtils::nbTscTable();
    ts +=  + (GSM_NB_TSC_LEN * m_tsc);
    uint8_t *bp = burst + 61;
    bool tsNotif = false;
    bool shoulHave = bType != ARFCN::BurstAccess;
    for (unsigned int i = 0;i < GSM_NB_TSC_LEN;i++) {
	if (ts[i] == bp[i])
	    continue;
	String t1,t2;
	t1.hexify((void*)ts,GSM_NB_TSC_LEN,' ');
	t2.hexify(bp,GSM_NB_TSC_LEN,' ');
	Debug(a,shoulHave ? DebugWarn : DebugInfo,"Wrong Training Sequence! ARFCN=%d, FN=%d, TN=%d, T2=%d, T3=%d bType : %s\nsequence:%s\nburst   :%s",a->arfcn(),t.fn(),t.tn(),t.fn() % 26,t.fn() % 51,ARFCN::burstType(bType),t1.c_str(),t2.c_str());
	tsNotif = true;
	break;
    }
    Debug(a,DebugAll,"Processed RX burst! Good Demod Performance %s Valid Training Seq %s power %f",
	String::boolText(!notified),String::boolText(!tsNotif),b.m_powerLevel);
    if (!notified && !tsNotif)
	Debug(DebugTest,"Got good burst!! ARFCN=%d, FN=%d, TN=%d, T2=%d, T3=%d",a->arfcn(),t.fn(),t.tn(),t.fn() % 26,t.fn() % 51);
#endif
    a->m_rxTraffic.show(&b);
    return true;
}

// Starting radio power on notification
void TransceiverQMF::radioPowerOnStarting()
{
    Transceiver::radioPowerOnStarting();
    String tmp;
    const QMFBlockDesc* d = s_qmfBlock4;
    for (unsigned int i = 0; i < 15; i++, d++) {
	QmfBlock& b = m_qmf[i];
	b.arfcn = d->arfcn;
	if (b.freqShiftValue != d->freqShiftValue) {
	    b.freqShiftValue = d->freqShiftValue;
	    b.freqShift.clear();
	}
#if 0
	char buf[256];
	if (!tmp)
	    tmp << "\r\nIndex Name ARFCN FreqShift";
	String s;
	if (b.arfcn != 0xffffffff)
	    s = b.arfcn;
	int n = ::sprintf(buf,"%-5d %-4s %-5s %.15g",i,d->name,s.safe(),b.freqShiftValue);
	tmp << "\r\n";
	tmp.append(buf,n);
#endif
    }
    if (tmp)
	Debug(this,DebugAll,"%sQMF nodes: [%p]\r\n-----%s\r\n-----",
	    prefix(),this,tmp.c_str());
    // Generate the half band filter coefficients
    if (m_halfBandFltCoeffLen && m_halfBandFltCoeff.length() != m_halfBandFltCoeffLen) {
	m_halfBandFltCoeff.assign(m_halfBandFltCoeffLen);
	float lq_1 = m_halfBandFltCoeff.length() - 1;
	float n0 = lq_1 / 2;
	float* d = m_halfBandFltCoeff.data();
	for (unsigned int i = 0; i < m_halfBandFltCoeff.length(); i++) {
	    float offset = (float)i - n0;
	    float omega = 0.54 - 0.46 * ::cosf(2 * PI * i / lq_1);
	    if (offset) {
		float func = PI / 2 * offset;
		*d++ = omega * (::sinf(func) / func);
	    }
	    else
		*d++ = omega;
	}
#ifdef TRANSCEIVER_DUMP_RX_DEBUG
	String dumphb = "hb:";
	m_halfBandFltCoeff.dump(dumphb,SigProcUtils::appendFloat);
	::printf("%s",dumphb.c_str());
#endif
    }
}

// Set channel (slot) type
void TransceiverQMF::setChanType(unsigned int arfcn, unsigned int slot, int chanType)
{
    Transceiver::setChanType(arfcn,slot,chanType);
    if (arfcn >= m_arfcnCount)
	return;
    for (unsigned int i = 0; i < 15; i++) {
	if (m_qmf[i].arfcn != arfcn)
	    continue;
	// Setup have chans availability flags in ARFCN node and parents
	for (unsigned int n = i; m_arfcn[arfcn]->chans(); n = (n - 1) / 2) {
	    m_qmf[n].chans = true;
	    if (!n)
		break;
	}
	break;
    }
}

// ARFCN list changed notification
void TransceiverQMF::arfcnListChanged()
{
    Transceiver::arfcnListChanged();
    for (unsigned int i = 0; i < 15; i++)
	m_qmf[i].chans = false;
}

void TransceiverQMF::dumpFreqShift(unsigned int index) const
{
    if (index > 14) {
	Debug(this,DebugNote,"Invalid qmf index!");
	return;
    }
    String tmp;
    char t[100];
    for (unsigned int i = 0;i < m_qmf[index].freqShift.length();i++) {
	int l = ::sprintf(t,"%g%+g",m_qmf[index].freqShift[i].real(),m_qmf[index].freqShift[i].imag());
	tmp.append(t,l);
	if (i != m_qmf[index].freqShift.length() - 1)
	    tmp << ",";
    }
    printf("\nqmf(%d).fs:%s\n",index,tmp.c_str());
}

// Process received radio data
void TransceiverQMF::processRadioData(RadioRxData* d)
{
    if (!d)
	return;
    XDebug(this,DebugAll,"Processing radio input TN=%u FN=%u len=%u [%p]",
	d->m_time.tn(),d->m_time.fn(),d->m_data.length(),this);
    m_qmf[0].data.steal(d->m_data);
#ifdef TRANSCEIVER_DUMP_RX_INPUT_OUTPUT
    RxInData::add(m_qmf[0].data,d->m_time);
#endif
    qmf(d->m_time);
    TelEngine::destruct(d);
}

// Run the QMF algorithm
void TransceiverQMF::qmf(const GSMTime& time, unsigned int index)
{
    if (index > 14) {
	Debug(this,DebugFail,"qmf: invalid index %u [%p]",index,this);
	fatalError();
	return;
    }
    QmfBlock& crt = m_qmf[index];
    if (!(crt.chans && crt.data.data() && crt.data.length()))
	return;

    if (index == 0) {
	if (s_dumper) {
	    s_dumper->addData(crt.data);
	    return;
	}
	crt.power = 0;
	for (unsigned int i = 0;i < crt.data.length();i++)
	    crt.power += crt.data[i].mulConj();
	crt.power /= crt.data.length();
    }

    bool final = (index > 6);
    int indexLo = -1;
    int indexHi = -1;
    if (final) {
	if (crt.arfcn >= m_arfcnCount) {
	    Debug(this,DebugFail,"No ARFCN in QMF block at index %d [%p]",index,this);
	    fatalError();
	    return;
	}
    }
    else {
	indexLo = 2 * (int)index + 1;
	indexHi = indexLo + 1;
	if (!m_qmf[indexLo].chans)
	    indexLo = -1;
	if (!m_qmf[indexHi].chans)
	    indexHi = -1;
	if (indexLo < 0 && indexHi < 0)
	    return;
	final = false;
    }

    crt.power = 10 * ::log10f(crt.power);

#ifdef XDEBUG
    String tmp;
    tmp << "QMF[" << index << "] ";
    Debug(this,DebugAll,"%s len=%u low=%d high=%d powerDb=%f [%p]",
	  tmp.c_str(),crt.data.length(),indexLo,indexHi,crt.power,this);
#endif
#ifdef TRANSCEIVER_DUMP_QMF_IN
    String tmp1;
    tmp1 << "QMF[" << index << "] starting";
    dumpRecvBurst(tmp1,time,crt.data.data(),crt.data.length());
#endif
    dumpRxData("qmf[",index,"].x",crt.data.data(),crt.data.length());
    if (crt.power < m_burstMinPower)
	return;
    // Forward data to ARFCNs
    if (final) {
	RadioRxData* r = new RadioRxData(time);
	r->m_data.steal(crt.data);
	XDebug(this,DebugAll,"Forwarding radio data ARFCN=%u len=%u TN=%u FN=%u [%p]",
	    crt.arfcn,r->m_data.length(),time.tn(),time.fn(),this);
	m_arfcn[crt.arfcn]->recvRadioData(r);
	return;
    }
    // Frequency shift
    qmfApplyFreqShift(crt);
#ifdef TRANSCEIVER_DUMP_QMF_FREQSHIFTED
    String tmp2;
    tmp2 << "QMF[" << index << "] applied frequency shifting";
    dumpRecvBurst(tmp2,time,crt.data.data(),crt.data.length());
#endif
    dumpRxData("qmf[",index,"].xp",crt.data.data(),crt.data.length());
    // Build the half band filter, generate low/high band data and call QMF again
    qmfBuildHalfBandFilter(crt);
#ifdef TRANSCEIVER_DUMP_QMF_HALFBANDFILTER
    String tmp3;
    tmp3 << "QMF[" << index << "] built half band filter";
    dumpRecvBurst(tmp3,time,crt.halfBandFilter.data(),crt.halfBandFilter.length());
#endif
    dumpRxData("qmf[",index,"].w",crt.halfBandFilter.data(),crt.halfBandFilter.length());

    if (indexLo >= 0 && !thShouldExit(this)) {
	qmfBuildOutputLowBand(crt,m_qmf[indexLo].data,&m_qmf[indexLo].power);
	qmf(time,indexLo);
    }
    if (indexHi >= 0 && !thShouldExit(this)) {
	qmfBuildOutputHighBand(crt,m_qmf[indexHi].data,&m_qmf[indexHi].power);
	qmf(time,indexHi);
    }
}

// Build the half band filter
// x: the input data, kept in b.data
// w: half band filter data, kept in b.halfBandFilter
// h: half band filter coefficients, kept in m_halfBandFltCoeff
// w[i] = SUM[j=0..n0](x[i + 2 * j] * h[2 * j])
void TransceiverQMF::qmfBuildHalfBandFilter(QmfBlock& b)
{
    if (b.halfBandFilter.length() < b.data.length())
	b.halfBandFilter.resize(b.data.length() + 10);

    unsigned int n0 = (m_halfBandFltCoeff.length() + 1) / 2;
    unsigned int n0_2 = m_halfBandFltCoeff.length();
    Complex* x = b.data.data();
    Complex* w = b.halfBandFilter.data();
    float* h = m_halfBandFltCoeff.data();

    // From w vector only half of the values are used to determine the high and low band filters
    // So we skip calculating the data.

    // The input data should be padded with m_halfBandFltCoeff length.

    // substract represents the amount of data that should be at the start of x.
    int substract = n0 - m_halfBandFltCoeff.length() % 2;

    for (unsigned int i = 0;i < n0; i += 2, w += 2) {
	// If i + 2 * j < n0 the input data will contain 0;
	// So we have i + 2 * j >= n0 -> 2j >= n0 - i
	(*w).set(0,0);
	for (unsigned int j = (n0 - i);j <  n0_2;j += 2)
	    Complex::sumMulF(*w,x[i + j - substract],h[j]);
    }

    unsigned int end = b.data.length() - n0;
    end += b.data.length() % 2;

    for (unsigned int i = n0;i < end; i += 2, w += 2) {
	(*w).set(0,0);
	Complex* wx = &x[i - substract];
	float* wh = h;
	for (unsigned int j = 0;j < n0_2;j += 2, wx += 2,wh += 2)
	    Complex::sumMulF(*w,(*wx),(*wh));
    }

    for (unsigned int i = end;i < b.data.length(); i += 2, w += 2) {
	unsigned int lastJ = (b.data.length() - i) + n0;
	(*w).set(0,0);
	for (unsigned int j = 0;j < lastJ;j += 2)
	    Complex::sumMulF(*w,x[i + j - substract],h[j]);
    }
}

// Build the QMF low band output
// x: the input data, kept in b.data
// w: half band filter data, kept in b.halfBandFilter
// y[i] = (x[2 * i + 2 * n0] + w[2 * i]) / 2
void TransceiverQMF::qmfBuildOutputLowBand(QmfBlock& b, ComplexVector& y, float* power)
{
    y.resize(b.data.length() / 2);
    Complex* yData = y.data();
    Complex* x = b.data.data();
    Complex* w = b.halfBandFilter.data();
    *power = 0;

    for (unsigned int i = 0;i < y.length();i ++,x += 2,w += 2) {
	Complex::sum(yData[i],*x,*w);
	yData[i] *= 0.5f;
	*power += yData[i].mulConj();
    }
    *power /= y.length();
}

// Build the QMF high band output
// Behave as the input vector is prepended with n0 zero samples
// x: the input data, kept in b.data
// w: half band filter data, kept in b.halfBandFilter
// y[i] = x[2 * i + 2 * n0] - w[2 * i]
void TransceiverQMF::qmfBuildOutputHighBand(QmfBlock& b, ComplexVector& y, float* power)
{
    y.resize(b.data.length() / 2);
    Complex* yData = y.data();
    Complex* x = b.data.data();
    Complex* w = b.halfBandFilter.data();
    *power = 0;

    for (unsigned int i = 0;i < y.length();i ++,x += 2,w += 2) {
	Complex::diff(yData[i],*x,*w);
	yData[i] *= 0.5f;
	*power += yData[i].mulConj();
    }
    *power /= y.length();
}

void TransceiverQMF::initNormalBurstTSC(unsigned int len)
{
    if (!len || len > GSM_NB_TSC_LEN)
	len = 16;
    const int8_t* table = GSMUtils::nbTscTable();
    
    static const float s_normalScv = 1.0F / 16.0F;
    for (unsigned int i = 0;i < 8;i++, table += GSM_NB_TSC_LEN) {
	m_nbTSC[i].resize(len);
	const int8_t* p = table;
	p += 5; // David needs to explain why
	float* f = m_nbTSC[i].data();
	for (unsigned int n = 0;n < len;n++,f++, p++)
	    *f = *p ? s_normalScv : - s_normalScv;
    }
}

void TransceiverQMF::initAccessBurstSync()
{
    m_abSync.resize(GSM_AB_SYNC_LEN);

    const int8_t* p = GSMUtils::abSyncTable();
    static const float s_accessScv = 1.0F / 41.0F;
    for (unsigned int i = 0; i < GSM_AB_SYNC_LEN;i++, p++)
	m_abSync[i] = *p ? s_accessScv : -s_accessScv;
}

void setBool(bool* table, unsigned int tableLen, const String& cond)
{
    ::memset(table,0,tableLen * sizeof(bool));
    ObjList* commaSplit = cond.split(',',false);
    for (ObjList* o = commaSplit->skipNull();o;o = o->skipNext()) {
	String* s = static_cast<String*>(o->get());
	if (s->find('-') == -1) {
	    int i = s->toInteger(-1);
	    if (i < 0 || (unsigned int)i >= tableLen) {
		Debug(DebugNote,"Invalid index %d in condition %s",i,cond.c_str());
		continue;
	    }
	    table[i] = true;
	    continue;
	}
	ObjList* min = s->split('-',false);
	if (min->count() != 2) {
	    Debug(DebugNote,"Invalid range count %d for cmd %s",min->count(),cond.c_str());
	    TelEngine::destruct(min);
	    continue;
	}
	int start = (static_cast<String*>((*min)[0]))->toInteger(-1);
	int end   = (static_cast<String*>((*min)[1]))->toInteger(-1);
	TelEngine::destruct(min);
	if (start < 0 || (unsigned int)start >= tableLen) {
	    Debug(DebugNote,"Invalid range interval start %d for cmd %s",start,cond.c_str());
	    TelEngine::destruct(min);
	    continue;
	}
	if (end < start || (unsigned int)end >= tableLen) {
	    Debug(DebugNote,"Invalid range interval start %d end %d tableLen %d for cmd %s",start,end,tableLen,cond.c_str());
	    TelEngine::destruct(min);
	    continue;
	}
	for (int i = start;i <= end;i++)
	    table[i] = true;
    }
    TelEngine::destruct(commaSplit);
}

void setInt(int* table, unsigned int tableLen, const String& cond, int value)
{
    ObjList* commaSplit = cond.split(',',false);
    for (ObjList* o = commaSplit->skipNull();o;o = o->skipNext()) {
	String* s = static_cast<String*>(o->get());
	if (s->find('-') == -1) {
	    int i = s->toInteger(-1);
	    if (i < 0 || (unsigned int)i >= tableLen) {
		Debug(DebugNote,"Invalid index %d in condition %s",i,cond.c_str());
		continue;
	    }
	    table[i] = value;
	    continue;
	}
	ObjList* min = s->split('-',false);
	if (min->count() != 2) {
	    Debug(DebugNote,"Invalid range count %d for cmd %s",min->count(),cond.c_str());
	    TelEngine::destruct(min);
	    continue;
	}
	int start = (static_cast<String*>((*min)[0]))->toInteger(-1);
	int end   = (static_cast<String*>((*min)[1]))->toInteger(-1);
	TelEngine::destruct(min);
	if (start < 0 || (unsigned int)start >= tableLen) {
	    Debug(DebugNote,"Invalid range interval start %d for cmd %s",start,cond.c_str());
	    TelEngine::destruct(min);
	    continue;
	}
	if (end < start || (unsigned int)end >= tableLen) {
	    Debug(DebugNote,"Invalid range interval start %d end %d tableLen %d for cmd %s",start,end,tableLen,cond.c_str());
	    TelEngine::destruct(min);
	    continue;
	}
	for (int i = start;i <= end;i++)
	    table[i] = value;
    }
    TelEngine::destruct(commaSplit);
}

//
// TrafficShower
//
TrafficShower::TrafficShower(bool in,unsigned int arfcn)
    : m_show(false), m_in(in), m_arfcn(arfcn), m_modulus(51)
{
    ::memset(*m_table,0,8 * 102 * sizeof(bool));
}


// Parse the list of arguments
// must be: on/off timeslot/s devider devider range
void TrafficShower::parse(const ObjList& args)
{
    ObjList* o = args.skipNull();
    if (!o)
	return;
    String* onoff = static_cast<String*>(o->get());
    m_show = onoff->toBoolean();
    if (!m_show)
	return;
    o = o->skipNext();
    if (!o) { // Show all
	::memset(*m_table,1,8 * 102 * sizeof(bool));
	return;
    }
    bool ts[8];
    setBool(ts,8,*(static_cast<String*>(o->get())));
    o = o->skipNext();
    if (!o) {
	for (unsigned int i = 0;i < 8;i++) {
	    if (!ts[i])
		continue;
	    ::memset(m_table[i],102,102 * sizeof(bool));
	}
	return;
    }
    m_modulus = (static_cast<String*>(o->get()))->toInteger(-1);
    if (m_modulus < 0 || m_modulus > 102) {
	Debug(DebugNote,"Invalid Modulus %d switching to 51",m_modulus);
	m_modulus = 51;
    }
    o = o->skipNext();

    String* r = 0;
    if (o)
	r = static_cast<String*>(o->get());
    for (unsigned int i = 0;i < 8;i++) {
	if (!ts[i])
	    continue;
	if (r)
	    setBool(m_table[i],102,*r);
	else
	    ::memset(m_table[i],102,102 * sizeof(bool));
    }
}

void TrafficShower::show(GSMBurst* burst)
{
    if (!m_show || !burst)
	return;
    int mod = burst->time().fn() % m_modulus;
    if (!m_table[burst->time().tn()][mod])
	return;
    Debug(DebugNote,"%s burst, ARFCN %d, Time (%d:%d), Modulus %d",
	  m_in ? "Received" : "Sending",m_arfcn,burst->time().fn(),
	  burst->time().tn(),mod);
}

//
// TrafficShaper
//
TrafficShaper::TrafficShaper(ARFCN* arfcn, unsigned int index)
    : m_arfcn(arfcn)
{
    for (unsigned int i = 0;i < 8;i++)
	m_table[i] = index;
    //::memset(m_table,index,8 * sizeof(int));
}

GSMTxBurst* TrafficShaper::getShaped(const GSMTime& t)
{
    if (!m_arfcn)
	return 0;
    int sh = m_table[t.tn()];
    if (sh == (int)m_arfcn->arfcn())
	return 0;
    if (sh >= 0 && sh < 4)
	return GSMTxBurst::getCopy(m_arfcn->transceiver()->arfcn(sh)->getFiller(t));
    // Negative values special case
    // -1 filler burst
    if (sh == -1) {
	GSMTxBurst* d = GSMTxBurst::buildFiller();
	if (!d)
	    return 0;
	d->buildTxData(m_arfcn->transceiver()->signalProcessing());
	return d;
    }
    // SH == -2 void air
    if (sh == -2)
	return GSMTxBurst::getVoidAirBurst(m_arfcn->transceiver()->signalProcessing());
    if (sh == -3)
	return GSMTxBurst::getCopy(m_arfcn->transceiver()->getTxTestBurst());
    if (sh == -4) {
	uint8_t tmp[154];
	::memset(tmp,0,154);
	GSMTxBurst* d =  GSMTxBurst::parse(tmp,154);
	if (!d)
	    return 0;
	d->buildTxData(m_arfcn->transceiver()->signalProcessing());
	return d;
    }
    return 0;
}

void TrafficShaper::parse(ObjList* args)
{
    if (!m_arfcn || !args)
	return;
    ObjList* o = args->skipNull();
    if (!o)
	return;
    unsigned int shape = m_arfcn->arfcn();
    if (o->count() >= 2)
	shape = (static_cast<String*>((*o)[1]))->toInteger();
    if (shape == m_arfcn->arfcn())
	return;
    String* ts = static_cast<String*>(o->get());
    setInt(m_table,8,*ts,shape);
}

//
// TxFillerTable
//
// Initialize the filler table
void TxFillerTable::init(unsigned int len, GSMTxBurst* filler)
{
    clear();
    m_filler = filler;
    m_fillers.assign(len >= FILLER_FRAMES_MIN ? len : FILLER_FRAMES_MIN);
    for (unsigned int i = 0; i < m_fillers.length(); i++) {
	GSMTxBurstPtrVector& v = m_fillers[i];
	v.assign(8);
	v.fill(m_filler);
    }
}

void TxFillerTable::clear()
{
    for (unsigned int i = 0; i < m_fillers.length(); i++) {
	GSMTxBurstPtrVector& v = m_fillers[i];
	for (unsigned int j = 0; j < v.length(); j++)
	    if (v[j] != m_filler)
		TelEngine::destruct(v[j]);
    }
    m_fillers.clear();
    TelEngine::destruct(m_filler);
}

GSMTxBurst** TxFillerTable::fillerHolder(const GSMTime& time)
{
    unsigned int modulus = m_owner ? m_owner->getFillerModulus(time.tn()) : 102;
    if (modulus > m_fillers.length())
	modulus = m_fillers.length();
    return &(m_fillers[time.fn() % modulus][time.tn()]);
}

//
// ARFCN
//
ARFCN::ARFCN(unsigned int index)
    : m_mutex(false,"ARFCN"),
    m_rxQueue(24,"ARFCNRx"),
    m_radioInThread(0),
    m_chans(0),
    m_rxBursts(0),
    m_rxDroppedBursts(0),
    m_averegeNoiseLevel(0),
    m_rxTraffic(true,index),
    m_txTraffic(false,index),
    m_arfcn(index),
    m_fillerTable(this),
    m_txMutex(false,"ARFCNTx"),
    m_shaper(this,index)
{
    for (uint8_t i = 0; i < 8; i++) {
	m_slots[i].slot = i;
	m_slots[i].type = ChanNone;
	m_slots[i].burstType = BurstNone;
    }
}

ARFCN::~ARFCN()
{
    stop();
}

// Dump chan type
void ARFCN::dumpChanType(String& buf, const char* sep)
{
    for (int i = 0; i < 8; i++) {
	const char* s = chanType(m_slots[i].type);
	buf.append(s ? s : String(m_slots[i].type).c_str(),sep);
    }
}

// Extract expired bursts from this ARFCN's queue.
// Move them to filler table
void ARFCN::moveBurstsToFillers(const GSMTime& time, bool warnExpired)
{
    unsigned int expired = 0;
    Lock lck(m_txMutex);
    for (ObjList* o = m_expired.skipNull(); o; o = o->skipNull()) {
	GSMTxBurst* b = static_cast<GSMTxBurst*>(o->get());
	if (!b->filler())
	    expired++;
	m_fillerTable.set(static_cast<GSMTxBurst*>(o->remove(false)));
    }
    for (ObjList* o = m_txQueue.skipNull(); o; o = o->skipNull()) {
	GSMTxBurst* b = static_cast<GSMTxBurst*>(o->get());
	if (b->time() >= time)
	    break;
	m_fillerTable.set(static_cast<GSMTxBurst*>(o->remove(false)));
	expired++;
    }
    if (expired && warnExpired)
	Debug(this,DebugNote,"%s%u burst(s) expired at fn=%u tn=%u [%p]",
	    prefix(),expired,time.fn(),time.tn(),this);
}

// Initialize a timeslot and related data
void ARFCN::initTimeslot(unsigned int slot, int t)
{
    if (slot > 7 || m_slots[slot].type == t)
	return;
    ArfcnSlot& s = m_slots[slot];
    s.type = t;
    Debug(this,DebugAll,"%sSlot %u type set to %d '%s' [%p]",
	prefix(),slot,t,chanType(t),this);
    setSlotBurstType(s,getChanBurstType(t));
    s.warnRecvDrop = true;
    switch (t) {
	case ChanNone:
	case ChanI:
	case ChanII:
	case ChanIII:
	case ChanFill:
	case ChanIGPRS:
	    s.modulus = 26;
	    break;
	case ChanIV:
	case ChanV:
	case ChanVI:
	    s.modulus = 51;
	    break;
	case ChanVII:
	    s.modulus = 102;
	    break;
	default:
	    s.modulus = 26;
	    Debug(this,DebugStub,"ARFCN::initTimeslot() unhandled chan type %u",t);
    }
    uint8_t n = 0;
    for (int i = 0; i < 8; i++)
	if (m_slots[i].type != ChanNone)
	    n++;
    m_chans = n;
}

// Start the ARFCN, stop it if already started
bool ARFCN::start()
{
    DBGFUNC_TRXOBJ("ARFCN::start()",this);
    stop();
    return true;
}

// Stop the ARFCN
void ARFCN::stop()
{
    DBGFUNC_TRXOBJ("ARFCN::stop()",this);
    radioPowerOff();
}

// Initialize/start radio power on related data
bool ARFCN::radioPowerOn(String* reason)
{
    m_rxQueue.clear();
    Lock lck(m_mutex);
    if (!TrxWorker::create(m_radioInThread,TrxWorker::ARFCNRx,this))
	return false;
    m_rxBursts = 0;
    m_rxDroppedBursts = 0;
    return true;
}

// Stop radio power on related data
void ARFCN::radioPowerOff()
{
    m_mutex.lock();
    TrxWorker::cancelThreads(this,0,&m_radioInThread);
    int level = DebugInfo;
    String tmp;
    if (m_rxBursts) {
	if (m_rxDroppedBursts)
	    level = DebugNote;
	tmp << " RX: bursts=" << m_rxBursts << " dropped=" << m_rxDroppedBursts;
	m_rxBursts = 0;
    }
    if (tmp)
	Debug(transceiver(),level,"%sRadio power off.%s [%p]",prefix(),tmp.c_str(),this);
    m_mutex.unlock();
    m_rxQueue.clear();
}

// Forward a burst to upper layer
bool ARFCN::recvBurst(GSMRxBurst*& burst)
{
    if (!burst)
	return true;
    Debug(this,DebugStub,"ARFCN::recvBurst() not implemented [%p]",this);
    return false;
}

void ARFCN::addBurst(GSMTxBurst* burst)
{
    if (!burst)
	return;
    GSMTime txTime;
    transceiver()->getTxTime(txTime);
    Lock myLock(m_txMutex);

    if (burst->filler() || txTime > burst->time()) {
	if (!burst->filler()) {
	    Debug(this,DebugNote,
		"%sReceived delayed burst fn=%u tn=%u current: fn=%u tn=%u [%p]",
		prefix(),burst->time().fn(),burst->time().tn(),
		txTime.fn(),txTime.tn(),this);
	}
	m_expired.insert(burst);
	return;
    }
    GSMTime tmpTime = txTime;
    tmpTime.advance(204,0);
    if (burst->time() > tmpTime) {
	if (debugAt(DebugNote)) {
	    uint32_t f;
	    uint8_t n;
	    GSMTime::diff(f,n,burst->time().fn(),burst->time().tn(),
		txTime.fn(),txTime.tn());
	    Debug(this,DebugNote,"%sReceived burst too far in future diff: fn=%u tn=%u [%p]",
		prefix(),f,n,this);
	}
	m_expired.insert(burst);
	return;
    }
    ObjList* last = &m_txQueue;
    for (ObjList* o = m_txQueue.skipNull();o;o = o->skipNext()) {
	GSMTxBurst* b = static_cast<GSMTxBurst*>(o->get());

	int comp = GSMTime::compare(b->time(),burst->time());
	
	if (comp > 0) {
	    XDebug(this,DebugAll,"%sInsert burst fn=%u tn=%u [%p]",
		prefix(),burst->time().fn(),burst->time().tn(),this);
	    o->insert(burst);
	    return;
	}
	if (comp == 0) {
	    Debug(this,DebugAll,"%sDuplicate burst received fn=%u tn=%u [%p]",
		prefix(),b->time().fn(),b->time().tn(),this);
	    TelEngine::destruct(burst);
	    return;
	}
	last = o;
    }
    last->append(burst);
}

GSMTxBurst* ARFCN::getBurst(const GSMTime& time)
{
    Lock myLock(m_txMutex);
    for (ObjList* o = m_txQueue.skipNull(); o; o = o->skipNext()) {
	GSMTxBurst* b = static_cast<GSMTxBurst*>(o->get());
	if (b->time() > time)
	    continue;
	o->remove(false);
	return b;
    }
    return 0;
}

// Process received radio data
void ARFCN::recvRadioData(RadioRxData* d)
{
    if (!d)
	return;
    m_rxBursts++;
    if (d->m_data.length() < ARFCN_RXBURST_LEN) {
	TelEngine::destruct(d);
	dropRxBurst("invalid length",d->m_time,d->m_data.length(),DebugFail,true,d);
	return;
    }
    XDebug(this,DebugInfo,"%sEnqueueing Rx burst (%p) TN=%u FN=%u len=%u [%p]",
	prefix(),d,d->m_time.tn(),d->m_time.fn(),d->m_data.length(),this);
    if (m_rxQueue.add(d,transceiver()))
	return;
    if (thShouldExit(transceiver()))
	TelEngine::destruct(d);
    else
	dropRxBurst("queue full",d->m_time,d->m_data.length(),DebugFail,true,d);
}

// Worker terminated notification
void ARFCN::workerTerminated(Thread* th)
{
    if (!th)
	return;
    Lock lck(TrxWorker::s_mutex);
    if (m_radioInThread == th)
	m_radioInThread = 0;
}

// Run radio input data process loop
void ARFCN::runRadioDataProcess()
{
    if (!transceiver())
	return;
    transceiver()->waitPowerOn();
    GenObject* gen = 0;
    GSMRxBurst* burst = 0;
    while (m_rxQueue.waitPop(gen,transceiver())) {
	if (!gen)
	    continue;
	RadioRxData* d = static_cast<RadioRxData*>(gen);
	gen = 0;
	XDebug(this,DebugAll,"%sDequeued Rx burst (%p) TN=%u FN=%u len=%u [%p]",
	    prefix(),d,d->m_time.tn(),d->m_time.fn(),d->m_data.length(),this);
	if (d->m_time.tn() > 7) {
	    dropRxBurst("timeslot outside range",d->m_time,d->m_data.length(),
		DebugFail,true,d);
	    continue;
	}
	if (!burst)
	    burst = new GSMRxBurst;
	burst->time(d->m_time);
	burst->m_data.steal(d->m_data);
	TelEngine::destruct(d);
	ArfcnSlot& slot = m_slots[burst->time().tn()];
	if (!transceiver()->processRadioBurst(arfcn(),slot,*burst) ||
	    recvBurst(burst))
	    continue;
	transceiver()->fatalError();
	break;
    }
    TelEngine::destruct(gen);
    TelEngine::destruct(burst);
}

void ARFCN::dumpBursts()
{
    Lock myLock(m_txMutex);
    for (ObjList* o = m_txQueue.skipNull();o;o = o->skipNext()) {
	GSMTxBurst* b = static_cast<GSMTxBurst*>(o->get());
	Debug(this,DebugAll,"Queued burst arfcn %u fn %u tn %u [%p]",
	    m_arfcn,b->time().fn(),b->time().tn(),b);
    }
}

// Retrieve the channel type dictionary
const TokenDict* ARFCN::chanTypeName()
{
    return s_gsmSlotType;
}

// Retrieve the burst type dictionary
const TokenDict* ARFCN::burstTypeName()
{
    return s_gsmBurstType;
}

// Set timeslot access burst type
void ARFCN::setSlotBurstType(ArfcnSlot& slot, int val)
{
    if (slot.burstType == val)
	return;
    if (transceiver() && transceiver()->state() == Transceiver::PowerOn)
	Debug(this,DebugAll,"%sSlot %u burst type changed %s -> %s [%p]",
	    prefix(),slot.slot,burstType(slot.burstType),burstType(val),this);
    slot.burstType = val;
}

int ARFCN::getChanBurstType(int chanType)
{
    switch (chanType) {
	case ChanI:
	case ChanIII:
	case ChanIGPRS:
	    return BurstNormal;
	case ChanIV:
	case ChanVI:
	    return BurstAccess;
	case ChanII:
	case ChanV:
	case ChanVII:
	case ChanLoopback:
	    return BurstCheck;
	case ChanFill:
	    return BurstIdle;
	case ChanNone:
	    return BurstNone;
    }
    Debug(DebugStub,"ARFCN::getChanBurstType() unhandled chan type %u",chanType);
    return BurstUnknown;
}

int ARFCN::burstTypeCheckFN(int chanType, unsigned int fn)
{
    switch (chanType) {
	case ChanII:
	    return (fn % 2) ? BurstIdle : BurstNormal;
	case ChanV:
	    fn = fn % 51;
	    if ((14 <= fn && fn <= 36) || fn == 4 || fn == 5 || fn == 45 || fn == 46)
		return BurstAccess;
	    return BurstNormal;
	case ChanVII:
	    fn = fn % 51;
	    return (fn < 12 || fn > 14) ? BurstNormal : BurstIdle;
	case ChanLoopback:
	    fn = fn % 51;
	    return (fn < 48) ? BurstNormal : BurstIdle;
    }
    Debug(DebugStub,"ARFCN::burstTypeCheckFN() unhandled chan type %u",chanType);
    return BurstNone;
}

void ARFCN::dropRxBurst(const char* reason, const GSMTime& t, unsigned int len, int level,
    bool debug, RadioRxData* d)
{
    bool fatal = (level == DebugFail);
#ifdef DEBUG
    debug = true;
#endif
    if (debug && level >= 0 && reason)
	Debug(this,level,"%sDropping Rx burst TN=%u FN=%u len=%u: %s [%p]",
	    prefix(),t.tn(),t.fn(),len,reason,this);
    m_rxDroppedBursts++;
    TelEngine::destruct(d);
    if (fatal && transceiver())
	transceiver()->fatalError();
}

void ARFCN::addAverageNoise(float current)
{
    m_averegeNoiseLevel += (current - m_averegeNoiseLevel) / AVEREGE_NOISE_LENGTH;
}

void ARFCN::dumpSlotsDelay(String& dest)
{
    dest << "ARFCN[" << m_arfcn << "][";
    for (unsigned int i = 0;i < 8;i++) {
	dest << "s:";
	dest << i;
	dest << ";ds:";
	SigProcUtils::appendFloat(dest,m_slots[i].delay,"");
	if (i != 7)
	    dest << ";";
    }
    dest << "];";
}

//
// ARFCNSocket
//
ARFCNSocket::ARFCNSocket(unsigned int index)
    : ARFCN(index), m_data("data",false,5),
    m_ctrl("control"),
    m_dataReadThread(0)
{
}

ARFCNSocket::~ARFCNSocket()
{
    stop();
}

// Start the ARFCN, stop it if already started
// Start the processing threads
bool ARFCNSocket::start()
{
    DBGFUNC_TRXOBJ("ARFCNSocket::start()",this);
    stop();
    if (!ARFCN::start())
	return false;
    Lock lck(m_mutex);
    bool ok = true;
    // Bind the sockets, start workers
    for (int i = 0; ok && i < 2; i++) {
	bool data = (i != 0);
	TransceiverSockIface& ifc = (data ? m_data : m_ctrl);
	ok = ifc.initSocket(*this);
    }
    lck.drop();
    if (!ok)
	stop();
    return ok;
}

// Stop the ARFCN.
// Stop the processing threads
void ARFCNSocket::stop()
{
    DBGFUNC_TRXOBJ("ARFCNSocket::stop()",this);
    radioPowerOff();
    m_mutex.lock();
    m_ctrl.terminate();
    m_mutex.unlock();
    ARFCN::stop();
}

// Initialize/start radio power on related data
bool ARFCNSocket::radioPowerOn(String* reason)
{
    if (!ARFCN::radioPowerOn(reason))
	return false;
    Lock lck(m_mutex);
    if (!TrxWorker::create(m_dataReadThread,TrxWorker::ARFCNData,this))
	return false;
    return true;
}

// Stop radio power on related data
void ARFCNSocket::radioPowerOff()
{
    ARFCN::radioPowerOff();
    m_mutex.lock();
    TrxWorker::cancelThreads(this,0,&m_dataReadThread);
    m_data.terminate();
    m_mutex.unlock();
}

// Forward a burst to upper layer
bool ARFCNSocket::recvBurst(GSMRxBurst*& burst)
{
    if (!burst)
	return true;
    burst->fillEstimatesBuffer();
    return m_data.writeSocket(burst->m_bitEstimate,ARFCN_RXBURST_LEN,*this) >= 0;
}

// Read socket loop
void ARFCNSocket::runReadDataSocket()
{
    if (!transceiver())
	return;
    transceiver()->waitPowerOn();
    FloatVector tmpV;
    ComplexVector tmpW;
    while (!thShouldExit(transceiver())) {
	int r = m_data.readSocket(*this);
	if (r < 0)
	    return;
	if (!r)
	    continue;
	DataBlock tmp;
	GSMTxBurst* burst = GSMTxBurst::parse(m_data.m_readBuffer.data(0),r,&tmp);
	if (!burst)
	    continue;
	// Transform (modulate + freq shift)
	burst->buildTxData(transceiver()->signalProcessing(),&tmpV,&tmpW,tmp);
	tmp.clear(false);
	m_txTraffic.show(burst);
	addBurst(burst);
    }
}

// Worker terminated notification
void ARFCNSocket::workerTerminated(Thread* th)
{
    if (!th)
	return;
    Lock lck(TrxWorker::s_mutex);
    if (m_dataReadThread == th)
	m_dataReadThread = 0;
    else {
	lck.drop();
	ARFCN::workerTerminated(th);
    }
}

// Initialize UDP socket
bool ARFCNSocket::initUDP(bool data, int rPort, const char* rAddr, int lPort,
    const char* lAddr)
{
    Lock lck(m_mutex);
    TransceiverSockIface& i = data ? m_data : m_ctrl;
    return i.setAddr(true,lAddr,lPort,*this) && i.setAddr(false,rAddr,rPort,*this);
}


//
// RadioIOData
//
// Fill with zero samples, advance in buffer
unsigned int RadioIOData::fill(unsigned int samples)
{
    unsigned int room = free();
    if (samples > room)
	samples = room;
    if (samples) {
	::memset(buffer(),0,samples * 2 * sizeof(int16_t));
	advance(samples);
    }
    return samples;
}

// Copy samples, advance in buffer
unsigned int RadioIOData::copy(int16_t* buf, unsigned int samples)
{
    if (!buf)
	return 0;
    unsigned int room = free();
    if (samples > room)
	samples = room;
    if (samples) {
	::memcpy(buffer(),buf,samples * 2 * sizeof(int16_t));
	advance(samples);
    }
    return samples;
}


//
// RadioIface
//
// Constructor
RadioIface::RadioIface()
    : m_mutex(false,"RadioIface"),
    m_samplesPerSymbol(1), m_readThread(0), m_txPowerScale(1.0), m_rxData(true),
    m_readTimeout(0), m_writeTimeout(0) , m_powerOn(false), m_clockMutex(false,"RadioClock"),
    m_txBufferSpace(2,0), m_testMin(0), m_testMax(0), m_testIncrease(0),
    m_testValue(0), m_loopback(false), m_loopbackSleep(0), m_loopbackNextSend(0),
    m_loopbackArray(0), m_showClocks(false)
{
    m_txSynchronizer.lock();
#ifdef CALLGRIND_CHECK
    m_loopback = true;
    m_loopbackSleep = 1000;
#endif
}

// Initialize radio
bool RadioIface::init(const NamedList& params)
{
    Lock lck(m_mutex);
    if (m_powerOn)
	return true;
    m_samplesPerSymbol = getUInt(params,YSTRING("samples_per_symbol"));
    if (transceiver())
	m_clock = transceiver()->startTime();
    // I/O errors check
    unsigned int m = getUInt(params,YSTRING("io_errors_max"),30,2);
    unsigned int b = getUInt(params,YSTRING("io_errors_inc"),10,1);
    m_rxErrors.init(m,b);
    m_txErrors.init(m,b);
    // NOTE: A frame has 156.25 symbols so 8 timeslots = 1250 symbols
    m_rxData.resetInt16(initialReadTs(),1250 * m_samplesPerSymbol);
#ifdef DEBUG
    if (debugAt(DebugAll)) {
	String tmp;
	tmp << "\r\nsamples_per_symbol=" << m_samplesPerSymbol;
	Debug(this,DebugAll,"%sInitialized [%p]\r\n-----%s\r\n-----",
	    prefix(),this,tmp.c_str());
    }
#endif
    return true;
}

// Set Tx power
int RadioIface::setTxPower(double dB)
{
    double val = 0;
    int ok = radioSetGain(false,-dB,&val);
    if (ok)
	return ok;
    float f = (float)(dB + val);
    float tmp = ::powf(10.0,0.1 * f);
    m_txPowerScale = tmp < 1 ? 1 : (1.0 / ::sqrt(tmp));
    Debug(this,DebugInfo,"%sSet Tx power=%g gain=%g power_scaling=%g [%p]",
	prefix(),dB,f,m_txPowerScale,this);
    return 0;
}

// Start the radio
bool RadioIface::powerOn()
{
    Lock lck(m_mutex);
    if (m_powerOn)
	return true;
    m_powerOn = radioPowerOn();
    if (!m_powerOn)
	return false;
    if (TrxWorker::create(m_readThread,TrxWorker::RadioRead,this))
	return true;
    lck.drop();
    powerOff();
    return false;
}

// Stop the radio
void RadioIface::powerOff()
{
    m_mutex.lock();
    bool show = m_powerOn;
    m_powerOn = false;
    unsigned int wait = m_readTimeout ? (m_readTimeout + 50) : 0;
    TrxWorker::cancelThreads(this,wait,&m_readThread);
    radioPowerOff();
    m_mutex.unlock();
    if (!show)
	return;
    String tmp;
    tmp << " rx ts=" << m_rxData.timestamp();
    Debug(this,DebugAll,"%sPowered off%s [%p]",prefix(),tmp.c_str(),this);
}

// Read radio loop
void RadioIface::runReadRadio()
{
    if (!transceiver())
	return;
    transceiver()->waitPowerOn();
    GSMTime time(m_clock);
    time.decTn(3);
    unsigned int shortBufLen = BITS_PER_TIMESLOT * m_samplesPerSymbol;
#if 1
    m_rxData.resetInt16(initialReadTs(),shortBufLen);
    while (!thShouldExit(transceiver())) {
	unsigned int& rd = shortBufLen;
	// Read samples from radio until the buffer is full
	while (m_rxData.pos() < rd) {
	    if (m_showClocks) {
		m_showClocks = false;
		GSMTime bt,rt,diff;
		u_int64_t btime = getBoardTimestamp();
		uint fd = shortBufLen * 8;
		bt.assign(btime / fd,(btime % fd) / shortBufLen);
		rt.assign(m_rxData.timestamp() / fd,(m_rxData.timestamp() % fd) / shortBufLen);
		GSMTime::diff(diff,bt,rt);
		Debug(this,DebugNote,"\nBoard time FN %d TN %d unused %d\nRadio time FN %d TN %d unused %d\nDiff %d %d\nRadio Clock FN %d TN %d",
		    bt.fn(),bt.tn(),(int)(btime % shortBufLen),rt.fn(),rt.tn(),(int)(m_rxData.timestamp() % shortBufLen),diff.fn(),diff.tn(),m_clock.fn(),m_clock.tn());
	    }
	    unsigned int samples = rd - m_rxData.pos();
	    if (!readRadio(m_rxData,&samples)) {
		Alarm(this,"system",DebugWarn,"%sRadio read fatal error [%p]",
		    prefix(),this);
		if (transceiver())
		    transceiver()->fatalError();
		return;
	    }
	    if (thShouldExit(transceiver()))
		return;
	}
	RadioRxData* r = new RadioRxData(time);
	r->m_data.assign(rd);
	Complex::setInt16(r->m_data.data(),rd,m_rxData.data(),rd * 2);
	if (!m_loopback)
	    transceiver()->recvRadioData(r);
	else
	    TelEngine::destruct(r);
	m_clockMutex.lock();
	m_clock.incTn();
	m_clockMutex.unlock();
	time.incTn();
	m_rxData.consumed(rd);
	signalSendTx();
    }
#else
    while (!thShouldExit(transceiver())) {
	// Read samples from radio until the buffer is full
	while (!m_rxData.full()) {
	    if (!readRadio(m_rxData)) {
		Alarm(this,"system",DebugWarn,"%sRadio read fatal error [%p]",
		    prefix(),this);
		if (transceiver())
		    transceiver()->fatalError();
		return;
	    }
	    if (thShouldExit(transceiver()))
		return;
	}
	int16_t* b = m_rxData.data();
	unsigned int consumed = m_rxData.samples();
	while (consumed) {
	    bool shortBuf = ((time.tn() % 4) != 0);
	    unsigned int& n = shortBuf ? shortBufLen : longBufLen;
	    if (consumed < n)
		break;
	    unsigned int bLen = n * 2;
	    RadioRxData* r = new RadioRxData(time);
	    r->m_data.assign(n);
	    Complex::setInt16(r->m_data.data(),n,b,bLen);
	    if (!m_loopback)
		transceiver()->recvRadioData(r);
	    else
		TelEngine::destruct(r);
	    time.incTn();
	    b += bLen;
	    consumed -= n;
	    Lock lck(m_clockMutex);
	    m_clock.incTn();
	    lck.drop();
	    signalSendTx();
	}
	m_rxData.consumed(m_rxData.samples() - consumed);
	signalSendTx();
    }
#endif
}

// Worker terminated notification
void RadioIface::workerTerminated(Thread* th)
{
    if (!th)
	return;
    Lock lck(TrxWorker::s_mutex);
    if (m_readThread == th)
	m_readThread = 0;
}

// Retrieve the initial read timestamp
uint64_t RadioIface::initialReadTs()
{
    return 1;
}

// Retrieve the initial write timestamp
uint64_t RadioIface::initialWriteTs()
{
    return 1;
}

// Energize a float value
static inline float energize(float cv, float div,unsigned int* clamp)
{
    if (div != 1)
	cv *= div;
    static unsigned int s_energizer = 2047; // 2^11 - 1
    float ret = 0;
    if (cv >= 1) {
	ret = s_energizer;
	(*clamp)++;
    } else if (cv <= -1) {
	ret = -s_energizer;
	(*clamp)++;
    } else
	ret = cv * s_energizer;
    return ret;
}

// Send data to radio
bool RadioIface::sendData(const ComplexVector& data, const GSMTime& time)
{
    // Special case for loopback
    if (m_loopback) {
	if (m_loopbackNextSend > Time::now()) {
	    Thread::idle();
	    return true;
	}
	m_loopbackNextSend = Time::now() + m_loopbackSleep;
	GSMTime t = time;
	const ComplexVector* out = &data;
	int outLen = data.length();
	if (m_loopbackArray) {
	    if (m_loopbackArray->length() == 1251) {
		t.assign((*m_loopbackArray)[1250].real(),(*m_loopbackArray)[1250].imag());
		outLen = 1250;
	    }
	    // TODO add Loopback time
	    out = m_loopbackArray;
	}

	RadioRxData* r = new RadioRxData(t);
	r->m_data.assign(data.length());
	Lock myLock(m_mutex);
	unsigned int end = outLen;
	if (data.length() < end)
	    end = data.length();
	for (unsigned int i = 0;i < end;i++) {
	    r->m_data[i].set((*out)[i].real(),(*out)[i].imag());
	}
	transceiver()->recvRadioData(r);
	return true;
    }
    float f = m_txPowerScale / transceiver()->getConfArfcns();
    bool haveDumper = s_dumper;
    if (m_testIncrease == 0 && !haveDumper)
	return writeRadio(data,f);

    int16_t fillValue = 0;
    if (haveDumper) {
	switch (time.tn() % 4) {
	    case 0:
		fillValue = 0;
		break;
	    case 1:
		fillValue = 2047;
		break;
	    case 2:
		fillValue = 0;
		break;
	    case 3:
		fillValue = -2047;
		break;
	}
    }
    static ComplexVector out(data.length());
    for (unsigned int i = 0;i < out.length();i++) {
	if (haveDumper) {
	    out[i].set(fillValue,fillValue);
	    continue;
	}
	out[i].set(m_testValue,m_testValue);
	
	m_testValue += m_testIncrease;
	if (m_testValue > m_testMax)
	    m_testValue = m_testMin;
    }
    return writeRadio(out,f);
}

bool RadioIface::waitSendTx()
{
    unsigned int wait = 3 * Thread::idleUsec();
    while (!(m_txSynchronizer.lock(wait) || thShouldExit(transceiver())))
	;
    return !thShouldExit(transceiver());
}

void RadioIface::signalSendTx()
{
    m_txSynchronizer.unlock();
}

// Retrieve device factory data
int RadioIface::getFactoryValue(const String& name, String& value)
{
    if (!name)
	return 1;
    Debug(this,DebugStub,"%sUnknown factory value '%s' [%p]",prefix(),name.c_str(),this);
    value = "0";
    return 0;
}

// Execute a command
int RadioIface::command(const String& cmd, String* rspParam, String* reason)
{
    if (!cmd)
	return Transceiver::CmdEOk;
    if (cmd == YSTRING("dump-clocks")) {
	m_showClocks = true;
	return Transceiver::CmdEOk;
    }
    return Transceiver::CmdEUnkCmd;
}

/* vi: set ts=8 sw=4 sts=4 noet: */

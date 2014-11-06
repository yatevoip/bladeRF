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

// Dump data when entering in qmf() function
//#define TRANSCEIVER_DUMP_QMF_IN
// Dump data after applying QMF frequency shifting
//#define TRANSCEIVER_DUMP_QMF_FREQSHIFTED
// Dump QMF half band filter data
//#define TRANSCEIVER_DUMP_QMF_HALFBANDFILTER
// Dump input data on ARFCN process start / exit
//#define TRANSCEIVER_DUMP_ARFCN_PROCESS_IN
//#define TRANSCEIVER_DUMP_ARFCN_PROCESS_OUT


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

ObjList TrxWorker::s_threads;
Mutex TrxWorker::s_mutex(false,"TrxWorkers");

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
};

static const TokenDict s_cmdName[] = {
    {"HANDOVER",    CmdHandover},
    {"NOHANDOVER",  CmdNoHandover},
    {"SETSLOT",     CmdSetSlot},
    {"SETPOWER",    CmdSetPower},
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
    {"QH",   0xffffffff, (float)(-0.749149017394489)},
    {"QLL",  0xffffffff, (float)(-0.821647309400407)},
    {"QLH",  0xffffffff, (float)(-2.31994534418939)},
    {"QHL",  0xffffffff, (float)(-0.821647309400407)},
    {"QHH",  0xffffffff, (float)(-2.31994534418939)},
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
    return trx && (trx->exiting() || trx->inError());
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
    if (m_max) {
	while (!m_free.lock(Thread::idleUsec()))
	    if (thShouldExit(trx))
		return false;
    }
    m_mutex.lock();
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
    String tmp;
    Complex::dump(tmp,c,len);
    Debug(this,DebugAll,"%s%s TN=%u FN=%u len=%u:%s [%p]",
	prefix(),TelEngine::c_safe(str),time.tn(),time.fn(),len,
	tmp.safe(),this);
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
	    String tmp;
	    if (m_text)
		tmp.assign((const char*)buf,len);
	    else
		tmp.hexify((void*)buf,len,' ');
	    Debug(&dbg,DebugAll,"%sSocket(%s) sent %u bytes: %s [%p]",
		dbg.prefix(),name().c_str(),len,tmp.c_str(),&dbg);
#endif
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
    m_rxQueue(8,"TrxRxQueue"),
    m_oversamplingRate(1),
    m_burstMinPower(0),
    m_arfcn(0),
    m_arfcnCount(0),
    m_clockIface("clock"),
    m_startTime((uint32_t)Random::random()),
    m_clockUpdMutex(false,"TrxClockUpd"),
    m_txLatency(2),
    m_fillers(0),
    m_fillerMaxFrames(100),
    m_tsc(0),
    m_rxFreq(0),
    m_txFreq(0),
    m_freqOffset(0),
    m_txPower(-10),
    m_txAttnOffset(0),
    m_error(false),
    m_exiting(false)
{
    setTransceiver(*this,name);
    DDebug(this,DebugAll,"Transceiver() [%p]",this);
}

Transceiver::~Transceiver()
{
    DDebug(this,DebugAll,"~Transceiver() [%p]",this);

    stop();
    clearFillers();
    // NOTE: If you need to use m_arfcnCount use it before this line!
    // resetARFCNs will set it to 0.
    resetARFCNs();
    TelEngine::destruct(m_radio);
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
    clearFillers();
    reInit(params);
    bool ok = false;
    while (true) {
	m_oversamplingRate = getUInt(params,"oversampling");
	setBurstMinPower(params.getDoubleValue(YSTRING("burst_min_power")));
	unsigned int arfcns = getUInt(params,"arfcns");
	const String* rAddr = params.getParam(YSTRING("remoteaddr"));
	m_fillerMaxFrames = params.getIntValue(YSTRING("filler_frames"),m_fillerMaxFrames);
	unsigned int tableSize = arfcns * m_fillerMaxFrames * 8;
	m_fillers = new GSMTxBurst*[tableSize];
	for (unsigned int i = 0; i < tableSize; i++)
	    m_fillers[i] = 0;
	m_signalProcessing.initialize(m_oversamplingRate,arfcns);
	int port = rAddr ? params.getIntValue(YSTRING("port")) : 0;
	const char* lAddr = rAddr ? params.getValue(YSTRING("localaddr"),*rAddr) : 0;
	if (rAddr &&
	    !(m_clockIface.setAddr(true,lAddr,port,*this) &&
	    m_clockIface.setAddr(false,*rAddr,port + 100,*this)))
	    break;
	if (!resetARFCNs(arfcns,port,rAddr,lAddr))
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
    m_freqOffset = params.getIntValue(YSTRING("RadioFrequencyOffset"),128,96,160);
    setDebugLevel(this,params.getParam(YSTRING("debug_level")));
    setDebugLevel(m_radio,params.getParam(YSTRING("debug_radio_level")));
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
	    txTime.advance(diff.fn(),diff.tn());
	}
	while (rTime > txTime) {
	    if (sendBurst(txTime)) {
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

void Transceiver::addFillerBurst(GSMTxBurst* burst)
{
    if (!burst)
	return;
    if (burst->arfcn() >= m_arfcnCount || burst->time().tn() > 8) {
	Debug(this,DebugConf,
	    "Invalid request for addFillerBurst conf arfcn %u, Burst: arfcn: %u tn: %u",
	    m_arfcnCount,burst->arfcn(),burst->time().tn());
	TelEngine::destruct(burst);
	return;
    }
    unsigned int tmpFN = burst->time().fn() % m_fillerMaxFrames;
    unsigned int pos = burst->arfcn() + m_arfcnCount * (tmpFN + m_fillerMaxFrames * burst->time().tn());
    XDebug(this,DebugAll,"Adding filler burst to [%u] [%p], %p",
	pos,m_fillers[pos], burst);
    TelEngine::destruct(m_fillers[pos]);
    m_fillers[pos] = burst;
}

GSMTxBurst* Transceiver::getFiller(unsigned int arfcn, const GSMTime& time)
{
    if (arfcn >= m_arfcnCount || time.tn() > 7) {
	Debug(this,DebugConf,
	    "Invalid request for getFiller conf arfcn %u, Burst: arfcn: %u tn: %u",
	    m_arfcnCount,arfcn,time.tn());
	return 0;
    }
    unsigned int fn = time.fn() % m_fillerMaxFrames;
    unsigned int pos = arfcn + m_arfcnCount * (fn + m_fillerMaxFrames * time.tn());
    XDebug(this,DebugAll,"getFiller for %u returning [%p]",pos,m_fillers[pos]);
    return m_fillers[pos];
}

bool Transceiver::sendBurst(GSMTime& time)
{
    XDebug(this,DebugAll,"Request send burst time fn:%u tn:%u",time.fn(),time.tn());
    // Extract the expired bursts and add them to the filler table
    for (unsigned int i = 0; i < m_arfcnCount; i++) {
	ObjList expired;
	m_arfcn[i]->extractExpired(expired,time);
	ObjList* o = expired.skipNull();
	if (o)
	    Debug(this,DebugNote,"ARFCN %u: %d bursts expired! Time fn:%u tn:%u",
		i,expired.count(),time.fn(),time.tn());
	for (; o; o = o->skipNull()) {
	    GSMTxBurst* burst = static_cast<GSMTxBurst*>(o->remove(false));
	    addFillerBurst(burst);
	}
    }

    unsigned int sendLength = (unsigned int)(BITS_PER_TIMESLOT * m_oversamplingRate);
    if (m_oversamplingRate % 4 != 0)
	sendLength ++;

    ComplexArray toSend(sendLength);
    bool addFiller = true;
    for (unsigned int i = 0; i < m_arfcnCount; i++) {
	GSMTxBurst* burst = m_arfcn[i]->getBurst(time);
	if (!burst) {
	    burst = getFiller(i,time);
	    addFiller = false;
	}
	if (!burst)
	    continue;
	const ComplexArray* transformed = burst->transform(m_signalProcessing);
	if (!transformed) {
	    Debug(this,DebugStub,"Unable to transform GSMTxBurst! Bug??");
	    if (addFiller)
		TelEngine::destruct(burst);
	    continue;
	}
	toSend.sum(*transformed);

	if (addFiller)
	    addFillerBurst(burst);
    }
#ifdef XDEBUG
    String dump;
    toSend.dump(dump);
    //Debug(this,DebugAll,"Send tx data : %s",dump.c_str());
#endif
    return m_radio->sendData(toSend);
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
    DDebug(this,DebugAll,"Handling command '%s' arfcn=%u [%p]",str,arfcn,this);
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
	case CmdRxTune:
	case CmdTxTune:
	    status = handleCmdTune(c == CmdRxTune,arfcn,s,&rspParam);
	    break;
	case CmdSetTsc:
	    status = handleCmdSetTsc(arfcn,s,&rspParam);
	    break;
	case CmdSetMaxDelay:
	    DDebug(this,DebugStub,"Set max delay command [%p]",this);
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
	default:
	    status = CmdEUnkCmd;
    }
    if (status)
	Debug(this,DebugNote,"Command '%s' (ARFCN=%u) failed in state %s: %d %s [%p]",
	    str,arfcn,trxStateName(state()),status,
	    reason.safe(lookup(status,s_cmdErrorName,"failure")),this);
    syncGSMTime();
    return buildCmdRsp(rsp,cmd,status,rspParam);
}

// Set the minimum power to accept radio bursts
void Transceiver::setBurstMinPower(float val)
{
    if (m_burstMinPower == val)
	return;
    m_burstMinPower = val;
    Debug(this,DebugInfo,"Burst minimum power level set to %f [%p]",val,this);
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
    const char* lAddr)
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
	    m_arfcn[i] = new ARFCNSocket;
	else
	    m_arfcn[i] = new ARFCN;
	m_arfcn[i]->m_arfcn = i;
	m_arfcn[i]->setTransceiver(*this,"ARFCN[" + String(i) + "]");
	m_arfcn[i]->debugChain(this);
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

// Clear filler table
void Transceiver::clearFillers()
{
    if (!m_fillers)
	return;
    unsigned int len = m_arfcnCount * m_fillerMaxFrames * 8;
    for (unsigned int i = 0; i < len; i++)
	TelEngine::destruct(m_fillers[i]);
    delete[] m_fillers;
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
	if (m_clockIface.writeSocket(tmp.c_str(),tmp.length(),*this) > 0)
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
	static bool s_stub = true;
	if (s_stub) {
	    s_stub = false;
	    Debug(DebugStub,"Please check Transceiver::handleCmdTune()");
	}
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

// Handle CUSTOM command. Return status code
int Transceiver::handleCmdCustom(String& cmd, String* rspParam, String* reason)
{
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
    m_halfBandFltCoeffLen(19),
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
	    tmp = "unhandled ";
	    tmp << " burst type (" << bType << "," << ARFCN::burstType(bType) << ")";
	    tmp << " slot_type " << a->chanType(slot.type);
	}
	a->dropRxBurst(tmp,t,len,level);
	return false;
    }
    if (len < ARFCN_RXBURST_LEN)
	return false;
#ifdef TRANSCEIVER_DUMP_ARFCN_PROCESS_IN
    a->dumpRecvBurst("processRadioBurst()",t,b.m_data.data(),len);
#else
    XDebug(a,DebugAll,"%sprocessRadioBurst() TN=%u FN=%u len=%u burst_type=%s [%p]",
	a->prefix(),t.tn(),t.fn(),len,ARFCN::burstType(bType),a);
#endif
    // Channel estimator
    unsigned int inIdx = 0;
    ComplexVector* s = 0;
    ComplexVector chanEstimate;
    if (bType == ARFCN::BurstNormal) {
	s = &m_nbTSC[m_tsc];
	inIdx = (ARFCN_RXBURST_LEN - m_tscSamples) / 2;
	chanEstimate.resize(m_tscSamples);
    }
    else {
	s = &m_abSync;
	inIdx = 8;
	chanEstimate.resize(m_abSync.length());
    }
    if (inIdx + chanEstimate.length() > ARFCN_RXBURST_LEN) {
	a->dropRxBurst("invalid channel estimator lengths",t,len,DebugGoOn);
	return false;
    }
    Complex* he = chanEstimate.data();
    Complex* x = b.m_data.data() + inIdx;
    unsigned int xLen = ARFCN_RXBURST_LEN - inIdx;
    for (unsigned int n = chanEstimate.length(); n; n--)
	Complex::sumMul(*he++,x++,xLen--,s->data(),s->length());
    // Demodulate
    // We will use the conjugate of channel estimator array
    Complex::conj(chanEstimate.data(),chanEstimate.length());
    unsigned int halfHe = chanEstimate.length() / 2;
    b.m_bitEstimate.resize(ARFCN_RXBURST_LEN);
    float* w = b.m_bitEstimate.data();
    for (unsigned int i = 0; i < b.m_bitEstimate.length(); i++) {
	Complex c;
	// w[i] = SUM(j=0..chanEstimate.length() - 1) x[i + j - halhHe] * CONJ(he[j])
	x = b.m_data.data();
	he = chanEstimate.data();
	if (i >= halfHe) {
	    unsigned int idx = i - halfHe;
	    Complex::sumMul(c,x + idx,len - idx,he,chanEstimate.length());
	}
	else {
	    unsigned int skip = halfHe - i;
	    Complex::sumMul(c,x,len,he + skip,chanEstimate.length() - skip);
	}
	*w++ = c.real();
    }
    // Calculate the power level
    b.m_powerLevel = SignalProcessing::computePower(b.m_bitEstimate.data(),
	b.m_bitEstimate.length(),5,0.2);
    if (!b.m_powerLevel) {
	a->dropRxBurst("zero power",t,len,DebugNote,false);
	return false;
    }
    // Set the soft bit estimate in real part of output
    float sqrtPower = ::sqrtf(b.m_powerLevel);
    unsigned int n = b.m_bitEstimate.length();
    for (w = b.m_bitEstimate.data(); n; n--, w++)
	*w = (((*w / sqrtPower) + 1) / 2);
#ifdef TRANSCEIVER_DUMP_ARFCN_PROCESS_OUT
    String tmpExit;
    n = b.m_bitEstimate.length();
    for (w = b.m_bitEstimate.data(); n; n--, w++) {
	tmpExit << " ";
        tmpExit.append(*w,3);
    }
    Debug(a,DebugAll,"%sprocessRadioBurst() exit. power=%g RSSI=%g data:%s [%p]",
	a->prefix(),b.m_powerLevel,b.m_timingError,tmpExit.c_str(),a);
#endif
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
	    float omega = 0.54 - 0.46 * ::cosf(2 * PI * offset / lq_1);
	    if (offset) {
		float func = PI / 2 * offset;
		*d++ = omega * (::sinf(func) / func);
	    }
	    else
		*d++ = omega;
	}
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

// Process received radio data
void TransceiverQMF::processRadioData(RadioRxData* d)
{
    if (!d)
	return;
    XDebug(this,DebugAll,"Processing radio input TN=%u FN=%u len=%u [%p]",
	d->m_time.tn(),d->m_time.fn(),d->m_data.length(),this);
    m_qmf[0].data.steal(d->m_data);
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
#ifdef XDEBUG
    String tmp;
    tmp << "QMF[" << index << "] ";
    Debugger dbg(DebugAll,tmp.c_str(),"len=%u low=%d high=%d [%p]",
	crt.data.length(),indexLo,indexHi,this);
#endif
#ifdef TRANSCEIVER_DUMP_QMF_IN
    String tmp1;
    tmp1 << "QMF[" << index << "] starting";
    dumpRecvBurst(tmp1,time,crt.data.data(),crt.data.length());
#endif
    // Frequency shift
    qmfApplyFreqShift(crt);
#ifdef TRANSCEIVER_DUMP_QMF_FREQSHIFTED
    String tmp2;
    tmp2 << "QMF[" << index << "] applied frequency shifting";
    dumpRecvBurst(tmp2,time,crt.data.data(),crt.data.length());
#endif
    // Forward data to ARFCNs
    if (final) {
	RadioRxData* r = new RadioRxData(time);
	r->m_data.steal(crt.data);
	XDebug(this,DebugAll,"Forwarding radio data ARFCN=%u len=%u TN=%u FN=%u [%p]",
	    crt.arfcn,r->m_data.length(),time.tn(),time.fn(),this);
	m_arfcn[crt.arfcn]->recvRadioData(r);
	return;
    }
    // Build the half band filter, generate low/high band data and call QMF again
    qmfBuildHalfBandFilter(crt);
#ifdef TRANSCEIVER_DUMP_QMF_HALFBANDFILTER
    String tmp3;
    tmp3 << "QMF[" << index << "] built half band filter";
    dumpRecvBurst(tmp3,time,crt.halfBandFilter.data(),crt.halfBandFilter.length());
#endif
    if (indexLo >= 0 && !thShouldExit(this)) {
	qmfBuildOutputLowBand(crt,m_qmf[indexLo].data);
	qmf(time,indexLo);
    }
    if (indexHi >= 0 && !thShouldExit(this)) {
	qmfBuildOutputHighBand(crt,m_qmf[indexHi].data);
	qmf(time,indexHi);
    }
}

// Build the half band filter
// Behave as the input vector is prepended with n0 zero samples
// E.g. The data index 0 is at index -n0
// x: the input data, kept in b.data
// w: half band filter data, kept in b.halfBandFilter
// h: half band filter coefficients, kept in m_halfBandFltCoeff
// w[i] = SUM[j=0..n0](x[i + 2 * j] * h[2 * j])
void TransceiverQMF::qmfBuildHalfBandFilter(QmfBlock& b)
{
    if (b.halfBandFilter.length() < b.data.length())
	b.halfBandFilter.resize(b.data.length() + 10);
    unsigned int n0 = (m_halfBandFltCoeff.length() - 1) / 2;
    unsigned int n0_2 = n0 * 2;
    Complex* x = b.data.data();
    Complex* w = b.halfBandFilter.data();
    float* h = m_halfBandFltCoeff.data();
    unsigned int rest = b.data.length();
    for (unsigned int i = 0; rest; ++i, ++w, --rest) {
	w->set();
	int offs = (int)i - (int)n0;
	unsigned int j = 0;
	// Start processing when (offs + j) is at least 0
	// We ignore negative indexes (zero samples): multiplication would be 0 anyway
	// E.g x[i + 2 * j] is (0,0)
	// j will start with the number of steps ignored
	if (offs < 0) {
	    j = ((unsigned int)(-offs) + 1) / 2;
	    if (j > n0_2)
		continue;
	}
	unsigned int n = (n0_2 - j + 2) / 2;
	unsigned int bIters = (rest + 1) / 2;
	if (n > bIters)
	    n = bIters;
	Complex* d = x + i;
	float* f = h + j;
	for (; n ; d += 2, f += 2, --n)
	    Complex::sumMulF(*w,*d,*f);
    }
}

// Build the QMF low band output
// Behave as the input vector is prepended with n0 zero samples
// x: the input data, kept in b.data
// w: half band filter data, kept in b.halfBandFilter
// y[i] = x[2 * i + 2 * n0] + w[2 * i]
void TransceiverQMF::qmfBuildOutputLowBand(QmfBlock& b, ComplexVector& y)
{
    y.resize(b.data.length() / 2);
    unsigned int n0 = (m_halfBandFltCoeff.length() - 1) / 2;
    unsigned int n0_2 = n0 * 2;
    Complex* yData = y.data();
    Complex* x = b.data.data();
    Complex* w = b.halfBandFilter.data();
    // 2 * i + 2 * (i - n0) = 4 * i - 2 * n0
    // E.g. for the first n0 / 2 elements adding 0 to w is useless
    unsigned int i = 0;
    unsigned int n = n0 / 2;
    if (n > y.length())
	n = y.length();
    for (; i < n; i++, yData++, w += 2)
	*yData = *w;
    for (; i < y.length(); i++, yData++, w += 2) {
	unsigned int idx = 4 * i - n0_2;
	if (idx >= b.data.length())
	    break;
	Complex::sum(*yData,x[idx],*w);
    }
    for (; i < y.length(); i++, yData++, w += 2)
	*yData = *w;
}

// Build the QMF high band output
// Behave as the input vector is prepended with n0 zero samples
// x: the input data, kept in b.data
// w: half band filter data, kept in b.halfBandFilter
// y[i] = x[2 * i + 2 * n0] - w[2 * i]
void TransceiverQMF::qmfBuildOutputHighBand(QmfBlock& b, ComplexVector& y)
{
    y.resize(b.data.length() / 2);
    unsigned int n0 = (m_halfBandFltCoeff.length() - 1) / 2;
    unsigned int n0_2 = n0 * 2;
    Complex* yData = y.data();
    Complex* x = b.data.data();
    Complex* w = b.halfBandFilter.data();
    // 2 * i + 2 * (i - n0) = 4 * i - 2 * n0
    // E.g. for the first n0 / 2 elements the data is zero
    unsigned int i = 0;
    unsigned int n = n0 / 2;
    if (n > y.length())
	n = y.length();
    for (; i < n; i++, yData++, w += 2)
	yData->set(-w->real(),-w->imag());
    for (; i < y.length(); i++, yData++, w += 2) {
	unsigned int idx = 4 * i - n0_2;
	if (idx >= b.data.length())
	    break;
	Complex::diff(*yData,x[idx],*w);
    }
    for (; i < y.length(); i++, yData++, w += 2)
	yData->set(-w->real(),-w->imag());
}

void TransceiverQMF::initNormalBurstTSC(unsigned int len)
{
    if (!len || len > GSM_NB_TSC_LEN)
	len = 16;
    const int8_t* table = GSMUtils::nbTscTable();
    table += (GSM_NB_TSC_LEN - len) / 2;
    for (unsigned int i = 0; i < 8; i++, table += GSM_NB_TSC_LEN) {
	const int8_t* p = table;
	m_nbTSC[i].resize(len);
	Complex* c = m_nbTSC[i].data();
	for (unsigned int n = len; n; n--, c++, p++)
	    c->set(2 * *p - 1);
    }
}

void TransceiverQMF::initAccessBurstSync()
{
    m_abSync.resize(GSM_AB_SYNC_LEN);
    const int8_t* p = GSMUtils::abSyncTable();
    Complex* c = m_abSync.data();
    for (unsigned int n = GSM_AB_SYNC_LEN; n; n--, c++, p++)
	c->set(2 * *p - 1);
}


//
// ARFCN
//
ARFCN::ARFCN()
    : m_mutex(false,"ARFCN"),
    m_rxQueue(16,"ARFCNRx"),
    m_radioInThread(0),
    m_chans(0),
    m_rxBursts(0),
    m_rxDroppedBursts(0),
    m_arfcn(0),
    m_txMutex(false,"ARFCNTx")
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

    if (burst->isFiller() || txTime > burst->time()) {
	if (!burst->isFiller()) {
	    Debug(this,DebugNote,
		"%sReceived delayed burst arfcn:%u burst fn %u tn %u, TxTime fn:%u tn:%u",
		prefix(),m_arfcn,burst->time().fn(),burst->time().tn(),txTime.fn(),txTime.tn());
	}
	m_expired.insert(burst);
	return;
    }
    ObjList* last = &m_txQueue;
    for (ObjList* o = m_txQueue.skipNull();o;o = o->skipNext()) {
	GSMTxBurst* b = static_cast<GSMTxBurst*>(o->get());

	int comp = GSMTime::compare(b->time(),burst->time());
	
	if (comp > 0) {
	    XDebug(this,DebugAll,"%sInsert burst FN=%u TN=%u [%p]",
		prefix(),burst->time().fn(),burst->time().tn(),this);
	    o->insert(burst);
	    return;
	}
	if (comp == 0) {
	    Debug(this,DebugAll,"%sDuplicate burst received FN=%u TN=%u [%p]",
		prefix(),b->time().fn(),b->time().tn(),this);
	    TelEngine::destruct(burst);
	    return;
	}
	last = o;
    }
    last->append(burst);
}

void ARFCN::extractExpired(ObjList& dest, const GSMTime& time)
{
    Lock myLock(m_txMutex);
    for (ObjList* o = m_expired.skipNull();o;o = o->skipNull())
	dest.append(o->remove(false));

    for (ObjList* o = m_txQueue.skipNull();o;o = o->skipNull()) {
	GSMTxBurst* b = static_cast<GSMTxBurst*>(o->get());
	if (GSMTime::compare(b->time(),time) >= 0)
	    return;
	o->remove(false);
	dest.append(b);
    }
}

GSMTxBurst* ARFCN::getBurst(const GSMTime& time)
{
    Lock myLock(m_txMutex);
    for (ObjList* o = m_txQueue.skipNull();o;o = o->skipNext()) {
	GSMTxBurst* b = static_cast<GSMTxBurst*>(o->get());
	int comp = GSMTime::compare(b->time(),time);
	if (comp > 0)
	    return 0;
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
	dropRxBurst("invalid length",d->m_time,d->m_data.length(),DebugFail,true,d);
	return;
    }
    if (transceiver() &&
	!transceiver()->checkBurstMinPower(d->m_data.data(),ARFCN_RXBURST_LEN)) {
	dropRxBurst("low power",d->m_time,d->m_data.length(),DebugNote,false,d);
	return;
    }
    XDebug(this,DebugAll,"%sEnqueueing Rx burst (%p) TN=%u FN=%u len=%u [%p]",
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


//
// ARFCNSocket
//
ARFCNSocket::ARFCNSocket()
    : m_data("data",false,5),
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
#define ARFCN_BURST_BUFLEN (8 + GSM_BURST_LENGTH)
    int8_t buf[ARFCN_BURST_BUFLEN];
    unsigned int n = burst->buildEstimatesBuffer(buf,ARFCN_BURST_BUFLEN);
    if (n < ARFCN_BURST_BUFLEN)
	::memset(&buf[n],0,ARFCN_BURST_BUFLEN - n);
    return m_data.writeSocket(buf,ARFCN_BURST_BUFLEN,*this) >= 0;
#undef ARFCN_BURST_BUFLEN
}

// Read socket loop
void ARFCNSocket::runReadDataSocket()
{
    if (transceiver())
	transceiver()->waitPowerOn();
    while (!thShouldExit(transceiver())) {
	int r = m_data.readSocket(*this);
	if (r < 0)
	    return;
	if (!r)
	    continue;
	GSMTxBurst* burst = GSMTxBurst::get(m_data.m_readBuffer,r);
	if (!burst)
	    continue;
	burst->arfcn(arfcn());
	burst->transform(transceiver()->signalProcessing());
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
    m_samplesPerSymbol(1),
    m_readThread(0),
    m_txPowerScale(1.0),
    m_rxData(true),
    m_txData(false),
    m_powerOn(false),
    m_clockMutex(false,"RadioClock"),
    m_txBufferSpace(2,0)
{
    m_txSynchronizer.lock();
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
    m_txData.resetInt16(initialWriteTs(),1250 * m_samplesPerSymbol);
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
    unsigned int wait = m_rxData.syncIOWait() ? (m_rxData.syncIOWait() + 50) : 0;
    TrxWorker::cancelThreads(this,wait,&m_readThread);
    radioPowerOff();
    m_mutex.unlock();
    if (!show)
	return;
    String tmp;
    tmp << " tx ts=" << m_txData.timestamp();
    tmp << " future=" << ((int64_t)m_txData.timestamp() - (int64_t)m_rxData.timestamp());
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
    unsigned int shortBufLen = 156 * m_samplesPerSymbol;
    unsigned int longBufLen = 157 * m_samplesPerSymbol;
#if 1
    m_rxData.resetInt16(initialReadTs(),longBufLen);
    while (!thShouldExit(transceiver())) {
	bool shortBuf = ((time.tn() % 4) != 0);
	unsigned int& rd = (shortBuf ? shortBufLen : longBufLen);
	// Read samples from radio until the buffer is full
	while (m_rxData.pos() < rd) {
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
	transceiver()->recvRadioData(r);
	m_clockMutex.lock();
	m_clock.incTn();
	m_clockMutex.unlock();
	time.incTn();
	m_rxData.consumed(rd);
	if (!shortBuf)
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
	    transceiver()->recvRadioData(r);
	    time.incTn();
	    b += bLen;
	    consumed -= n;
	    Lock lck(m_clockMutex);
	    m_clock.incTn();
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
static inline float energize(float cv)
{
    static float s_energizer = 32767; // 2^15 - 1
    if (cv >= 1)
	return s_energizer;
    if (cv <= -1)
	return -s_energizer;
    return cv * s_energizer;
}

// Send data to radio
bool RadioIface::sendData(const ComplexArray& data)
{
    // NOTE: We are assuming this is done from a single thread
    // E.g. There is no send buffer protection
    unsigned int len = data.length();
    const Complex* c = data.data();
    if (!(c && len))
	return true;
    float f = m_txPowerScale;
    while (len && m_txData.free()) {
	// Apply power scaling
	int16_t* b = m_txData.buffer();
	unsigned int n = len;
	if (n > m_txData.free())
	    n = m_txData.free();
	len -= n;
	m_txData.advance(n);
	if (f == 1.0) {
	    for (; n; n--, c++) {
		*b++ = (int16_t)energize(c->real());
		*b++ = (int16_t)energize(c->imag());
	    }
	}
	else
	    for (; n; n--, c++) {
		*b++ = (int16_t)energize(c->real() * f);
		*b++ = (int16_t)energize(c->imag() * f);
	    }
	if (!m_txData.full())
	    return true;
	if (!writeRadio(m_txData))
	    return false;
    }
    if (!len || thShouldExit(transceiver()))
	return true;
    Debug(this,DebugWarn,
	"%sFailed to send %u samples: not enough space in buffer free=%u [%p]",
	prefix(),len,m_txData.free(),this);
    return false;
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
    return Transceiver::CmdEUnkCmd;
}

/* vi: set ts=8 sw=4 sts=4 noet: */

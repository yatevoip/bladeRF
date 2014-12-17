/**
 * tranceiver.h
 * This file is part of the Yate-BTS Project http://www.yatebts.com
 *
 * Tranceiver related classes
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

#ifndef TRANSCEIVER_H
#define TRANSCEIVER_H

#include <yateclass.h>
#include "gsmutil.h"
#include "sigproc.h"

namespace TelEngine {

class GenQueue;                          // A queue of GenObject
class TransceiverObj;                    // A tranceiver related object
class TransceiverSockIface;              // Transceiver socket interface
class Transceiver;                       // A transceiver
class TransceiverQMF;                    // A QMF transceiver
class ARFCN;                             // A transceiver ARFCN
class ARFCNSocket;                       // A transceiver ARFCN with socket interface
class RadioRxData;                       // Radio read data
class RadioErrors;                       // A radio interface error counter
class RadioIOData;                       // Radio Rx/Tx buffer and related data
class RadioIface;                        // Transceiver radio interface
class TransceiverWorker;                 // Private worker thread


/**
 * This class implements a generic queue
 * @short A generic queue
 */
class GenQueue
{
public:
    /**
     * Constructor
     * @param maxLen Optional maximum queue length
     * @param name Queue name
     */
    GenQueue(unsigned int maxLen = 0, const char* name = "GenQueue");

    /**
     * Check if the queue is empty
     * @return True if the queue is empty
     */
    inline bool empty() const
	{ return m_empty; }

    /**
     * Add an object to the queue.
     * Wait for space to be freed if there is maximum queue length
     * Don't consume the given object on failure
     * @param obj Object to add
     * @param trx Optional transceiver to check for exiting condition
     * @return True on success, false on failure (queue full, calling thread was cancelled or
     *  given transceiver is exiting)
     */
    bool add(GenObject* obj, Transceiver* trx = 0);

    /**
     * Wait for an object to be put in the queue.
     * Extract the first element
     * Returns when there is something in the queue or the calling thread was
     *  cancelled or the given transceiver is exiting
     * @param obj Destination for extracted object
     * @param trx Optional transceiver to check for exiting condition
     * @return True on success, false if the calling thread should exit
     *  (the returned object is always 0 if false is returned)
     */
    bool waitPop(GenObject*& obj, Transceiver* trx = 0);

    /**
     * Clear the queue
     */
    void clear();

private:
    String m_name;

protected:
    /**
     * Pop an object from queue head
     * @return Queue head object, 0 if empty
     */
    GenObject* internalPop();

    Mutex m_mutex;                       // Queue mutex
    Semaphore m_free;                    // 
    Semaphore m_used;                    // 
    ObjList m_data;                      // The queue
    bool m_empty;                        // Queue is empty
    unsigned int m_count;                // The number of objects in queue
    unsigned int m_max;                  // Maximum length
};


/**
 * Base class for objects owned by transceiver
 * @short An object owned by a transceiver
 */
class TransceiverObj : public DebugEnabler , public GenObject
{
public:
    /**
     * Constructor
     */
    TransceiverObj()
	: m_transceiver(0)
	{}

    /**
     * Retrieve the transceiver
     * @return The transceiver owning this object
     */
    inline Transceiver* transceiver() const
	{ return m_transceiver; }

    /**
     * Retrieve the object debug prefix
     * @return Debug prefix
     */
    inline const char* prefix() const
	{ return m_prefix.safe(); }

    /**
     * Set the transceiver
     * @param t The transceiver
     * @param name Extra debug name
     */
    void setTransceiver(Transceiver& t, const char* name);

    /**
     * Worker terminated notification
     * @param th Worker thread
     */
    virtual void workerTerminated(Thread* th);

    /**
     * Dump to output a received burst
     * @param str String to print before data
     * @param time Burst time
     * @param c Pointer to data to dump
     * @param len The number of elements to dump
     * @param dumpShort True to dump the short version
     */
    void dumpRecvBurst(const char* str, const GSMTime& time,
	const Complex* c, unsigned int len, bool dumpShort);

private:
    Transceiver* m_transceiver;          // The transceiver owning this object
    String m_name;                       // Debug name
    String m_prefix;                     // Debug prefix
};


/**
 * Holds socket, addresses and worker (read) thread
 * @short Transceiver socket interface
 */
class TransceiverSockIface
{
public:
    /**
     * Constructor
     * @param name Interface name
     * @param txt True if this a text interface
     * @param writeAttempts Optional write attempts
     */
    inline TransceiverSockIface(const char* name, bool txt = true,
	unsigned int writeAttempts = 1)
	: m_readBuffer(0,1500), m_text(txt),
	m_writeAttempts(writeAttempts ? writeAttempts : 1), m_name(name),
	m_printOne(false)
	{}

    /**
     * Destructor
     */
    ~TransceiverSockIface()
	{ terminate(); }

    /**
     * Retrieve the interface name
     * @return Interface name
     */
    inline const String& name() const
	{ return m_name; }

    /**
     * Set a socket address
     * @param local True to set the local, false to set the remote address
     * @param addr Address to set (can't be all interface address for remote)
     * @param port Port to set
     * @param dbg Debug holder (for debug purposes)
     * @return True on success, false on failure
     */
    bool setAddr(bool local, const char* addr, int port, TransceiverObj& dbg);

    /**
     * Prepare the socket: create, set reuse, bind, set unblocking mode.
     * Don't terminate the socket on failure
     * @param dbg Debug holder (for debug purposes)
     * @return True on success, false on failure
     */
    bool initSocket(TransceiverObj& dbg);

    /**
     * Read from socket. Call Thread::idle() if nothing is read
     * @param dbg Debug holder (for debug purposes)
     * @return The number of bytes read, negative on non retryiable error
     */
    int readSocket(TransceiverObj& dbg);

    /**
     * Write to socket. Put an alarm if dbg is non 0
     * @param buf Buffer to send
     * @param len The number of bytes to send
     * @param dbg Debug holder (for debug purposes)
     * @return The number of bytes wrote, negative on non retryiable error
     */
    int writeSocket(const void* buf, unsigned int len, TransceiverObj& dbg);

    /**
     * Terminate the socket
     * @param linger Linger interval
     */
    inline void terminate(int linger = -1) {
	    if (m_socket.valid()) {
		m_socket.setLinger(linger);
		m_socket.terminate();
	    }
	}

    /**
     * Print one burst sent to the socket.
     */
    inline void printOne()
	{ m_printOne = true; }

    Socket m_socket;                     // The socket
    SocketAddr m_local;                  // Socket local address
    SocketAddr m_remote;                 // Socket remote address
    DataBlock m_readBuffer;              // Socket read buffer

private:
    bool m_text;                         // Text interface
    unsigned int m_writeAttempts;        // Counter for write attempts
    String m_name;
    bool m_printOne;                     // Test flag used to print one packet sent to socket
};


/**
 * This structure holds data for a slot in ARFCN
 * @short An ARFCN slot
 */
struct ArfcnSlot
{
    inline ArfcnSlot()
	: slot(0), type(0), modulus(26), burstType(0), warnRecvDrop(true)
	{}
    uint8_t slot;                        // Timeslot number
    int type;                            // Timeslot (channel) type
    int modulus;                         // Modulus (used to build idle slot filler) in frames
    int burstType;                       // Expected burst type
    bool warnRecvDrop;                   // Warn dropping recv frame
};


/**
 * This class implements the interface to a transceiver
 * @short A transceiver
 */
class Transceiver : public TransceiverObj
{
    YCLASS(Transceiver,GenObject)
    YNOCOPY(Transceiver);
public:
    /**
     * Transceiver state
     */
    enum State {
	Invalid = 0,                     // Init failed, release the object
	Idle,                            // Not started
	PowerOff,                        // Upper layer interface started, radio Power OFF
	PowerOn,                         // Upper layer interface started, radio Power ON
    };

    /**
     * Command failure codes
     */
    enum CmdError {
	CmdEOk = 0,
	CmdEUnkCmd,
	CmdEFailure,
	CmdEInvalidParam,
	CmdEInvalidState,
	CmdEInvalidARFCN,
    };

    /**
     * Constructor
     * @param name Transceiver name
     */
    Transceiver(const char* name = "transceiver");

    /**
     * Destructor
     */
    ~Transceiver();

    /**
     * Retrieve the transceiver state
     * @return Transceiver state
     */
    inline int state() const
	{ return m_state; }

    /**
     * Retrieve the tranceiver start time
     * @return Transceiver start time
     */
    inline GSMTime startTime() const
	{ return m_startTime; }

    /**
     * Retrieve the number of ARFCNs
     * @return The number of ARFCNs
     */
    inline unsigned int arfcnCount() const
	{ return m_arfcnCount; }

    /**
     * Retrieve an ARFCN
     * @return ARFCN pointer or 0
     */
    inline ARFCN* arfcn(unsigned int index) const
	{ return index < m_arfcnCount ? m_arfcn[index] : 0; }

    /**
     * Initialize the transceiver. This method should be called after construction
     * @param radio The radio interface. The transceiver will own of the object
     * @param params Transceiver parameters
     */
    virtual bool init(RadioIface* radio, const NamedList& params);

    /**
     * Re-Initialize the transceiver.
     * Apply parameters that can be changed after construction
     * @param params Parameters list
     */
    virtual void reInit(const NamedList& params);

    /**
     * Wait for PowerOn state
     */
    void waitPowerOn();

    /**
     * Process received radio data
     * @param d Radio data, the pointer will be consumed
     */
    virtual void recvRadioData(RadioRxData* d);

    /**
     * Run radio input data process loop
     */
    virtual void runRadioDataProcess();

    /**
     * Add a Tx Burst to the filler table
     * @param burst The burst to be added.
     * Note: The filler table will own the burst
     */
    void addFillerBurst(GSMTxBurst* burst);

    /**
     * Get a burst from the filler table
     * @param arfcn The arfcn number of the filler burst
     * @param time The time of the filler burst
     * @return 0 if no filler burst is stored for the given arfcn and time.
     */
    GSMTxBurst* getFiller(unsigned int arfcn, const GSMTime& time);

    /**
     * Extract the bursts for sending.
     * Sum them for each ARFCN and send them to the radio interface.
     * @param time The time of the burst that needs to be sent.
     * @return True on success, false on fatal error
     */
    bool sendBurst(GSMTime& time);

    /**
     * Process a received radio burst
     * @param arfcn ARFCN requesting it
     * @param slot The slot receiving the burst
     * @param b The burst
     * @return True on success, false to ignore the burst
     */
    virtual bool processRadioBurst(unsigned int arfcn, ArfcnSlot& slot, GSMRxBurst& b);

    /**
     * Get Tx Time
     * @param time The Tx time
     */
    inline void getTxTime(GSMTime& time) {
	    Lock lck(m_clockUpdMutex);
	    time = m_txTime;
	}

    /**
     * Get the SignallProcessing instance used by this tranceiver
     * @return Reference to SignallProcessing instance
     */
    const SignalProcessing& signalProcessing() const
	{ return m_signalProcessing; }

    /**
     * Start the transceiver upper layer interface if not already done
     * @return True on success
     */
    bool start();

    /**
     * Stop the transceiver upper layer interface, power off the radio
     */
    void stop();

    /**
     * Execute a command
     * @param str Command string
     * @param rsp Optional response
     * @param arfcn Optional ARFCN
     * @return True on success
     */
    bool command(const char* str, String* rsp = 0, unsigned int arfcn = 0);

    /**
     * Check if the transceiver is exiting (stopping)
     * @return True if the transceiver is exiting
     */
    inline bool exiting() const
	{ return m_exiting; }

    /**
     * Check if the transceiver error flag is set
     * @return True if the transceiver error flag is set
     */
    inline bool inError() const
	{ return m_error; }

    /**
     * Set the minimum power to accept radio bursts
     * @param val New minimum power level to accept
     */
    void setBurstMinPower(float val = 0);

    /**
     * Check the power level of a burst
     * @param data Pointer to burst start
     * @param len Burst length
     * @return True if accepted, false to drop it
     */
    inline bool checkBurstMinPower(const Complex* data, unsigned int len) {
	    return !m_burstMinPower ||
		m_burstMinPower <= SignalProcessing::computePower(data,len,5,0.2);
	}

    /**
     * Get the number of configured arfcns
     * @return The number of configured arfcns
     */
    inline unsigned int getConfArfcns() const
	{ return m_arfcnConf; }

    /**
     * Internal fatal error. Set the error flag
     */
    virtual void fatalError();

    /**
     * Run read control socket loop
     */
    void runReadCtrlSocket();

    /**
     * Run send data loop
     */
    void runRadioSendData();

    /**
     * Worker terminated notification
     * @param th Worker thread
     */
    virtual void workerTerminated(Thread* th);

    /**
     * Destroy the object
     */
    virtual void destruct();

    /**
     * Retrieve the state name dictionary
     * @return State name dictionary
     */
    static const TokenDict* dictStateName();

protected:
    /**
     * Process received radio data
     * @param d Radio data, the pointer will be consumed
     */
    virtual void processRadioData(RadioRxData* d) = 0;

    /**
     * Starting radio power on notification
     */
    virtual void radioPowerOnStarting();

    /**
     * Set channel (slot) type
     * @param arfcn ARFCN number
     * @param slot Timeslot number
     * @param chanType Channel type
     */
    virtual void setChanType(unsigned int arfcn, unsigned int slot, int chanType);

    /**
     * ARFCN list changed notification
     */
    virtual void arfcnListChanged();
    
    /**
     * Dump the frequency shift vectors
     * @param index The index of the vector
     */
    virtual void dumpFreqShift(unsigned int index) const
	{ }
    
    int m_state;                         // Transceiver state
    Mutex m_stateMutex;                  // Serialize state change
    Thread* m_ctrlReadThread;            // Worker (read control socket(s)) thread
    RadioIface* m_radio;                 // The radio interface
    Thread* m_radioInThread;             // Worker (process radio input) thread
    Thread* m_radioOutThread;            // Worker (radio feeder) thread
    GenQueue m_rxQueue;                  // Pending radio data
    unsigned int m_oversamplingRate;     // Tranceiver oversampling rate
    float m_burstMinPower;               // Minimum burst power level to accept
    ARFCN** m_arfcn;                     // ARFCNs list
    unsigned int m_arfcnCount;           // The number of ARFCNs
    unsigned int m_arfcnConf;            // The number of configured ARFCNs
    TransceiverSockIface m_clockIface;   // Upper layer clock sync socket
    GSMTime m_startTime;                 // Tranceiver start time
    Mutex m_clockUpdMutex;               // Protect clock update and tx time
    GSMTime m_nextClockUpdTime;          // Next clock update time
    GSMTime m_txTime;                    // Transmit time
    GSMTime m_txLatency;                 // Transmit latency
    GSMTxBurst** m_fillers;              // Table of filler bursts
    GSMTxBurst* m_dummyTxBurst;          // Dummy TX burst
    unsigned int m_fillerMaxFrames;      // Maximum number of frames that the filler table should keep
    SignalProcessing m_signalProcessing; // SignalProcessing class initialized for this tranceiver
    unsigned int m_tsc;                  // GSM TSC index
    double m_rxFreq;                     // Rx frequency
    double m_txFreq;                     // Tx frequency
    int m_freqOffset;                    // Configured frequency offset
    int m_txPower;                       // Tx power level (in dB)
    int m_txAttnOffset;                  // Tx power level attenuation
    // Test data
    int m_alterSend;                     // Flags mask used to send test data.
    GSMTxBurst* m_txTestBurst;           // Test burst to be sent.
    Mutex m_testMutex;                   // Mutex used to block the access to test burst
    bool m_dumpOneTx;                    // Flag used to dump random Tx bursts
    bool m_dumpOneRx;                    // Flag used to dump random Rx bursts

private:
    // Power on the radio if not already done
    bool radioPowerOn(String* reason = 0);
    // Power off the radio if not already done
    void radioPowerOff();
    // Change transceiver state
    void changeState(int newState);
    // Initialize the ARFCNs list (set or release)
    bool resetARFCNs(unsigned int arfcns = 0, int port = 0,
	const String* rAddr = 0, const char* lAddr = 0);
    // Stop all ARFCNs
    void stopARFCNs();
    // Clear filler tables
    void clearFillers();
    // Sync upper layer GSM clock (update time)
    bool syncGSMTime();
    // Handle (NO)HANDOVER commands. Return status code
    int handleCmdHandover(bool on, unsigned int arfcn, String& cmd, String* rspParam);
    // Handle SETSLOT command. Return status code
    int handleCmdSetSlot(unsigned int arfcn, String& cmd, String* rspParam);
    // Handle SETPOWER command. Return status code
    int handleCmdSetPower(unsigned int arfcn, String& cmd, String* rspParam);
    // Handle RXTUNE/TXTUNE commands. Return status code
    int handleCmdTune(bool rx, unsigned int arfcn, String& cmd, String* rspParam);
    // Handle SETTSC commands. Return status code
    int handleCmdSetTsc(unsigned int arfcn, String& cmd, String* rspParam);
    // Handle SETRXGAIN command. Return status code
    int handleCmdSetGain(bool rx, String& cmd, String* rspParam);
    // Handle CUSTOM command. Return status code
    int handleCmdCustom(String& cmd, String* rspParam, String* reason);
    // Handle MANAGETX command. NOTE: this is for test purposes
    int handleCmdManageTx(unsigned int arfcn, String& cmd, String* rspParam);

    bool m_error;                        // Fatal error occured
    bool m_exiting;                      // Stopping flag
};


/**
 * This structure holds data for a QMF block
 * @short A QMF block
 */
struct QmfBlock
{
    inline QmfBlock()
	: chans(false), arfcn(0xffffffff), freqShiftValue(0)
	{}
    bool chans;                          // ARFCN channels availablity in subtree
    unsigned int arfcn;                  // ARFCN
    float freqShiftValue;                // Frequency shift parameter
    ComplexVector data;                  // Input data
    ComplexVector freqShift;             // Frequency shifting vector
    ComplexVector halfBandFilter;        // Used when applying the half band filter 
};


/**
 * This class implements a Quadrature Mirror Filter transceiver
 * @short A QMF transceiver
 */
class TransceiverQMF : public Transceiver
{
    YCLASS(TransceiverQMF,Transceiver)
    YNOCOPY(TransceiverQMF);
public:
    /**
     * Constructor
     * @param name Transceiver name
     */
    TransceiverQMF(const char* name = "transceiver");

    /**
     * Initialize the transceiver. This method should be called after construction
     * @param radio The radio interface. The transceiver will own of the object
     * @param params Transceiver parameters
     */
    virtual bool init(RadioIface* radio, const NamedList& params);

    /**
     * Re-Initialize the transceiver.
     * Apply parameters that can be changed after construction
     * @param params Parameters list
     */
    virtual void reInit(const NamedList& params);

    /**
     * Process a received radio burst
     * @param arfcn ARFCN requesting it
     * @param slot The slot receiving the burst
     * @param b The burst
     * @return True on success, false to ignore the burst
     */
    virtual bool processRadioBurst(unsigned int arfcn, ArfcnSlot& slot, GSMRxBurst& b);

protected:
    /**
     * Process received radio data
     * @param d Radio data, the pointer will be consumed
     */
    virtual void processRadioData(RadioRxData* d);

    /**
     * Starting radio power on notification
     */
    virtual void radioPowerOnStarting();

    /**
     * Set channel (slot) type
     * @param arfcn ARFCN number
     * @param slot Timeslot number
     * @param chanType Channel type
     */
    virtual void setChanType(unsigned int arfcn, unsigned int slot, int chanType);

    /**
     * ARFCN list changed notification
     */
    virtual void arfcnListChanged();

    /**
     * Dump the frequency shift vectors
     * @param index The index of the vector
     */
    virtual void dumpFreqShift(unsigned int index) const;

private:
    // Run the QMF algorithm on node at given index
    void qmf(const GSMTime& time, unsigned int index = 0);
    inline void qmfApplyFreqShift(QmfBlock& b) {
	    if (b.freqShift.length() < b.data.length())
		SignalProcessing::setFreqShifting(&b.freqShift,b.freqShiftValue,
		    b.data.length() + 10);

	    Complex::multiply(b.data.data(),b.data.length(),
		b.freqShift.data(),b.data.length());
	}
    void qmfBuildHalfBandFilter(QmfBlock& b);
    void qmfBuildOutputLowBand(QmfBlock& b, ComplexVector& y);
    void qmfBuildOutputHighBand(QmfBlock& b, ComplexVector& y);
    void initNormalBurstTSC(unsigned int len = 16);
    void initAccessBurstSync();

    QmfBlock m_qmf[15];                  // QMF tree
    unsigned int m_halfBandFltCoeffLen;  // Half band filter coefficients length
    FloatVector m_halfBandFltCoeff;      // Half band filter coefficients vector
    unsigned int m_tscSamples;           // The number of TSC samples used to build the channel estimate
    ComplexVector m_nbTSC[8];            // GSM Normal Burst TSC vectors
    ComplexVector m_abSync;              // Access burst sync vector
};


/**
 * This class implements a transceiver ARFCN
 * @short An ARFCN
 */
class ARFCN : public TransceiverObj
{
    YCLASS(ARFCN,GenObject)
    YNOCOPY(ARFCN);
    friend class Transceiver;
    friend class TransceiverQMF;
public:
    /**
     * Channel type
     */
    enum ChannelType {
	ChanFill,                        // Channel is transmitted, but unused
	ChanI,                           // TCH/FS
	ChanII,                          // TCH/HS, idle every other slot
	ChanIII,                         // TCH/HS
	ChanIV,                          // FCCH+SCH+CCCH+BCCH, uplink RACH
	ChanV,                           // FCCH+SCH+CCCH+BCCH+SDCCH/4+SACCH/4, uplink RACH+SDCCH/4
	ChanVI,                          // CCCH+BCCH, uplink RACH
	ChanVII,                         // SDCCH/8 + SACCH/8
	ChanNone,                        // Channel is inactive
	ChanLoopback,                    // Similar to VII, used in loopback testing
	ChanIGPRS                        // GPRS channel, like I with static filler frames
    };

    /**
     * Expected channel burst type
     */
    enum BurstType {
	BurstNormal,                     // Normal burst (with Training Sequence Code)
	BurstAccess,                     // Access burst (RACH, handover)
	BurstIdle,                       // Idle burst
	BurstNone,                       // Timeslot is inactive
	BurstUnknown,                    // Unknown
	BurstCheck                       // Check slot type and frame number
    };

    /**
     * Constructor
     */
    ARFCN();

    /**
     * Destructor, stop the ARFCN
     */
    ~ARFCN();

    /**
     * Retrieve the channel number
     * @return ARFCN number
     */
    inline unsigned int arfcn() const
	{ return m_arfcn; }

    /**
     * Retrieve the number of configured channels
     * @return The number of configured channels
     */
    inline uint8_t chans() const
	{ return m_chans; }

    /**
     * Dump channels type
     * @param buf Destination buffer
     */
    void dumpChanType(String& buf, const char* sep = ",");

    /**
     * Add a burst to this ARFQN's queue
     * @param burst The burst to add.
     */
    void addBurst(GSMTxBurst* burst);

    /**
     * Extract expired bursts from this ARFQN's queue.
     * @param dest The destination list to be filled with expired bursts.
     * @param time Time base for expire calculation.
     */
    void extractExpired(ObjList& dest, const GSMTime& time);

    /**
     * Obtain the burst that has to be sent at the specified time.
     * @param time The burst time.
     */
    GSMTxBurst* getBurst(const GSMTime& time);

    /**
     * Process received radio data
     * @param d Radio data, the pointer will be consumed
     */
    void recvRadioData(RadioRxData* d);

    /**
     * Worker terminated notification
     * @param th Worker thread
     */
    virtual void workerTerminated(Thread* th);

    /**
     * Run radio input data process loop
     */
    virtual void runRadioDataProcess();

    /**
     * Dump the TxQueue bursts.
     */
    void dumpBursts();

    /**
     * Retrieve the channel type dictionary
     * @return TokenDict pointer
     */
    static const TokenDict* chanTypeName();

    /**
     * Retrieve the string associated with a channel type
     * @param t Channel type
     * @return Requested channel type name
     */
    static inline const char* chanType(int t)
	{ return lookup(t,chanTypeName()); }

    /**
     * Retrieve the burst type dictionary
     * @return TokenDict pointer
     */
    static const TokenDict* burstTypeName();

    /**
     * Retrieve the string associated with a burst type
     * @param t Burst type
     * @return Requested burst type name
     */
    static inline const char* burstType(int t)
	{ return lookup(t,burstTypeName()); }

protected:
    /**
     * Initialize a timeslot and related data
     * @param slot Timeslot number
     * @param chanType Channel type
     */
    void initTimeslot(unsigned int slot, int chanType);

    /**
     * Set timeslot access burst type
     * @param slot Timeslot number
     * @param on True to set access burst type, false to chan type default
     */
    inline void setBurstAccess(unsigned int slot, bool on) {
	    if (slot < 8)
		setSlotBurstType(m_slots[slot],
		    on ? BurstAccess : getChanBurstType(m_slots[slot].type));
	}

    /**
     * Start the ARFCN, stop it if already started
     * @return True on success
     */
    virtual bool start();

    /**
     * Stop the ARFCN
     */
    virtual void stop();

    /**
     * Initialize/start radio power on related data
     * @param reason Optional failure reason
     * @return True on success
     */
    virtual bool radioPowerOn(String* reason = 0);

    /**
     * Stop radio power on related data
     */
    virtual void radioPowerOff();

    /**
     * Forward a burst to upper layer
     * @param burst The burst to process. The pointer must be reset if consumed
     * @return True on success, false on fatal error
     */
    virtual bool recvBurst(GSMRxBurst*& burst);

    Mutex m_mutex;                       // Protect data changes
    GenQueue m_rxQueue;                  // Radio input
    Thread* m_radioInThread;             // Radio input processor
    uint8_t m_chans;                     // The number of channels whose type is not ChanNone
    ArfcnSlot m_slots[8];                // Channels
    uint64_t m_rxBursts;                 // Received bursts
    uint64_t m_rxDroppedBursts;          // Dropped Rx bursts

private:
    void dropRxBurst(const char* reason = 0, const GSMTime& t = GSMTime(),
	unsigned int len = 0, int level = -1, bool debug = true, RadioRxData* d = 0);
    void setSlotBurstType(ArfcnSlot& slot, int val);
    static inline int burstType(ArfcnSlot& slot, unsigned int fn) {
	    if (slot.burstType != BurstCheck)
		return slot.burstType;
	    return burstTypeCheckFN(slot.type,fn);
	}
    static int getChanBurstType(int chanType);
    static int burstTypeCheckFN(int chanType, unsigned int fn);

    unsigned int m_arfcn;                // ARFCN
    Mutex m_txMutex;                     // Transmit queue blocker
    ObjList m_txQueue;                   // Transmit queue
    ObjList m_expired;                   // List of expired bursts
};


/**
 * This class implements a transceiver ARFCN with socket interface
 * @short A transceiver ARFCN with socket interface
 */
class ARFCNSocket : public ARFCN
{
    YCLASS(ARFCNSocket,ARFCN)
    YNOCOPY(ARFCNSocket);
    friend class Transceiver;
public:
    /**
     * Constructor
     */
    ARFCNSocket();

    /**
     * Destructor, stop the ARFCN
     */
    ~ARFCNSocket();

    /**
     * Run read data socket loop
     */
    void runReadDataSocket();

    /**
     * Worker terminated notification
     * @param th Worker thread
     */
    virtual void workerTerminated(Thread* th);

    /**
     * Set the print socket flag
     */
    inline void setPrintSocket()
	{ m_data.printOne(); }

protected:
    /**
     * Start the ARFCN, stop it if already started
     * Start the processing threads
     * @return True on success
     */
    virtual bool start();

    /**
     * Stop the ARFCN.
     * Stop the processing threads
     */
    virtual void stop();

    /**
     * Initialize/start radio power on related data
     * @param reason Optional failure reason
     * @return True on success
     */
    virtual bool radioPowerOn(String* reason = 0);

    /**
     * Stop radio power on related data
     */
    virtual void radioPowerOff();

    /**
     * Forward a burst to upper layer
     * @param burst The burst to process. The pointer must be reset if consumed
     * @return True on success, false on fatal error
     */
    virtual bool recvBurst(GSMRxBurst*& burst);

    /**
     * Initialize UDP addresses
     * @param data True for data socket, false for control socket
     * @param rPort Remote port
     * @param rAddr Remote address
     * @param lPort Local port to bind on
     * @param lAddr Local address to bind on
     */
    bool initUDP(bool data, int rPort, const char* rAddr, int lPort,
	const char* lAddr = "0.0.0.0");

    TransceiverSockIface m_data;         // Data interface
    TransceiverSockIface m_ctrl;         // Control interface
    Thread* m_dataReadThread;            // Worker (read data socket) thread
};


/**
 * This class holds data read from radio device
 * @short Radio read data
 */
class RadioRxData : public GenObject
{
    YCLASS(RadioRxData,GenObject)
    YNOCOPY(RadioRxData);
public:
    /**
     * Constructor
     * @param t Burst GSM time
     */
    inline RadioRxData(const GSMTime& t)
	: m_time(t)
	{}

    GSMTime m_time;
    ComplexVector m_data;
};


/**
 * This class holds RadioIface Rx/Tx error counter
 * @short A radio interface error counter
 */
class RadioErrors
{
public:
    /**
     * Constructor
     */
    inline RadioErrors()
	: m_count(0), m_max(30), m_errorInc(10)
	{}

    /**
     * Initialize data. Reset error counter
     * @param m Maximum number of errors allowed (minimum value: 2)
     * @param inc Error counter increment (it will be set to m_max / inc)
     */
    inline void init(unsigned int m, unsigned int inc) {
	    m_count = 0;
	    m_max = (m > 2) ? m : 2;
	    m_errorInc = (inc && inc < m_max) ? (m_max / inc) : 1;
	}

    /**
     * Reset error counter
     */
    inline void reset()
	{ m_count = 0; }

    /**
     * Update data
     * @param ok True on success, false on error
     * @return True if the counter is less then maximum allowed value, false otherwise
     */
    inline bool update(bool ok) {
	    if (ok) {
		if (m_count)
		    m_count--;
	    }
	    else {
		m_count += m_errorInc;
		if (m_count >= m_max) {
		    m_count = 0;
		    return false;
		}
	    }
	    return true;
	}

protected:
    unsigned int m_count;                // Current number of errors
    unsigned int m_max;                  // Maximum number of errors
    unsigned int m_errorInc;             // Value to increase error counter on failure
};


/**
 * This class holds RadioIface Rx/Tx buffer and related data
 * The buffer keeps samples
 * @short Radio Rx/Tx buffer and related data
 */
class RadioIOData
{
public:
    /**
     * Constructor
     * @param rx True for Rx buffer, false to Tx buffer
     */
    inline RadioIOData(bool rx)
	: m_timestamp(0), m_syncIOWaitMs(0), m_samples(0), m_pos(0),
	m_rx(rx)
	{}

    /**
     * Retrieve I/O timestamp
     * @return I/O timestamp
     */
    inline uint64_t timestamp() const
	{ return m_timestamp; }

    /**
     * Set the I/O timestamp
     * @param ts New I/O timestamp
     */
    inline void timestamp(uint64_t ts)
	{ m_timestamp = ts; }

    /**
     * Retrieve synchronous I/O timeout
     * @return Synchronous I/O timeout in milliseconds
     */
    inline unsigned int syncIOWait() const
	{ return m_syncIOWaitMs; }

    /**
     * Set the synchronous I/O timeout in milliseconds
     * @param val Synchronous I/O timeout in milliseconds
     */
    inline void syncIOWait(unsigned int val)
	{ m_syncIOWaitMs = val; }

    /**
     * Retrieve the buffer start data pointer
     * @return Pointer to buffer data, 0 if not allocated
     */
    inline int16_t* data() const
	{ return (int16_t*)m_buffer.data(); }

    /**
     * Retrieve the buffer data pointer at current position
     * @return Buffer data pointer at current position, 0 if not allocated
     */
    inline int16_t* buffer() const
	{ return data() + m_pos * 2; }

    /**
     * Retrieve the buffer length in samples
     * @return Buffer length in samples
     */
    inline unsigned int samples() const
	{ return m_samples; }

    /**
     * Retrieve buffer I/O position
     * @return Buffer I/O position
     */
    inline unsigned int pos() const
	{ return m_pos; }

    /**
     * Retrieve the number of free samples in buffer
     * @return The number of free samples in buffer
     */
    inline unsigned int free() const
	{ return m_samples - m_pos; }

    /**
     * Check if the buffer is full
     * @return True if the buffer is full
     */
    inline bool full() const
	{ return !free(); }

    /**
     * Consume samples. Adjust position
     * @param samples The number of samples consumed
     */
    inline void consumed(unsigned int samples) {
	    if (m_pos == samples) {
		m_pos = 0;
		return;
	    }
	    Debug(DebugGoOn,"RadioIOData %s buffer consume incomplete",
		(m_rx ? "Rx" : "Tx"));
	    m_pos = 0;
	}

    /**
     * Advance in buffer
     * @param samples The number of samples to advance
     */
    inline void advance(unsigned int samples) {
	    if (!samples)
		return;
	    m_timestamp += samples;
	    m_pos += samples;
	    if (m_pos <= m_samples)
		return;
	    Debug(DebugGoOn,"RadioIOData %s buffer overflow",(m_rx ? "Rx" : "Tx"));
	    m_pos = m_samples;
	}

    /**
     * Fill with zero samples, advance in buffer
     * @param samples The number of samples to advance
     * @return The number of samples filled with 0
     */
    unsigned int fill(unsigned int samples);

    /**
     * Copy samples, advance in buffer
     * @param buf Source buffer
     * @param samples The number of samples to copy
     * @return The number of copied samples
     */
    unsigned int copy(int16_t* buf, unsigned int samples);

    /**
     * Reset the buffer to int16 complex samples
     * @param ts I/O initial timestamp
     * @param samples Buffer length in samples
     *  (the length in bytes will be samples * 2 * sizeof(int16_t))
     */
    inline void resetInt16(uint64_t ts, unsigned int samples) {
	    m_timestamp = ts;
	    m_samples = samples;
	    m_pos = 0;
	    m_buffer.resize(m_samples * 2 * sizeof(int16_t));
	}

protected:
    uint64_t m_timestamp;                // I/O timestamp
    unsigned int m_syncIOWaitMs;         // Synchronous I/O timeout (0: async I/O)
    unsigned int m_samples;              // Buffer length in samples
    unsigned int m_pos;                  // Buffer I/O position
    DataBlock m_buffer;                  // The buffer

private:
    bool m_rx;
};


/**
 * This class implements the transceiver interface to a radio device
 * @short A radio interface
 */
class RadioIface : public TransceiverObj
{
    YCLASS(RadioIface,GenObject)
    YNOCOPY(RadioIface);
public:
    /**
     * Constructor
     */
    RadioIface();

    /**
     * Retrieve synchronous send timeout in milliseconds
     * @return Synchronous send timeout in milliseconds
     */
    inline unsigned int syncWriteWaitMs() const
	{ return m_txData.syncIOWait(); }

    /**
     * Initialize radio
     * @param params Radio parameters
     * @return True on success, false on failure
     */
    virtual bool init(const NamedList& params);

    /**
     * Set Tx power
     * @param dB Power value
     * @return 0 on success, negative on failure, positive on fatal
     *  error (transceiver should stop)
     */
    int setTxPower(double power);

    /**
     * Start the radio
     * @return True on success
     */
    bool powerOn();

    /**
     * Stop the radio
     */
    void powerOff();

    /**
     * Read radio loop
     */
    virtual void runReadRadio();

    /**
     * Worker terminated notification
     * @param th Worker thread
     */
    virtual void workerTerminated(Thread* th);

    /**
     * Retrieve the initial read timestamp. The default implementation returns 1
     * @return Initial read timestamp value
     */
    virtual uint64_t initialReadTs();

    /**
     * Retrieve the initial write timestamp. The default implementation returns 1
     * @return Initial write timestamp value
     */
    virtual uint64_t initialWriteTs();

    /**
     * Send the data to the radio
     * @param data the data to be sent
     * @return True if data was sent, false on fatal error
     */
    virtual bool sendData(const ComplexArray& data);

    /**
     * Get the radio clock
     * @param dest The radio clock to be filled;
     */
    inline void getRadioClock(GSMTime& dest)
	{ Lock myLock(m_clockMutex); dest = m_clock; }

    /**
     * Wait untill we have to push a new buffer to the radio
     * @return True on success, false if the calling thread should exit
     */
    bool waitSendTx();

    /**
     * Signal Tx ready
     */
    void signalSendTx();

    /**
     * Get the tx radio buffer space.
     * @return The tx radio buffer space.
     */
    virtual const GSMTime& getTxBufSpace()
	{ return m_txBufferSpace; }

    /**
     * Run specific tests for the Tx side of the radio interface
     * @param params The list of parameters
     */
    virtual void testTx(NamedList& params)
	{}

    /**
     * Set Rx/Tx frequency
     * @param rx True for Rx, false for Tx
     * @param freq Frequency value in Hertz
     * @param adj Adjust value (ussually the VCTCXO value)
     * @return 0 on success, negative on failure, positive on fatal
     *  error (transceiver should stop)
     */
    virtual int tune(bool rx, double freq, double adj) = 0;

    /**
     * Set Rx/Tx power gain
     * @param rx True to set Rx gain, false to set Tx gain
     * @param dB Power value
     * @param setVal Optional pointer to set value
     * @return 0 on success, negative on failure, positive on fatal
     *  error (transceiver should stop)
     */
    virtual int radioSetGain(bool rx, double dB, double* setVal = 0) = 0;

    /**
     * Retrieve device factory data
     * @param name Data name to retrieve
     * @param value Destination string
     * @return 0 on success, non 0 on failure
     */
    virtual int getFactoryValue(const String& name, String& value);

    /**
     * Execute a command
     * @param cmd Command to execute
     * @param rspParam Optional response
     * @param reason Optional failure reason
     * @return 0 on success, non 0 on failure
     */
    virtual int command(const String& cmd, String* rspParam = 0, String* reason = 0);

    /**
     * Set test data limits. Data will be fabricated after formula:
     * from x = min to x = max : data = (val,val i), val += inc, if (val >= max) val = min
     * The fabricated data will be sent to bladerf 
     * NOTE this is for test
     * @param min The minimum value for input data
     * @param max The maximum value for input data
     * @param inc The increment value.
     * NOTE The data will not be wrapped to 2047
     */
    inline void setTestData(int min, int max, int inc)
    {
	m_testMin = min;
	m_testMax = max;
	m_testIncrease = inc;
	m_testValue = min;
    }

    /**
     * Enable / disable internal loopback
     * @param enable True to enable loopback, false to disable
     */
    inline void setLoopback(bool enabled = false, int sleep = 0)
	{ m_loopback = enabled; m_loopbackSleep = sleep; }

    /**
     * Check if the radio interface is on loopback mode
     * @return True if loopback mode is active.
     */
    inline bool loopback() const
	{ return m_loopback; }

protected:
    /**
     * Read data from radio
     * @param data I/O data structure
     * @param samples Optional number of samples to read,
     *  it will be set to actual read samples
     * @return True on success, on unrecoverable error
     */
    virtual bool readRadio(RadioIOData& data, unsigned int* samples = 0) = 0;

    /**
     * Write data to radio
     * @param data I/O data structure
     * @return True on success, on unrecoverable error
     */
    virtual bool writeRadio(RadioIOData& data) = 0;

    /**
     * Start the radio device
     * @return True on success
     */
    virtual bool radioPowerOn() = 0;

    /**
     * Stop the radio device
     */
    virtual void radioPowerOff() = 0;

    /**
     * Update Rx/Tx error counter
     * @param rx True for Rx, false for Tx
     * @param ok True on success, false on error
     * @param setTrxError True to call Transceiver::fatalError()
     * @return True on success, false to stop
     */
    inline bool updateError(bool rx, bool ok, bool setTrxError = true) {
	    ok = ok ? m_rxErrors.update(ok) : m_txErrors.update(ok);
	    if (ok)
		return ok;
	    Debug(this,DebugWarn,"%sExcesive %cx errors [%p]",
		prefix(),(rx ? 'R' : 'T'),this);
	    if (setTrxError && transceiver())
		transceiver()->fatalError();
	    return false;
	}

    Mutex m_mutex;
    unsigned int m_samplesPerSymbol;     // Samples per symbol
    Thread* m_readThread;                // Read thread
    // Radio device data
    float m_txPowerScale;                // Tx power scaling factor
    RadioIOData m_rxData;                // Rx buffer and data
    RadioIOData m_txData;                // Tx buffer and data
    RadioErrors m_rxErrors;              // Receive radio errors
    RadioErrors m_txErrors;              // Send radio errors

private:
    bool m_powerOn;                      // Powered on
    Mutex m_clockMutex;                  // Radio clock mutex
    GSMTime m_clock;                     // Radio clock
    GSMTime m_txBufferSpace;             // The space available for data sending in radio side
    Semaphore m_txSynchronizer;          // Semaphore used to synchronize the receive / send data
    // Test data
    int m_testMin;                       // Minimum value for imput data
    int m_testMax;                       // Maximum value for input data
    int m_testIncrease;                  // Increment value
    int m_testValue;                     // Current input data value
    bool m_loopback;                     // Loopback mode flag
    int m_loopbackSleep;                 // Time to sleep between two consecutive bursts sent in loopback mode
    u_int64_t m_loopbackNextSend;        // Next time to send a loopback burst
};

}; // namespace TelEngine

#endif // TRANSCEIVER_H

/* vi: set ts=8 sw=4 sts=4 noet: */

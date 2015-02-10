/**
 * bladerf.h
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

#ifndef BLADERF_H
#define BLADERF_H

#include "transceiver.h"
#include <libbladeRF.h>

namespace TelEngine {

struct BrfIO;                           // A bladeRF I/O buffer and related data
class BrfIface;                         // A bladeRF interface
class BladeRFDump;                      // Helper class used to dump data

// Timestamped data includes a header at buffer start
// The header has 16 octets = 4 samples (4 complex numbers)
// Timestamp is in half sample components
#define BRF_SUPERSPEED_SAMPLES 512
#define BRF_SUPERSPEED_SAMPLES_USE (BRF_SUPERSPEED_SAMPLES - 4)
#define BRF_HIGHSPEED_SAMPLES 256
#define BRF_HIGHSPEED_SAMPLES_USE (BRF_HIGHSPEED_SAMPLES - 4)

#define BRF_ENERGYPEAK 2047 // 2 ^ 11 - 1

struct BrfTsHeader
{
    uint32_t reserved;
    uint32_t timeLo;
    uint32_t timeHi;
    uint32_t flags;
};

struct BrfSuperSpeedTsBuffer
{
    BrfTsHeader header;
    int16_t samples[BRF_SUPERSPEED_SAMPLES_USE * 2];
};

struct BrfHighSpeedTsBuffer
{
    BrfTsHeader header;
    int16_t samples[BRF_HIGHSPEED_SAMPLES_USE * 2];
};

/**
 * Helper class to dump data into a file.
 */
class DataDumper
{
public:
    DataDumper(BrfIface* iface, u_int64_t samples, unsigned int spt);
    bool open(const String& fileName);
    void write(void* buf, unsigned int len, u_int64_t timestamp);
private:
    BrfIface* m_iface;
    u_int64_t m_samples;
    u_int64_t m_written;
    File m_file;
    unsigned int m_samplesPerTimeslot;
};

class BrfIO
{
public:
    inline BrfIO()
	: m_buffers(0), m_pos(0), m_timestamp(0),
	m_mutex(false,"BrfIO"), m_busy(false),
	m_libBuffers(0), m_count(0), m_current(0),
	m_useSamples(0), m_super(0), m_high(0)
	{ setBuffers(1,1,false); }
    inline ~BrfIO()
	{ clear(); }
    // Retrieve the buffer pointer
    inline void* buffer() const
	{ return m_super ? (void*)m_super : (void*)m_high; }
    // Retrieve buffer length in samples
    inline unsigned int bufSamples() const
	{ return m_bufSamples; }
    // Retrieve buffer usable samples
    inline unsigned int useSamples() const
	{ return m_useSamples; }
    // Retrieve buffer samples
    inline int16_t* samples(unsigned int idx) const
	{ return m_super ? m_super[idx].samples : m_high[idx].samples; }
    // Retrieve the current buffer samples
    inline int16_t* currentSamples() const
	{ return samples(current()); }
    // Retrieve buffer header
    inline BrfTsHeader& header(unsigned int idx) const
	{ return m_super ? m_super[idx].header : m_high[idx].header; }
    // Retrieve the current buffer header
    inline BrfTsHeader& currentHeader() const
	{ return header(current()); }
    // Retrieve the number of buffers
    inline unsigned int count() const
	{ return m_count; }
    // Retrieve the number of buffers in bladeRF library
    inline unsigned int libBuffers() const
	{ return m_libBuffers; }
    // Set buffers, clear data
    inline void setBuffers(unsigned int libBuffs, unsigned int ioBuffs,
	bool superSpeed) {
	    m_count = ioBuffs ? ioBuffs : 1;
	    m_libBuffers = (libBuffs > m_count) ? libBuffs : m_count;
	    reset(superSpeed);
	}
    // Retrieve the current buffer index
    inline unsigned int current() const
	{ return m_current; }
    // Retrieve available space in current buffer, may be negative
    inline int available() const
	{ return (int)useSamples() - (int)m_pos; }
    // Advance the current buffer, return true on success, false if reset to 0
    inline bool advanceCurrent() {
	    m_current++;
	    if (m_current < m_buffers)
		return true;
	    m_current = 0;
	    m_buffers = 0;
	    return false;
	}
    // Advance timestamp and consumed samples
    inline void advance(unsigned int samples) {
	    m_pos += samples;
	    m_timestamp += samples;
	}
    // Reset data
    void reset(bool superSpeed);

    unsigned int m_buffers;              // Used buffers
    unsigned int m_pos;                  // Buffer position in samples
    uint64_t m_timestamp;                // I/O timestamp
    // Serialize I/O
    Mutex m_mutex;
    bool m_busy;

private:
    void clear();
    unsigned int m_libBuffers;           // The number of buffers to set in bladeRF library
    unsigned int m_count;                // The number of buffers
    unsigned int m_current;              // Current buffer
    unsigned int m_bufSamples;           // Buffer length in samples
    unsigned int m_useSamples;           // Usable samples in a buffer
    BrfSuperSpeedTsBuffer* m_super;
    BrfHighSpeedTsBuffer* m_high;
};


/**
 * This class implements the interface to a bladeRF device
 * @short A bladeRF interface
 */
class BrfIface : public RadioIface
{
    YCLASS(BrfIface,RadioIface)
    YNOCOPY(BrfIface);
public:
    /**
     * Constructor
     */
    BrfIface();

    /**
     * Destructor
     */
    virtual ~BrfIface();

    /**
     * Initialize radio
     * @param params Radio parameters
     * @return True on success, false on failure
     */
    virtual bool init(const NamedList& params);

    /**
     * Set Rx/Tx frequency
     * @param rx True for Rx, false for Tx
     * @param freq Frequency value in Hertz
     * @param adj Adjust value (ussually the VCTCXO value)
     * @return 0 on success, negative on failure, positive on fatal
     *  error (transceiver should stop)
     */
    virtual int tune(bool rx, double freq, double adj);

    /**
     * Set Rx/Tx power gain
     * @param rx True to set Rx gain, false to set Tx gain
     * @param dB Power value
     * @param setVal Optional pointer to set value
     * @return 0 on success, negative on failure, positive on fatal
     *  error (transceiver should stop)
     */
    virtual int radioSetGain(bool rx, double dB, double* setVal = 0);

    /**
     * Execute a command
     * @param cmd Command to execute
     * @param rspParam Optional response
     * @param reason Optional failure reason
     * @return 0 on success, non 0 on failure
     */
    virtual int command(const String& cmd, String* rspParam = 0, String* reason = 0);

    /**
     * Set frequency correction offset
     * @param val The new value
     * @return 0 on success 1 on failure
     */
    virtual bool setFreqCorr(int val)
	{ return setVCTCXO(val,false); }

    /**
     * Destroy the object
     */
    virtual void destruct();

    /**
     * Stop dumping data.
     * @param rx True if we should stop dumping RX data
     */
    inline void stopDump(bool rx = true)
    {
	Lock myLock(m_dumpMutex);
	if (!rx) {
	    TelEngine::destruct(m_txDump);
	    m_txDump = 0;
	}
    }

    /**
     * Get The last timestamp received from the bord
     * @return The last timestamp received from the board
     */
    virtual u_int64_t getBoardTimestamp()
	{ return m_lastBoardTs; }

    /**
     * Reset data dumper.
     * @param d The data dumper to reset.
     * @param isSafe True if the operation is safe
     * @return True if the dumper was reseted
     */
    bool resetDumper(DataDumper* d, bool isSafe = false);

protected:
    /**
     * Read data from radio
     * @param data I/O data structure
     * @param samples Optional number of samples to read,
     *  it will be set to actual read samples
     * @return True on success, on unrecoverable error
     */
    virtual bool readRadio(RadioIOData& data, unsigned int* samples = 0);

    /**
     * Write data to radio
     * @param data I/O data structure
     * @return True on success, on unrecoverable error
     */
    virtual bool writeRadio(RadioIOData& data);

    /**
     * Start the radio device
     * @return True on success
     */
    virtual bool radioPowerOn();

    /**
     * Stop the radio device
     */
    virtual void radioPowerOff();

private:
    bool open(const NamedList& params);
    void doPowerOff(bool safeRx, bool safeTx);
    void doClose(bool safeRx, bool safeTx);
    bool enableModule(bool rx, bool on);
    inline bool enableModules()
	{ return enableModule(true,true) && enableModule(false,true); }
    bool loadFPGA(const NamedList& params, int& code, String& what, String& fpga);
    bool setSampleRate(bool rx, int& code, String& what, unsigned int* actual = 0);
    bool setBandwith(bool rx, int& code, String& what, unsigned int value,
	unsigned int* actual = 0);
    bool initSyncIO(bool rx, unsigned int numBuffers, unsigned int bufSizeSamples,
	unsigned int numTransfers, int* code = 0, String* what = 0);
    inline bool initSyncIO() {
	    unsigned int bufSizeSamples = 8192;  // Buffer size in samples
	    unsigned int numTransfers = 4;
	    return initSyncIO(true,m_rxIO.libBuffers(),bufSizeSamples,numTransfers) &&
		initSyncIO(false,m_txIO.libBuffers(),bufSizeSamples,numTransfers);
	}
    bool initTimestampsGPIO();
    bool setCorrection(bool rx, int offsetI, int offsetQ, bool safe);
    bool setCorrection(bool rx, bool offsI, int val);
    bool setVCTCXO(unsigned int val, bool safe);
    bool setFreq(bool rx, unsigned int hz, unsigned int vctcxo, bool safe);
    // Set Rx/TX power gain.
    // Return 0 on success, negative on library error, positive on fatal error
    int setGain(bool rx, int vga2, bool updError = true, bool safe = true,
	int* setVal = 0);
    // Set Rx/TX VGA1 (pre-LPF) or VGA2 (post-LPF) power gain.
    // Return 0 on success, negative on library error, positive on fatal error
    int setGainVga(bool pre, bool rx, int value, bool updError = false,
	bool safe = true);
    // Set device speed and related data
    void setSpeed(bool super, const NamedList& params = NamedList::empty());
    // Compute Rx avg values, autocorrect offsets if configured
    void computeRx();
    int handleRxCmds(String& s, String* rspParam, String* reason);
    int handleTxCmds(String& s, String* rspParam, String* reason);
    bool setLoopback(int loopback);
    bool getLoopback(int &loopbak);
    bool setSampling(int sampling);
    bool getSampling(int &sampling);
    bool setLnaGain(int lnaGain);
    bool getLnaGain(int &gain);
    bool getSampleRate(bool rx, unsigned int& sampleRate);
    bool getBandwidth(bool rx, unsigned int& bandwidth);
    bool setLpfMode(bool rx, int lpfMode);
    bool getLpfMode(bool rx, int &lpfMode);
    bool getVga(bool rx,bool one, int& gain);
    bool getFrequency(bool rx, unsigned int &freq);
    bool setRxDump(DataDumper* data = 0);
    bool setTxDump(DataDumper* data = 0);

    inline uint32_t flag(uint32_t mask) const
	{ return (m_flags & mask); }
    inline bool isFlag(uint32_t mask) const
	{ return mask == flag(mask); }
    inline void setFlag(uint32_t mask, bool on) {
	    if (on)
		m_flags |= mask;
	    else
		m_flags &= ~mask;
	}

    uint32_t m_flags;                    // Flags
    struct bladerf* m_dev;               // The device
    bool m_superSpeed;                   // Super/high USB speed
    BrfIO m_rxIO;                        // Rx buffer and related data
    uint64_t m_rxResyncCandidate;        // Rx timestamp resync
    int m_rxShowDcInfo;                  // Output Rx DC info
    bool m_rxDcAuto;                     // Automatically adjust Rx DC offset
    int m_rxDcOffsetMax;                 // Rx DC offset correction
    int m_rxDcOffsetI;                   // Current I (in-phase) DC RX offset
    int m_rxDcOffsetQ;                   // Current Q (quadrature) DC RX offset
    int m_rxDcAvgI;                      // Current average for I (in-phase) DC RX offset
    int m_rxDcAvgQ;                      // Current average for Q (quadrature) DC RX offset
    int m_rxVga1;                        // Rx VGA1 gain
    BrfIO m_txIO;                        // Tx buffer and related data
    int m_txShowInfo;                    // Output Tx info (min/max values of input buffer)
    bool m_modulated;                    // Flag used in test mode. It says if we should send modulated data or just predefined data.
    int16_t* m_predefinedData;           // Predefined data to be sent in test mode
    // Test data
    Mutex m_dumpMutex;                   // Mutex used to block dump operations
    BladeRFDump* m_txDump;               // Dump data object for TX
    unsigned int m_loopbackFreq;         // Test loopback frequency
    u_int64_t m_lastBoardTs;             // Last timestamp received from the board
    bool m_dumpTxTime;                   // Flag used to dump RX time
    DataDumper* m_txDumper;              // TX file data dumper
    DataDumper* m_rxDumper;              // RX file data dumper
};

/**
 * Helper class which is used to dump data on console or in a file
 */
class BladeRFDump : public GenObject
{
public:
    enum DumpType {
	Raw,
	Samples,
	Complex
    };

    BladeRFDump(BrfIface* iface)
	: m_type(Raw), m_fileAppend(false), m_samples(1250), m_interface(iface)
    {}

    /**
     * Dump the received buffer on stdout or in file.
     * @param buf The buffer to dump.
     * @param len The buffer length.
     */
    void dump(const void* buf, unsigned int samples);

    int m_type;
    String m_fileName;
    bool m_fileAppend;
    int m_samples;
private:
    BrfIface* m_interface;
    DataBlock m_data;
    String m_dump;
};

}; // namespace TelEngine

#endif // BLADERF_H

/* vi: set ts=8 sw=4 sts=4 noet: */

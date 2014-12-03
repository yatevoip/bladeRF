/**
 * gsmutil.h
 * This file is part of the Yate-BTS Project http://www.yatebts.com
 *
 * GSM related utilities
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

#ifndef GSMUTIL_H
#define GSMUTIL_H

#include <yateclass.h>
#include "sigproc.h"

namespace TelEngine {

class GSMTime;                           // GSM time
class GSMBurst;                          // A GSM burst
class GSMTxBurst;                        // A GSM burst to send
class GSMRxBurst;                        // A received GSM burst

// TDMA frame number max value (see TS 100 908 (GSM 05.02) Section 4.3.3)
// The actual maximum value is GSM_HYPERFRAME_MAX - 1 (2715647)
#define GSM_HYPERFRAME_MAX (26U * 51U * 2048U)
#define GSM_HYPERFRAME_MAX_HALF (GSM_HYPERFRAME_MAX / 2)

// Burst length in symbols
#define GSM_BURST_LENGTH 148

// The number of GSM TSC (Training Sequence Code) Normal Burst
#define GSM_NB_TSC_LEN 26
// The number of GSM Sync bits for Access Burst
#define GSM_AB_SYNC_LEN 41


/**
 * This class holds GSM related utility data and methods
 * @short GSM utilities
 */
class GSMUtils
{
public:
    /**
     * Retrieve the TSC (Training Sequence Code) table for Normal Burst
     * @return Pointer to 8 by GSM_NB_TSC_LEN Normal Burst TSC table
     */
    static const int8_t* nbTscTable();

    /**
     * Retrieve the Sync table for Access Burst
     * @return Pointer to GSM_AB_SYNC_LEN Access Burst Sync table
     */
    static const int8_t* abSyncTable();
};


/**
 * This class implements GSM time as defined in GSM 05.02 4.3
 * @short GSM time
 */
class GSMTime
{
public:
    /**
     * Constructor
     */
   inline GSMTime()
	: m_fn(0), m_tn(0)
	{}

    /**
     * Constructor
     * @param fn Frame number
     * @param tn Timeslot number
     */
    inline GSMTime(uint32_t fn, uint8_t tn = 0)
	{ assign(fn,tn); }

    /**
     * Copy constructor
     * @param src Source object
     */
    inline GSMTime(const GSMTime& src)
	: m_fn(src.fn()), m_tn(src.tn())
	{}

    /**
     * Assign absolute time.
     * @param fn Frame number
     * @param tn Timeslot number
     */
    inline void assign(uint32_t fn, uint8_t tn = 0)
	{
	    m_fn = fn % GSM_HYPERFRAME_MAX;
	    m_tn = tn % 8;
	}

    /**
     * Retrieve the frame number
     * @return Frame number
     */
    inline uint32_t fn() const
	{ return m_fn; }

    /**
     * Retrieve the timeslot number
     * @return timeslot number
     */
    inline uint8_t tn() const
	{ return m_tn; }

    /**
     * Increment frame number
     */
    inline void incFn()
	{ m_fn = (m_fn + 1) % GSM_HYPERFRAME_MAX; }

    /**
     * Decrement frame number
     */
    inline void decFn() {
	    if (m_fn)
		m_fn--;
	    else
		m_fn = GSM_HYPERFRAME_MAX - 1;
	}

    /**
     * Increment timeslot number.
     * Increment frame number also if timeslot wrap around
     */
    inline void incTn() {
	    if (m_tn < 7)
		m_tn++;
	    else {
		m_tn = 0;
		incFn();
	    }
	}

    /**
     * Increase timeslot number.
     * Increment frame number also if timeslot wrap around
     * @param val Value to increase, interval: [1..7]
     */
    inline void incTn(uint8_t val) {
	    if (!val || val > 8)
		return;
	    m_tn += val;
	    if (m_tn > 7) {
		m_tn -= 8;
		incFn();
	    }
	}

    /**
     * Decrement timeslot number.
     * Decrement frame number also if timeslot wrap around
     */
    inline void decTn() {
	    if (m_tn)
		m_tn--;
	    else {
		m_tn = 7;
		decFn();
	    }
	}

    /**
     * Decrease timeslot number.
     * Decrement frame number also if timeslot wrap around
     * @param val Value to decrease, interval: [1..7]
     */
    inline void decTn(uint8_t val) {
	    if (!val || val > 8)
		return;
	    if (m_tn >= val)
		m_tn -= val;
	    else {
		m_tn = (uint8_t)(8 + ((int)m_tn - (int)val));
		decFn();
	    }
	}

    /**
     * Advance time
     * @param fn Frame number to advance
     * @param tn Timeslot number to advance
     */
    inline void advance(uint32_t fn, uint8_t tn = 0)
	{ sum(m_fn,m_tn,m_fn,m_tn,fn,tn); }

    /**
     * Greater then comparison operator
     * @param other Object to compare with
     * @return True if this time is greater then given time
     */
    inline bool operator>(const GSMTime& other) const
	{ return compare(fn(),tn(),other.fn(),other.tn()) > 0; }

    /**
     * Equality operator
     * @param other Object to compare with
     * @return True if this time is equal with given time
     */
    inline bool operator==(const GSMTime& other) const
	{ return compare(fn(),tn(),other.fn(),other.tn()) == 0; }

    /**
     * Inequality operator
     * @param other Object to compare with
     * @return True if this time is not equal with given time
     */
    inline bool operator!=(const GSMTime& other) const
	{ return compare(fn(),tn(),other.fn(),other.tn()) != 0; }

    /**
     * Asignment operator
     * @param src Source object
     */
    inline GSMTime& operator=(const GSMTime& src) {
	    m_fn = src.fn();
	    m_tn = src.tn();
	    return *this;
	}

    /**
     * Add time values
     * @param fn Destination frame number
     * @param tn Destination timeslot number
     * @param fn1 First frame number
     * @param tn1 First timeslot number
     * @param fn2 Second frame number
     * @param tn2 Second timeslot number
     */
    static inline void sum(uint32_t& fn, uint8_t& tn, uint32_t fn1, uint8_t tn1,
	uint32_t fn2, uint8_t tn2) {
	    tn = (tn1 + tn2) % 8;
	    fn = (fn1 + fn2 + (tn1 + tn2) / 8) % GSM_HYPERFRAME_MAX;
	}

    /**
     * Calculate the difference between time values
     * @param fn Destination frame number
     * @param tn Destination timeslot number
     * @param fn1 First frame number
     * @param tn1 First timeslot number
     * @param fn2 Second frame number
     * @param tn2 Second timeslot number
     */
    static inline void diff(uint32_t& fn, uint8_t& tn, uint32_t fn1, uint8_t tn1,
	    uint32_t fn2, uint8_t tn2) {
	    if (tn1 < tn2) {
		tn1 += 8;
		fn1 == 0 ? fn1 = GSM_HYPERFRAME_MAX : fn1--;
	    }
	    tn = tn1 - tn2;
	    fn = fn1 >= fn2 ? fn1 - fn2 : GSM_HYPERFRAME_MAX - fn2 + fn1;
	}

    /**
     * Calculate the difference between given time values
     * @param dest The destination time value
     * @param t1 First timer
     * @param t2 Second timer
     */
    static inline void diff(GSMTime& dest, const GSMTime& t1, const GSMTime& t2)
	{ diff(dest.m_fn,dest.m_tn,t1.fn(),t1.tn(),t2.fn(),t2.tn()); }

    /**
     * Compare time values
     * @param fn1 First frame number
     * @param tn1 First timeslot number
     * @param fn2 Second frame number
     * @param tn2 Second timeslot number
     * @return Negative if first time is less then the second one, positive if first
     *  time is greater then the second one, 0 if equal
     */
    static inline int32_t compare(uint32_t fn1, uint8_t tn1, uint32_t fn2, uint8_t tn2) {
	    if (fn1 != fn2)
		return fnOffset(fn1,fn2);
	    return (int)tn1 - (int)tn2;
	}

    /**
     * Compare time values
     * @param t1 First time
     * @param t2 Second time
     * @return Negative if first time is less then the second one, positive if first
     *  time is greater then the second one, 0 if equal
     */
    static inline int32_t compare(const GSMTime& t1, const GSMTime& t2)
	{ return compare(t1.fn(),t1.tn(),t2.fn(),t2.tn()); }

    /**
     * Compare time values. Increment first time frame number before comparison
     * @param t1 First time
     * @param t2 Second time
     * @param t1FN Value to add to first time frame number
     * @return Negative if first time is less then the second one, positive if first
     *  time is greater then the second one, 0 if equal
     */
    static inline int32_t compare(const GSMTime& t1, const GSMTime& t2, uint32_t t1FN)
	{ return compare((t1.fn() + t1FN) % GSM_HYPERFRAME_MAX,t1.tn(),t2.fn(),t2.tn()); }

    /**
     * Calculate the offset between 2 frames inside frame number max value interval
     * @param fn1 First frame number
     * @param fn2 Second frame number
     * @return The offset between fn1 and fn2
     */
    static inline int32_t fnOffset(uint32_t fn1, uint32_t fn2) {
	    int32_t delta = (int32_t)fn1 - (int32_t)fn2;
	    if (delta >= (int32_t)GSM_HYPERFRAME_MAX_HALF)
		return delta - (int32_t)GSM_HYPERFRAME_MAX;
	    if (delta < -(int32_t)GSM_HYPERFRAME_MAX_HALF)
		return delta + (int32_t)GSM_HYPERFRAME_MAX;
	    return delta;
	}

protected:
    uint32_t m_fn;                       // Frame number
    uint8_t m_tn;                        // Timeslot number
};


/**
 * This class implements a GSM burst
 * @short A GSM Burst
 */
class GSMBurst : public DataBlock
{
    YCLASS(GSMBurst,DataBlock)
    YNOCOPY(GSMBurst);
public:
    /**
     * Constructor
     */
    inline GSMBurst()
	{}

    /**
     * Retrieve burst time
     * @return Burst time
     */
    inline const GSMTime& time() const
	{ return m_time; }

    /**
     * Set burst time
     * @param t Burst time
     */
    inline void time(const GSMTime& t)
	{ m_time = t; }

protected:
    GSMTime m_time;                      // Burst time
};


/**
 * This class implements a GSM burst to be sent
 * @short A GSM burst to send
 */
class GSMTxBurst : public GSMBurst
{
    YCLASS(GSMTxBurst,GSMBurst)
    YNOCOPY(GSMTxBurst);
public:
    /**
     * Destructor
     */
    virtual ~GSMTxBurst();

    /**
     * Helper method to set ARFCN number
     */
    inline void arfcn(unsigned int arfcnNr)
	{ m_arfcnNumber = arfcnNr;}

    /**
     * Helper method to obtain ARFCN number
     */
    inline unsigned int arfcn() const
	{ return m_arfcnNumber; }

    /**
     * Obtain a burst from the given data
     * @param data The data read from the socket.
     * @param length The data length read from the socket
     * @return A new burst if the data is valid.
     */
    static GSMTxBurst* get(const DataBlock& data, unsigned int length);

    /**
     * Transform the received bit data into a complex array
     * @param sigproc Reference to the Signal Processing instance that holds the
     *  precalculated data.
     * @return The transformed complex array or 0 on error.
     * Note: The returned complex array is owned by the Burst!
     */
    const ComplexArray* transform(const SignalProcessing& sigproc);

    /**
     * Helper method to verify if this burst is a filler one
     * @return True if this burst is a filler.
     */
    inline bool isFiller() const
	{ return m_isFillerBurst; }

    /**
     * Mark this burst as beeing a dummy one
     */
    inline void setDummy()
	{ m_isDummy = true; }

    /**
     * Check if this burst is a dummy one
     * @return True if this burst is a dummy one
     */
    inline bool isDummy() const
	{ return m_isDummy; }

    /**
     * Get 'zero' burst. A burst that after all transformatons the output values are 0
     * @return A pointer to a 'zero burst'. NOTE The caller does not own the pointer
     */
    static GSMTxBurst* getZeroBurst();

    static GSMTxBurst s_zeroBurst;

private:
    /**
     * Constructor
     */
    GSMTxBurst()
	: m_transformed(0), m_arfcnNumber(0), m_powerLevel(0), m_isFillerBurst(false),
	m_isDummy(false)
	{}

    ComplexArray* m_transformed;
    unsigned int m_arfcnNumber;
    float m_powerLevel;
    bool m_isFillerBurst;
    bool m_isDummy;
};


/**
 * This class implements a received GSM burst
 * @short A received GSM burst
 */
class GSMRxBurst : public GSMBurst
{
    YCLASS(GSMRxBurst,GSMBurst)
    YNOCOPY(GSMRxBurst);
public:
    /**
     * Constructor
     */
    inline GSMRxBurst()
	: m_powerLevel(0), m_timingError(0)
	{}

    /**
     * Setup a buffer of symbol estimates from this burst
     * The buffer start is filled with time TN/FN, RSSI and timing offset
     * @param buf Destination buffer
     * @param len Buffer length
     * @return The number of bytes set in buffer
     */
    unsigned int buildEstimatesBuffer(int8_t* buf, unsigned int len);

    float m_powerLevel;                  // Received power level
    float m_timingError;                 // Timing advance error
    ComplexVector m_data;                // Data
    FloatVector m_bitEstimate;           // Output
};

}; // namespace TelEngine

#endif // GSMUTIL_H

/* vi: set ts=8 sw=4 sts=4 noet: */

/**
 * test.h
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

#ifndef TRANSCEIVER_TEST_H
#define TRANSCEIVER_TEST_H

#include "transceiver.h"

namespace TelEngine {

class TestIface;                         // A test radio interface


/**
 * This class implements a test radio interface
 * @short A test radio interface
 */
class TestIface : public RadioIface
{
    YCLASS(TestIface,RadioIface)
    YNOCOPY(TestIface);
public:
    /**
     * Constructor
     */
    TestIface();

    /**
     * Send the data to the radio.
     * @param data the data to be sent.
     * @return True if data was sent.
     */
    virtual bool sendData(const ComplexArray& data);

    /**
     * Set Rx/Tx frequency
     * @param rx True for Rx, false for Tx
     * @param freq Frequency value
     * @param adj Adjust value (ussually the VCTCXO value)
     * @return 0 on success, negative on failure, positive on fatal
     *  error (transceiver should stop)
     */
    virtual int tune(bool rx, double freq, double adj);

    /**
     * Utility method who dumps data into different stages of burst transformation
     * @param params The tests parameters
     */
    void testTx(NamedList& params);

protected:
    /**
     * Read data from radio
     * @param data I/O data structure
     * @param samples Optional samples to read
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

    /**
     * Set Rx/Tx power gain
     * @param rx True to set Rx gain, false to set Tx gain
     * @param dB Power value
     * @param setVal Optional pointer to set value
     * @return 0 on success, negative on failure, positive on fatal
     *  error (transceiver should stop)
     */
    virtual int radioSetGain(bool rx, double dB, double* setVal = 0);

private:
    bool m_powerOn;
};

}; // namespace TelEngine

#endif // TRANSCEIVER_TEST_H

/* vi: set ts=8 sw=4 sts=4 noet: */

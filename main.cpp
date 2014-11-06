/**
 * Tranceiver.h
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

//#define TRX_USE_YBTSLOG

#include "transceiver.h"
#include <yatengine.h>
#include <stdlib.h>
#include <signal.h>

#ifdef HAVE_BRF
#include "bladerf/bladerf.h"
#endif

#ifdef HAVE_TEST
#include "test/test.h"
#endif

#ifdef TRX_USE_YBTSLOG
#include <Configuration.h>
#include <LogConnection.h>
ConfigurationTable gConfig;
static Connection::LogConnection s_log(STDERR_FILENO + 1);
static void debugFunc(const char* str, int level) 
{
    s_log.write((char)level,str);
}
#endif

using namespace TelEngine;

int s_code = -1;
static Configuration s_cfg;
static NamedList* s_trxParams = 0;
static Transceiver* s_trx = 0;

class LocalTransceiver : public TransceiverQMF
{
public:
    LocalTransceiver();
    virtual void fatalError();
};

static inline void setMissingParam(NamedList& nl, const String& name, const char* value,
    bool emptyOk = false)
{
    NamedString* ns = nl.getParam(name);
    if (ns) {
	if (!*ns)
	    *ns = value;
    }
    else
	nl.addParam(name,value,emptyOk);
}

static inline void loadConfig()
{
    s_cfg.load();
    s_trxParams = s_cfg.createSection("transceiver");
    // Set engine debug level
    const String* str = s_trxParams->getParam(YSTRING("debug_engine"));
    if (str)
	TelEngine::debugLevel(str->toInteger(TelEngine::debugLevel()));
    // Fill missing params
    setMissingParam(*s_trxParams,"RadioFrequencyOffset","128");
    setMissingParam(*s_trxParams,"TxAttenOffset","0");
    setMissingParam(*s_trxParams,"MinOversampling","1");
    setMissingParam(*s_trxParams,"remoteaddr","127.0.0.1");
    setMissingParam(*s_trxParams,"port","5700");
#ifdef HAVE_BRF
    // bladeRF specific parameters
    //p.addParam("bladerf_load_fpga","check"); // true/false/check, default: true
    //p.addParam("bladerf_fpga_40k","bladerf/hostedx40.rbf"); // default: hostedx40.rbf
    //p.addParam("bladerf_fpga_115k","bladerf/hostedx115.rbf"); // default: hostedx115.rbf
    //p.addParam("io_timeout","100"); // default: 500, minimum: Thread::idleMsec()
    //p.addParam("rx_dc_autocorrect","false"); // default: true
    //setMissingParam(*s_trxParams,"bladerf_debug_level",String(BLADERF_LOG_LEVEL_WARNING),false);
#endif
}

// Initialize libyate
static void initLibyate()
{
    TelEngine::Thread::idleMsec(0);
    Debugger::enableOutput(true,true);
#ifdef TRX_USE_YBTSLOG
    Debugger::setOutput(debugFunc);
#else
    Debugger::setFormatting(Debugger::Relative);
#endif
}

static void startTransceiver(int argc, const char** argv)
{
    NamedList p(*s_trxParams);
    //p.addParam("burst_min_power","0.2");
    for (int i = 1; i < argc; i++) {
	switch (i) {
	    case 1:
		// ARFCN count
		p.addParam("arfcns",argv[i],false);
		break;
	    case 2:
		// Radio device arguments
		p.addParam("radio.device_args",argv[i],false);
		break;
	}
    }
    RadioIface* radio = 0;
#ifdef HAVE_TEST
    if (!radio)
	radio = new TestIface;
#endif
#ifdef HAVE_BRF
    if (!radio)
	radio = new BrfIface;
#endif
    if (!s_trx)
	s_trx = new LocalTransceiver();
    if (s_trx->init(radio,p)) {
	if (s_trx->start()) {
#ifdef HAVE_TEST
	    s_trx->command("CMD RXTUNE 825000");
	    s_trx->command("CMD TXTUNE 870000");
	    s_trx->command("CMD POWERON");
	    // TEST: Configure slots
	    for (unsigned int i = 0; i < s_trx->arfcnCount(); i++)
		for (int j = 0; j < 8; j++)
		    s_trx->command("CMD SETSLOT " + String(j) + " 1",0,i);
#endif
	    return;
	}
	s_code = 2;
    }
    else
	s_code = 3;
    TelEngine::destruct(s_trx);
}

static void sigHandler(int sig)
{
    switch (sig) {
	case SIGINT:
	case SIGTERM:
	    s_code = 0;
	    break;
    }
}

LocalTransceiver::LocalTransceiver()
    : TransceiverQMF()
{
}

void LocalTransceiver::fatalError()
{
    Transceiver::fatalError();
    s_code = 1;
}

extern "C" int main(int argc, const char** argv, const char** envp)
{
    initLibyate();
    ::signal(SIGINT,sigHandler);
    ::signal(SIGTERM,sigHandler);
    int pid = (int)::getpid();
    Output("Transceiver application (%d) is starting",pid);
    s_cfg = ::getenv("MBTSConfigFile");
    loadConfig();
    startTransceiver(argc,argv);
    if (!s_trx && s_code == -1)
	s_code = 1;
    while (s_code == -1)
	TelEngine::Thread::idle();
    Output("Transceiver application (%d) is exiting with code %d",pid,s_code);
    if (s_trx)
	s_trx->stop();
    TelEngine::Thread::killall();
    TelEngine::destruct(s_trx);
    Output("Transceiver application (%d) terminated",pid);
    return s_code;
}

/* vi: set ts=8 sw=4 sts=4 noet: */

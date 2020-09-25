#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#include <string>
#include <sstream>
#include <fstream>

#include <sys/types.h>
#include <sys/stat.h>

#include <cantProceed.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsExit.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsPrint.h>
#include <ellLib.h>
#include <iocsh.h>

#include <yaml-cpp/yaml.h>
#include <yamlLoader.h>

#include <drvSup.h>
#include <epicsExport.h>
#include <registryFunction.h>

#include <asynPortDriver.h>
#include <asynOctetSyncIO.h>

#include <bldPvClient.h>

#include "pcavAsyn.h"


static bool            keep_stay_in_loop = true;
static epicsEventId    shutdownEvent;

static ELLLIST *pDrvEllList = NULL;

typedef struct {
    ELLNODE         node;
    char            *named_root;
    char            *port;
    char            *regPath;
    char            *bsaStream;
    char            *bsaPrefix;
    pcavAsynDriver  *pcavAsyn;
    void            *prv;
} pDrvList_t;


static void init_drvList(void)
{
    if(!pDrvEllList) {
        pDrvEllList = (ELLLIST *) mallocMustSucceed(sizeof(ELLLIST), "pcavAsyn driver: init_drvList()");
        ellInit(pDrvEllList);
    }

    return;
} 

static pDrvList_t *find_drvByPort(const char *port)
{
     init_drvList();
     pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);

     while(p) {
         if(p->port && strlen(p->port) && !strcmp(p->port, port)) break;
         p = (pDrvList_t *) ellNext(&p->node);
     }

     return p;
}

static pDrvList_t *find_drvByNamedRoot(const char *named_root)
{
    init_drvList();
    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);

    while(p) {
        if(p->named_root && strlen(p->named_root) && !strcmp(p->named_root, named_root)) break;
        p = (pDrvList_t *) ellNext(&p->node);
    }

    return p;
}


pcavAsynDriver::pcavAsynDriver(void *pDrv, const char *portName, const char *pathString, const char *bsaStream, const char *bsaPrefix, const char *named_root)
    : asynPortDriver(portName,
                     1, /* number of elements of this device */
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
                     NUM_LLRFHLS_DET_PARAMS, /* number of asyn params of be cleared for each device */
#endif  /* asyn version check, under 4.32 */
                     asynInt32Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask | asynInt16ArrayMask | asynInt32ArrayMask | asynFloat64ArrayMask, /* Interface mask */
                     asynInt32Mask | asynFloat64Mask | asynOctetMask | asynEnumMask    | asynInt16ArrayMask | asynInt32ArrayMask | asynFloat64ArrayMask,  /* Interrupt mask */
                     1, /* asynFlags.  This driver does block and it is not multi-device, so flag is 1 */
                     1, /* Autoconnect */
                     0, /* Default priority */
                     0) /* Default stack size*/
{
    Path p_root;
    Path p_pcav;
    port = epicsStrDup(portName);
    path = epicsStrDup(pathString);
    stream = (bsaStream && strlen(bsaStream))?epicsStrDup(bsaStream): NULL;
    bsa_name = (bsaPrefix && strlen(bsaPrefix))? epicsStrDup(bsaPrefix): "default_bsa";
    this->pDrv = pDrv;
    pollCnt = 0;
    streamPollCnt = 0;
    current_bsss  = -1;

    try {
        p_root = (named_root && strlen(named_root))? cpswGetNamedRoot(named_root): cpswGetRoot();
        p_pcav = p_root->findByName(pathString);
        if(stream) _bstream = IStream::create(p_root->findByName(stream));
    } catch (CPSWError &e) {
        fprintf(stderr, "CPSW Error: %s, file %s, line %d\n", e.getInfo().c_str(), __FILE__, __LINE__);
        throw e;
    }

    _pcav      = IpcavFw::create(p_pcav);
    _dacSigGen = IdacSigGenFw::create(p_root->findByName("mmio"));

    for(int i = 0; i < NUM_CAV; i++) {
        _coeff_time[i].a = _coeff_charge[i].a = 1.;
        _coeff_time[i].b = _coeff_charge[i].b = 0.;
    }

    _bld_data = { 0., 0., 0., 0. };
    _st_data  = { true,
                  0, 0, 0, 0,
                  0., 0., 0.004, 0.004,
                  0., 0., 0., 0.,
                  0., 0., 0., 0.,
                  0., 0., 0., 0 };


    ParameterSetup();
    bsaSetup();

}


pcavAsynDriver::~pcavAsynDriver() {}


asynStatus pcavAsynDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int        function      = pasynUser->reason;
    asynStatus status        = asynSuccess;
    const char *functionName = "writeInt32";

    status = (asynStatus) setIntegerParam(function, value);

    if(function == p_reset)  {
        if(!_st_data.reset) _st_data.reset = true;   // claculation has worked, need to reset
        else {  // caculation never worked after last reset, need to clear counter and need to post it.
            if(_st_data.validCnt0 || _st_data.validCnt1 
                 || _st_data.invalidCnt0 || _st_data.invalidCnt1) {
                _st_data.validCnt0   = _st_data.validCnt1   = 0;  // reset valid counter
                _st_data.invalidCnt0 = _st_data.invalidCnt1 = 0;  // reset invalid counter
                setIntegerParam(p_result[0].validCnt, _st_data.validCnt0);
                setIntegerParam(p_result[1].validCnt, _st_data.validCnt1);
                setIntegerParam(p_result[0].invalidCnt, _st_data.invalidCnt0);
                setIntegerParam(p_result[1].invalidCnt, _st_data.invalidCnt1);
            }
        }
    }
    else
    if(function == p_rfRefSel)                 _pcav->setRefSel((uint32_t) value);
    else

    for(int i = 0; i < NUM_CAV; i++) {
        if(function == p_cavFreqEvalStart[i]) {
            _pcav->setFreqEvalStart(i, (uint32_t) value); goto _escape;
        }
        if(function == p_cavFreqEvalEnd[i]) {
            _pcav->setFreqEvalEnd(i, (uint32_t) value); goto _escape;
        }
        if(function == p_cavRegLatchPoint[i]) {
            _pcav->setRegLatchPoint(i, (uint32_t) value); goto _escape;
        }
        for(int j = 0; j < NUM_PROBE; j++) {
            if(function == p_cavChanSel[i][j]) {
                _pcav->setChanSel(i, j, (uint32_t) value); goto _escape;
            }
            if(function == p_cavWindowStart[i][j]) {
                _pcav->setWindowStart(i, j, (uint32_t) value); goto _escape;
            }
            if(function == p_cavWindowEnd[i][j]) {
                _pcav->setWindowEnd(i, j, (uint32_t) value); goto _escape;
            }
        }
    }

    for(int i = 0; i < NUM_WFDATA; i++) {
        if(function == p_wfDataSel[i]) {
            _pcav->setWfDataSel(i, (uint32_t) value); goto _escape;
        }
    }
    
    _escape:

    return status;
}


asynStatus pcavAsynDriver::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int        function      = pasynUser->reason;
    asynStatus status        = asynSuccess;
    const char *functionName = "writeFloat64";

    status = (asynStatus) setDoubleParam(function, value);

    for(int i = 0; i < NUM_CAV; i++) {
        if(function == p_cavNCOPhaseAdj[i]) {
            uint32_t raw = _pcav->setNCO(i, value); 
            setIntegerParam(p_cavNCORaw[i], raw);
            goto _escape;
        }

        if(function == p_coeff_time[i].a) {
            _coeff_time[i].a = value;    goto _escape;
        }
        if(function == p_coeff_time[i].b) {
            _coeff_time[i].b = value;    goto _escape;
        }
        if(function == p_coeff_charge[i].a) {
            _coeff_charge[i].a = value;   goto _escape;
        }
        if(function == p_coeff_charge[i].b) {
            _coeff_charge[i].b = value;   goto _escape;
        }
        if(function == p_result[i].threshold) {
            switch(i) {
                case 0:
                    _st_data.thresholdChrg0 = value;
                    break;
                case 1:
                    _st_data.thresholdChrg1 = value;
                    break;
            }
            goto _escape;
        }
        if(function == p_result[i].var_gain) {
            switch(i) {
                case 0:
                    _st_data.var_gain0 = value;
                    break;
                case 1:
                    _st_data.var_gain1 = value;
                    break;
            }
            goto _escape;
        }

        for(int j = 0; j < NUM_PROBE; j++) {
            if(function == p_cavCalibCoeff[i][j]) {
                unsigned raw;
                raw = _pcav->setCalibCoeff(i, j, value);
                setIntegerParam(p_cavCalibCoeffRaw[i][j], raw);
                goto _escape;
            }
        }
    }


    _escape:
    callParamCallbacks();

    return status;
}

static epicsFloat64* __zero_pad(epicsFloat64 *value, epicsFloat64 *v, size_t nElements)
{
    if(nElements >= MAX_SAMPLES) return value;

    int i;
    for(i = 0; i < nElements;   i++)  *(v + i) = *(value + i);
    for(     ; i < MAX_SAMPLES; i++)  *(v + i) = *(value + i);

    return v;
}


asynStatus pcavAsynDriver::writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements)
{
    int        function      = pasynUser->reason;
    asynStatus status        = asynSuccess;
    const char *functionName = "writeFloat64Array";

    epicsFloat64 v[MAX_SAMPLES];

    if(function == i_baseband_wf) {
        _dacSigGen->setIWaveform(__zero_pad(value, v, nElements));
    } 
    else if(function == q_baseband_wf) {
        _dacSigGen->setQWaveform(__zero_pad(value, v, nElements));
    }


    return status;
}


void pcavAsynDriver::report(int interest)
{
    printf("\tpcavAsyn: version            : %d (%x)\n", version, version);
    printf("\tpcavAsyn: poll count         : %u\n", pollCnt);
    printf("\tpcavAsyn: bstream poll count : %u\n", streamPollCnt);
    printf("\tpcavAsyn: bstream read size  : %u\n", stream_read_size);
    if(interest < 5) return;

    char ts_str[80];
    uint32_t t =0, u =1;
    epicsTimeToStrftime(ts_str, sizeof(ts_str), "%Y/%m/%d %H:%M:%S.%09f", &((bsss_buf + current_bsss)->time));
    printf("\tpcavAsyn: BSSS stream buffer(%d)\n",     current_bsss);
    printf("\t          timestamp          : %s\n",    ts_str);
    printf("\t          pulse id           : %lu\n",   (bsss_buf + current_bsss)->pulse_id);
    printf("\t          channel mask       : %8.8x\n", (bsss_buf + current_bsss)->chn_mask);
    printf("\t          severity mask      : %8.8x\n", (bsss_buf + current_bsss)->srv_mask);

    for(int i = 0; i < 32; i++) {
        if((bsss_buf + current_bsss)->chn_mask & u) t++;
        u <<= 1;
    }

    for(u = 0; u < t; u++) printf("\t          payload[%d]         : %8.8x\n", u, (bsss_buf + current_bsss)->payload[u]);
    printf("\t          valid channel      : %8.8x\n", (bsss_buf + current_bsss)->payload[u]);
}

void pcavAsynDriver::poll(void)
{
    pollCnt++;
    monitor();
    callParamCallbacks();
}

void pcavAsynDriver::pollStream(void)
{
    while(stream) {
        streamPollCnt++;
        current_bsss++;
        current_bsss = (current_bsss < MAX_BSSS_BUF)? current_bsss: 0;
        bsss_packet_t *p = bsss_buf + current_bsss;
        stream_read_size = _bstream->read((uint8_t *) p, sizeof(bsss_packet_t), CTimeout());
        _SWAP_TIMESTAMP(&(p->time));
        setTimeStamp(&(p->time));

        if(p->chn_mask == 0xffff && p->payload[16]) {
            calcBldData(p);
            sendBldPacket(p);
            pushBsaValues(p);
            updateFastPVs();
        }
    }


}


void pcavAsynDriver::calcBldData(bsss_packet_t *p)
{
    _ref.phase  = _FIX_18_16(p->payload[1]);
    _c0p0.phase = _FIX_18_16(p->payload[2]);
    _c0p0.ampl  = p->payload[3];
    _c0p1.phase = _FIX_18_16(p->payload[6]);
    _c0p1.ampl  = p->payload[7];
    _c1p0.phase = _FIX_18_16(p->payload[10]);
    _c1p0.ampl  = p->payload[11];
    _c1p1.phase = _FIX_18_16(p->payload[14]);
    _c1p1.ampl  = p->payload[15];

    _bld_data.time0 = (0.5 * (_c0p0.phase + _c0p1.phase) - _ref.phase) * 1.E+6 / 2852.;
    _bld_data.time1 = (0.5 * (_c1p0.phase + _c1p1.phase) - _ref.phase) * 1.E+6 / 2852.;
    _bld_data.charge0 = 0.5 * (_c0p0.ampl + _c0p1.ampl);
    _bld_data.charge1 = 0.5 * (_c1p0.ampl + _c1p1.ampl);

    _bld_data.time0   = _coeff_time[0].a * _bld_data.time0 + _coeff_time[0].b;
    _bld_data.time1   = _coeff_time[1].a * _bld_data.time1 + _coeff_time[1].b;
    _bld_data.charge0 = _coeff_charge[0].a * _bld_data.charge0 + _coeff_charge[0].b;
    _bld_data.charge1 = _coeff_charge[1].a * _bld_data.charge1 + _coeff_charge[1].b;



#define  VAR_CALC(MEAS,GAIN,MEAN,VAR) \
{ \
    double v, v2; \
    if(isnan(MEAN) || isinf(MEAN)) (MEAN) = 0.; \
    if(isnan(VAR)  || isinf(VAR))  (VAR)  = 0.; \
    v = (MEAS) - (MEAN); \
    (MEAN) += (GAIN) * v; \
    v = (MEAS) - (MEAN); \
    v2 = v * v; \
    (VAR) += (GAIN) * (v2 - (VAR)); \
}


    if(_st_data.reset) {
        _st_data.reset = false;
        // post counters  before reset
        setIntegerParam(p_result[0].validCnt, _st_data.validCnt0);
        setIntegerParam(p_result[1].validCnt, _st_data.validCnt1);
        setIntegerParam(p_result[0].invalidCnt, _st_data.invalidCnt0);
        setIntegerParam(p_result[1].invalidCnt, _st_data.invalidCnt1);

        _st_data.validCnt0   = _st_data.validCnt1   = 0;  // reset valid counter
        _st_data.invalidCnt0 = _st_data.invalidCnt1 = 0;  // reset invalid counter
    }

    if(_bld_data.charge0 >= _st_data.thresholdChrg0) {
        _st_data.validCnt0++;
        VAR_CALC(_bld_data.time0,   _st_data.var_gain0, _st_data.avg_time0,   _st_data.var_time0);
        VAR_CALC(_bld_data.charge0, _st_data.var_gain0, _st_data.avg_charge0, _st_data.var_charge0);
        _st_data.rms_time0   = sqrt(_st_data.var_time0);
        _st_data.rms_charge0 = sqrt(_st_data.var_charge0);

    } else _st_data.invalidCnt0++;

    if(_bld_data.charge1 >= _st_data.thresholdChrg1) {
        _st_data.validCnt1++;
        VAR_CALC(_bld_data.time1,   _st_data.var_gain1, _st_data.avg_time1,   _st_data.var_time1);
        VAR_CALC(_bld_data.charge1, _st_data.var_gain1, _st_data.avg_charge1, _st_data.var_charge1);
    } else _st_data.invalidCnt1++;



}



void pcavAsynDriver::pushBsaValues(bsss_packet_t *p)
{
    BSA_StoreData(BsaChn_time[0], p->time, _bld_data.time0, 0, 0);
    BSA_StoreData(BsaChn_time[1], p->time, _bld_data.time1, 0, 0);
    BSA_StoreData(BsaChn_charge[0], p->time, _bld_data.charge0, 0, 0);
    BSA_StoreData(BsaChn_charge[1], p->time, _bld_data.charge1, 0, 0);
}


void pcavAsynDriver::updateFastPVs(void)
{

    setDoubleParam(p_result[0].avg_time, _st_data.avg_time0);
    setDoubleParam(p_result[1].avg_time, _st_data.avg_time1);
    setDoubleParam(p_result[0].avg_charge, _st_data.avg_charge0);
    setDoubleParam(p_result[1].avg_charge, _st_data.avg_charge1);

    setDoubleParam(p_result[0].rms_time, _st_data.rms_time0);
    setDoubleParam(p_result[1].rms_time, _st_data.rms_time1);
    setDoubleParam(p_result[0].rms_charge, _st_data.rms_charge0);
    setDoubleParam(p_result[1].rms_charge, _st_data.rms_charge1);

    setDoubleParam(p_result[0].raw_time, _bld_data.time0);
    setDoubleParam(p_result[1].raw_time, _bld_data.time1);
    setDoubleParam(p_result[0].raw_charge, _bld_data.charge0);
    setDoubleParam(p_result[1].raw_charge, _bld_data.charge1);


    callParamCallbacks();
}


void pcavAsynDriver::sendBldPacket(bsss_packet_t *p)
{
    unsigned int srcPhysicalID = 1;
    unsigned int xtcType       = 0x10000 | 0x0010;

    BldSendPacket(0, srcPhysicalID, xtcType, &(p->time), &_bld_data, sizeof(_bld_data));

}


/*    // test  cpde for BLD packet send
void pcavAsynDriver::sendBldPacket(bsss_packet_t *p)
{
    static uint32_t cnt = 0;
    static uint32_t a[4];

    a[0] = cnt++; a[1] = cnt++; a[2] = cnt++; a[3] = cnt++;

    size_t       size          = 16;
    unsigned int srcPhysicalID = 1;
    unsigned int xtcType       = 0x10000 | 0x0010;
    BldSendPacket(0, srcPhysicalID, xtcType, &(p->time), a, size);

}
*/

void pcavAsynDriver::ParameterSetup(void)
{
    char param_name[80];
    char raw_param_name[83];
    sprintf(param_name,    PCAV_VERSION_STR); createParam(param_name, asynParamInt32, &p_version);

    // rf reference monitoring
    sprintf(param_name,     RFREF_AMPL_STR);            createParam(param_name,     asynParamFloat64, &(p_rfRefAmpl.val));
    sprintf(raw_param_name, RAW_PARAM_STR, param_name); createParam(raw_param_name, asynParamInt32,   &(p_rfRefAmpl.raw));
    sprintf(param_name,     RFREF_PHASE_STR);           createParam(param_name,     asynParamFloat64, &(p_rfRefPhase.val));
    sprintf(raw_param_name, RAW_PARAM_STR, param_name); createParam(raw_param_name, asynParamInt32,   &(p_rfRefPhase.raw));
    sprintf(param_name,     RFREF_I_STR);               createParam(param_name,     asynParamFloat64, &(p_rfRefI.val));
    sprintf(raw_param_name, RAW_PARAM_STR, param_name); createParam(raw_param_name, asynParamInt32,   &(p_rfRefI.raw));
    sprintf(param_name,     RFREF_Q_STR);               createParam(param_name,     asynParamFloat64, &(p_rfRefQ.val));
    sprintf(raw_param_name, RAW_PARAM_STR, param_name); createParam(raw_param_name, asynParamInt32,   &(p_rfRefQ.raw));
    // rf reference control
    sprintf(param_name,     RFREF_SEL_STR);             createParam(param_name,     asynParamInt32,   &p_rfRefSel);

    for(int i = 0; i < NUM_WFDATA; i++) {
        sprintf(param_name, WFDATA_SEL_STR, i);             createParam(param_name,     asynParamInt32,   &(p_wfDataSel[i]));
    }



    for(int cav = 0; cav < NUM_CAV; cav++) {
        for(int probe = 0; probe < NUM_PROBE; probe++) {
            // cavitiy monitor
            sprintf(param_name,     CAV_IF_AMPL_STR,      cav, probe); createParam(param_name,     asynParamFloat64, &(p_cavIfAmpl[cav][probe].val));
            sprintf(raw_param_name, RAW_PARAM_STR,        param_name); createParam(raw_param_name, asynParamInt32,   &(p_cavIfAmpl[cav][probe].raw));
            sprintf(param_name,     CAV_IF_PHASE_STR,     cav, probe); createParam(param_name,     asynParamFloat64, &(p_cavIfPhase[cav][probe].val));
            sprintf(raw_param_name, RAW_PARAM_STR,        param_name); createParam(raw_param_name, asynParamInt32,   &(p_cavIfPhase[cav][probe].raw));
            sprintf(param_name,     CAV_IF_I_STR,         cav, probe); createParam(param_name,     asynParamFloat64, &(p_cavIfI[cav][probe].val));
            sprintf(raw_param_name, RAW_PARAM_STR,        param_name); createParam(raw_param_name, asynParamInt32,   &(p_cavIfI[cav][probe].raw));
            sprintf(param_name,     CAV_IF_Q_STR,         cav, probe); createParam(param_name,     asynParamFloat64, &(p_cavIfQ[cav][probe].val));
            sprintf(raw_param_name, RAW_PARAM_STR,        param_name); createParam(raw_param_name, asynParamInt32,   &(p_cavIfQ[cav][probe].raw));
            sprintf(param_name,     CAV_DCREAL_STR,       cav, probe); createParam(param_name,     asynParamFloat64, &(p_cavDCReal[cav][probe].val));
            sprintf(raw_param_name, RAW_PARAM_STR,        param_name); createParam(raw_param_name, asynParamInt32,   &(p_cavDCReal[cav][probe].raw));
            sprintf(param_name,     CAV_DCIMAGE_STR,      cav, probe); createParam(param_name,     asynParamFloat64, &(p_cavDCImage[cav][probe].val));
            sprintf(raw_param_name, RAW_PARAM_STR,        param_name); createParam(raw_param_name, asynParamInt32,   &(p_cavDCImage[cav][probe].raw));
            sprintf(param_name,     CAV_DCFREQ_STR,       cav, probe); createParam(param_name,     asynParamFloat64, &(p_cavDCFreq[cav][probe].val));
            sprintf(raw_param_name, RAW_PARAM_STR,        param_name); createParam(raw_param_name, asynParamInt32,   &(p_cavDCFreq[cav][probe].raw));
            sprintf(param_name,     CAV_INTEG_I_STR,      cav, probe); createParam(param_name,     asynParamFloat64, &(p_cavIntegI[cav][probe].val));
            sprintf(raw_param_name, RAW_PARAM_STR,        param_name); createParam(raw_param_name, asynParamInt32,   &(p_cavIntegI[cav][probe].raw));
            sprintf(param_name,     CAV_INTEG_Q_STR,      cav, probe); createParam(param_name,     asynParamFloat64, &(p_cavIntegQ[cav][probe].val));
            sprintf(raw_param_name, RAW_PARAM_STR,        param_name); createParam(raw_param_name, asynParamInt32,   &(p_cavIntegQ[cav][probe].raw));
            sprintf(param_name,     CAV_OUT_PHASE_STR,    cav, probe); createParam(param_name,     asynParamFloat64, &(p_cavOutPhase[cav][probe].val));
            sprintf(raw_param_name, RAW_PARAM_STR,        param_name); createParam(raw_param_name, asynParamInt32,   &(p_cavOutPhase[cav][probe].raw));
            sprintf(param_name,     CAV_OUT_AMPL_STR,     cav, probe); createParam(param_name,     asynParamFloat64, &(p_cavOutAmpl[cav][probe].val));
            sprintf(raw_param_name, RAW_PARAM_STR,        param_name); createParam(raw_param_name, asynParamInt32,   &(p_cavOutAmpl[cav][probe].raw));

            sprintf(param_name,     CAV_COMP_PHASE_STR,   cav, probe); createParam(param_name,     asynParamFloat64, &(p_cavCompPhase[cav][probe].val));
            sprintf(raw_param_name, RAW_PARAM_STR,        param_name); createParam(raw_param_name, asynParamInt32,   &(p_cavCompPhase[cav][probe].raw));

            // cavity control, per probe
            sprintf(param_name,     CAV_CHANSEL_STR,      cav, probe); createParam(param_name,     asynParamInt32,   &(p_cavChanSel[cav][probe]));
            sprintf(param_name,     CAV_WINDOW_START_STR, cav, probe); createParam(param_name,     asynParamInt32,   &(p_cavWindowStart[cav][probe]));
            sprintf(param_name,     CAV_WINDOW_END_STR,   cav, probe); createParam(param_name,     asynParamInt32,   &(p_cavWindowEnd[cav][probe]));
            sprintf(param_name,     CAV_CALIB_COEFF_STR,  cav, probe); createParam(param_name,     asynParamFloat64, &(p_cavCalibCoeff[cav][probe]));
            sprintf(param_name,     CAV_CALIB_COEFF_RAW_STR, cav, probe); createParam(param_name,  asynParamInt32,   &(p_cavCalibCoeffRaw[cav][probe]));
        }
        // cavity control, per cavity
        sprintf(param_name, CAV_NCO_PHASE_ADJ_STR, cav); createParam(param_name, asynParamFloat64, &(p_cavNCOPhaseAdj[cav]));
        sprintf(param_name, CAV_FREQ_EVAL_START_STR, cav); createParam(param_name, asynParamInt32, &(p_cavFreqEvalStart[cav]));
        sprintf(param_name, CAV_FREQ_EVAL_END_STR,   cav); createParam(param_name, asynParamInt32, &(p_cavFreqEvalEnd[cav]));
        sprintf(param_name, CAV_REG_LATCH_POINT_STR, cav); createParam(param_name, asynParamInt32, &(p_cavRegLatchPoint[cav]));

        // NCO raw set value
        sprintf(param_name, CAV_NCO_RAW_STR, cav);       createParam(param_name, asynParamInt32,   &(p_cavNCORaw[cav]));

        // linear conversion for BLD data
        sprintf(param_name, COEFF_TIME_A_STR, cav);      createParam(param_name, asynParamFloat64,  &(p_coeff_time[cav].a));
        sprintf(param_name, COEFF_TIME_B_STR, cav);      createParam(param_name, asynParamFloat64,  &(p_coeff_time[cav].b));
        sprintf(param_name, COEFF_CHARGE_A_STR, cav);    createParam(param_name, asynParamFloat64,  &(p_coeff_charge[cav].a));
        sprintf(param_name, COEFF_CHARGE_B_STR, cav);    createParam(param_name, asynParamFloat64,  &(p_coeff_charge[cav].b));
        // time and charge
        sprintf(param_name, TIME_STR, cav);              createParam(param_name, asynParamFloat64,  &(p_result[cav].avg_time));
        sprintf(param_name, CHARGE_STR, cav);            createParam(param_name, asynParamFloat64,  &(p_result[cav].avg_charge));
        sprintf(param_name, RAW_TIME_STR, cav);          createParam(param_name, asynParamFloat64,  &(p_result[cav].raw_time));
        sprintf(param_name, RAW_CHARGE_STR, cav);        createParam(param_name, asynParamFloat64,  &(p_result[cav].raw_charge));
        sprintf(param_name, RMS_TIME_STR, cav);          createParam(param_name, asynParamFloat64,  &(p_result[cav].rms_time));
        sprintf(param_name, RMS_CHARGE_STR, cav);        createParam(param_name, asynParamFloat64,  &(p_result[cav].rms_charge));

        sprintf(param_name, VALID_CNT_STR, cav);         createParam(param_name, asynParamInt32,    &(p_result[cav].validCnt));
        sprintf(param_name, INVALID_CNT_STR, cav);       createParam(param_name, asynParamInt32,    &(p_result[cav].invalidCnt));
        sprintf(param_name, THRESHOLD_CHRG_STR, cav);    createParam(param_name, asynParamFloat64,  &(p_result[cav].threshold));
        sprintf(param_name, VAR_GAIN_STR, cav);          createParam(param_name, asynParamFloat64,  &(p_result[cav].var_gain));

    }

    sprintf(param_name, RESET_STR);      createParam(param_name, asynParamInt32,        &(p_reset));
    // DacSigGen, baseline I&Q waveforms
    sprintf(param_name, I_BASEBAND_STR); createParam(param_name, asynParamFloat64Array, &(i_baseband_wf));
    sprintf(param_name, Q_BASEBAND_STR); createParam(param_name, asynParamFloat64Array, &(q_baseband_wf));
    
}

void pcavAsynDriver::bsaSetup(void)
{
    char param_name[120];

    BSA_ConfigSetAllPriorites(90);

    for(int i = 0; i < NUM_CAV; i++) {
        sprintf(param_name, BSA_TIME_STR,   bsa_name, i);  BsaChn_time[i]   = BSA_CreateChannel(param_name);
        sprintf(param_name, BSA_CHARGE_STR, bsa_name, i);  BsaChn_charge[i] = BSA_CreateChannel(param_name);
    }
}


void pcavAsynDriver::monitor(void)
{
    double      val;
    int32_t     raw;


    _pcav->getVersion(&version); setIntegerParam(p_version, version);

    val = _pcav->getRefAmpl(&raw);   setDoubleParam(p_rfRefAmpl.val,  val); setIntegerParam(p_rfRefAmpl.raw,  raw);
    val = _pcav->getRefPhase(&raw);  setDoubleParam(p_rfRefPhase.val, val); setIntegerParam(p_rfRefPhase.raw, raw);  
    val = _pcav->getRefI(&raw);      setDoubleParam(p_rfRefI.val,     val); setIntegerParam(p_rfRefI.raw,     raw);
    val = _pcav->getRefQ(&raw);      setDoubleParam(p_rfRefQ.val,     val); setIntegerParam(p_rfRefQ.raw,     raw);

    for(int i = 0; i < NUM_CAV; i++) {
        for(int j = 0; j < NUM_PROBE; j++) {
            val = _pcav->getIfAmpl(i, j, &raw);     setDoubleParam(p_cavIfAmpl[i][j].val,    val); setIntegerParam(p_cavIfAmpl[i][j].raw,    raw);
            val = _pcav->getIfPhase(i, j, &raw);    setDoubleParam(p_cavIfPhase[i][j].val,   val); setIntegerParam(p_cavIfPhase[i][j].raw,   raw);
            val = _pcav->getIfI(i, j, &raw);        setDoubleParam(p_cavIfI[i][j].val,       val); setIntegerParam(p_cavIfI[i][j].raw,       raw);
            val = _pcav->getIfQ(i, j, &raw);        setDoubleParam(p_cavIfQ[i][j].val,       val); setIntegerParam(p_cavIfQ[i][j].raw,       raw);
            val = _pcav->getDCReal(i, j, &raw);     setDoubleParam(p_cavDCReal[i][j].val,    val); setIntegerParam(p_cavDCReal[i][j].raw,    raw);
            val = _pcav->getDCImage(i, j, &raw);    setDoubleParam(p_cavDCImage[i][j].val,   val); setIntegerParam(p_cavDCImage[i][j].raw,   raw);
            val = _pcav->getDCFreq(i, j, &raw);     setDoubleParam(p_cavDCFreq[i][j].val,    val); setIntegerParam(p_cavDCFreq[i][j].raw,    raw);
            val = _pcav->getIntegI(i, j, &raw);     setDoubleParam(p_cavIntegI[i][j].val,    val); setIntegerParam(p_cavIntegI[i][j].raw,    raw);
            val = _pcav->getIntegQ(i, j, &raw);     setDoubleParam(p_cavIntegQ[i][j].val,    val); setIntegerParam(p_cavIntegQ[i][j].raw,    raw);
            val = _pcav->getOutPhase(i, j, &raw);   setDoubleParam(p_cavOutPhase[i][j].val,  val); setIntegerParam(p_cavOutPhase[i][j].raw,  raw);
            val = _pcav->getOutAmpl(i, j, &raw);    setDoubleParam(p_cavOutAmpl[i][j].val,   val); setIntegerParam(p_cavOutAmpl[i][j].raw,   raw);

            val = _pcav->getCompPhase(i, j, &raw);  setDoubleParam(p_cavCompPhase[i][j].val, val); setIntegerParam(p_cavCompPhase[i][j].raw, raw);
        }
    }


}


extern "C" {

int pcavAsynDriverConfigure(const char *portName, const char *regPathString, const char *bsaStream, const char *bsaPrefix, const char *named_root)
{
    init_drvList();

    pDrvList_t *p = find_drvByPort(portName);
    if(p) {
        printf("pcavAsynDriver found the port name (%s) has been used.\n", portName);
        return 0;
    }

    p = (pDrvList_t *) mallocMustSucceed(sizeof(pDrvList_t), "pcavAsynDriver: pcavAsynDriverConfigure()");
    p->named_root = (named_root && strlen(named_root))?epicsStrDup(named_root):cpswGetRootName();
    p->port       = epicsStrDup(portName);
    p->regPath    = epicsStrDup(regPathString);
    p->bsaStream  = (bsaStream && strlen(bsaStream))? epicsStrDup(bsaStream): NULL;
    p->bsaPrefix  = (bsaPrefix && strlen(bsaPrefix))? epicsStrDup(bsaPrefix): NULL;
    p->pcavAsyn   = new pcavAsynDriver((void *) p, 
                                       (const char *) p->port, 
                                       (const char *) p->regPath, 
                                       (const char *) p->bsaStream, 
                                       (const char *) p->bsaPrefix, 
                                       (const char *) p->named_root);
    p->prv        = NULL;

    ellAdd(pDrvEllList, &p->node);

    return 0;
}


// prepare ioc command for driver initialization
static const iocshArg initArg0 = {"port name",     iocshArgString};
static const iocshArg initArg1 = {"register path", iocshArgString};
static const iocshArg initArg2 = {"bsa stream",    iocshArgString};
static const iocshArg initArg3 = {"bsa prefix",    iocshArgString};
static const iocshArg initArg4 = {"named_root",    iocshArgString};
static const iocshArg * const initArgs[] = { &initArg0,
                                             &initArg1,
                                             &initArg2,
                                             &initArg3,
                                             &initArg4 };
static const iocshFuncDef initFuncDef = {"pcavAsynDriverConfigure", 5, initArgs};
static void  initCallFunc(const iocshArgBuf *args)
{
    pcavAsynDriverConfigure(args[0].sval,
                            args[1].sval,
                            (args[2].sval && strlen(args[2].sval))? args[2].sval: NULL,
                            (args[3].sval && strlen(args[3].sval))? args[3].sval: NULL,
                            (args[4].sval && strlen(args[4].sval))? args[4].sval: NULL);
}

static void pcavAsynDriverRegister(void)
{
    iocshRegister(&initFuncDef, initCallFunc);
}

epicsExportRegistrar(pcavAsynDriverRegister);


static int pcavAsynDriverStreamPoll(void *p)
{
    ((pcavAsynDriver*)p)->pollStream();

    return 0;
}

static int pcavAsynDriverPoll(void)
{
    while(keep_stay_in_loop) {
        pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);
        while(p) {
            if(p->pcavAsyn) p->pcavAsyn->poll();
            p = (pDrvList_t *) ellNext(&p->node);
        }
        epicsThreadSleep(1.);
    }

    epicsEventSignal(shutdownEvent);

    return 0;
    
}

static void stopPollingThread(void *p)
{
    keep_stay_in_loop = false;
    epicsEventWait(shutdownEvent);
    epicsPrintf("pcavAsynDriver: stop polling thread (%s)\n", (char*) p);
}


// EPICS driver support for pcavAsynDriver
static int pcavAsynDriverReport(int interest);
static int pcavAsynDriverInitialize(void);

static struct drvet pcavAsynDriver = {
    2,
    (DRVSUPFUN) pcavAsynDriverReport,
    (DRVSUPFUN) pcavAsynDriverInitialize
};   

epicsExportAddress(drvet, pcavAsynDriver);

static int pcavAsynDriverReport(int interest)
{
    init_drvList();
    printf("Total %d of pcavAsynDriver instance(s) is (are) registered\n", ellCount(pDrvEllList));

    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);
    while(p) {
        printf("\tnamed root    : %s\n", p->named_root);
        printf("\tport name     : %s\n", p->port);
        printf("\tregister path : %s\n", p->regPath);
        printf("\tbsa stream    : %s\n", p->bsaStream);
        printf("\tpcavAsyn      : %p\n", p->pcavAsyn);
        if(p->pcavAsyn) p->pcavAsyn->report(interest);
        p = (pDrvList_t *) ellNext(&p->node);
    }

    return 0;
}

static int pcavAsynDriverInitialize(void)
{
    init_drvList();

    if(!pDrvEllList) {
        printf("pcavAsynDriver never been configured\n");
        return 0;
    }

    keep_stay_in_loop = true;
    shutdownEvent     = epicsEventMustCreate(epicsEventEmpty);
    const char *name  = "pcavAsynDriver";

    epicsThreadCreate(name, epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) pcavAsynDriverPoll, 0);

    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);
    while(p) {
        if(p->pcavAsyn && p->bsaStream) {
            char name[80];
            sprintf(name, "bstream_%s", p->port);

            epicsThreadCreate(name, epicsThreadPriorityHigh,
                              epicsThreadGetStackSize(epicsThreadStackMedium),
                              (EPICSTHREADFUNC) pcavAsynDriverStreamPoll, (void*) p->pcavAsyn);
        }

        p = (pDrvList_t*) ellNext(&p->node);

    }

    epicsAtExit3((epicsExitFunc) stopPollingThread, (void*) epicsStrDup(name), epicsStrDup(name));

    return 0;
}



} /* extern C */





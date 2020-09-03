#ifndef _PCAVASYN_H
#define _PCAVASYN_H

#include <asynPortDriver.h>
#include <epicsEvent.h>
#include <epicsTypes.h>
#include <epicsTime.h>

#include <cpsw_api_user.h>
#include <pcavFw.h>
#include <dacSigGenFw.h>

#include <vector>
#include <string>
#include <dlfcn.h>

#include <stdio.h>
#include <sstream>
#include <fstream>


#include "BsaApi.h"


#define NUM_CAV         2       // number of cavities
#define NUM_PROBE       2       // number of probes per cavity 
#define MAX_BSSS_BUF   36


#define NUM_WFDATA      8

typedef struct {
    int raw;
    int val;
} pcavMon;

typedef struct {
    epicsTimeStamp   time;
    uint64_t         pulse_id;
    uint32_t         chn_mask;
    uint32_t         srv_mask;
    uint32_t         payload[32+1];
} bsss_packet_t;

#define _SWAP_TIMESTAMP(TS)             \
{    epicsUInt32 t = (TS)->nsec;        \
     (TS)->nsec = (TS)->secPastEpoch;   \
     (TS)->secPastEpoch = t;            }

#define _FIXED18_16_PHASE(P) \
(((P) & 0x20000) ? ((double)(P) - (double)(0x40000)):(double)(P));

#define _FIX_18_18(V) \
(((V) & 0x20000) ? ((double)(V) - (double)(0x40000))/(double)(0x40000):(double)(V)/(double)(0x40000))

#define _FIX_18_17(V) \
(((V) & 0x20000) ? ((double)(V) - (double)(0x40000))/(double)(0x20000):(double)(V)/(double)(0x20000))

#define _FIX_18_16(V) \
(((V) & 0x20000) ? ((double)(V) - (double)(0x40000))/(double)(0x10000):(double)(V)/(double)(0x10000))

#define _FIX_18_15(V) \
(((V) & 0x20000) ? ((double)(V) - (double)(0x40000))/(double)(0x8000):(double)(V)/(double)(0x8000))




//double phase = (p->payload[1] & 0x20000) ? (p->payload[1]-0x40000) : p->payload[1];
  


class pcavAsynDriver
    :asynPortDriver {
    public:
        pcavAsynDriver(void *pDrv, const char *portName, const char *pathString, const char *bsaStream, const char *bsaPrefix,  const char *named_root = NULL);
        ~pcavAsynDriver();
        asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
        asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
        asynStatus writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements);
        void report(int interest);
        void poll(void);
        void pollStream(void);
        void calcBldData(bsss_packet_t *p);
        void sendBldPacket(bsss_packet_t *p);
        void pushBsaValues(bsss_packet_t *p);
        void updateFastPVs(void);

    private:
        void        *pDrv;
        const char  *port;
        const char  *path;
        const char  *stream;
        const char  *bsa_name;
        pcavFw      _pcav;
        dacSigGenFw _dacSigGen;
        Stream      _bstream;
        int32_t     version;
        uint32_t    pollCnt;
        uint32_t    streamPollCnt;
        uint32_t    stream_read_size;

        int             current_bsss;
        bsss_packet_t   bsss_buf[MAX_BSSS_BUF];

        struct {
            double phase;
            double ampl;
        } _ref, _c0p0, _c0p1, _c1p0, _c1p1;

        struct {
            double time0;
            double time1;
            double charge0;
            double charge1;
        } _bld_data;

       struct {
           double a;
           double b;
       } _coeff_time[NUM_CAV], _coeff_charge[NUM_CAV];


       BsaChannel BsaChn_time[NUM_CAV];
       BsaChannel BsaChn_charge[NUM_CAV];
        

        void ParameterSetup(void);
        void bsaSetup(void);
        void monitor(void);


    protected:
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
        int firstPcavParam;
#define FIRST_PCAV_PARAM   firstPcavsParam
#endif /* ASYN VERSION CHECK under 4.32 */
        int     p_version;
        // rf reference 
        pcavMon p_rfRefAmpl;
        pcavMon p_rfRefPhase;
        pcavMon p_rfRefI;
        pcavMon p_rfRefQ;
        int     p_rfRefSel;

        int     p_wfDataSel[NUM_WFDATA];


        // 2 cavities, 2 probes
        pcavMon p_cavIfAmpl[NUM_CAV][NUM_PROBE];
        pcavMon p_cavIfPhase[NUM_CAV][NUM_PROBE];
        pcavMon p_cavIfI[NUM_CAV][NUM_PROBE];
        pcavMon p_cavIfQ[NUM_CAV][NUM_PROBE];
        int     p_cavChanSel[NUM_CAV][NUM_PROBE];
        pcavMon p_cavDCReal[NUM_CAV][NUM_PROBE];
        pcavMon p_cavDCImage[NUM_CAV][NUM_PROBE];
        int     p_cavWindowStart[NUM_CAV][NUM_PROBE];
        int     p_cavWindowEnd[NUM_CAV][NUM_PROBE];
        pcavMon p_cavDCFreq[NUM_CAV][NUM_PROBE];
        pcavMon p_cavIntegI[NUM_CAV][NUM_PROBE];
        pcavMon p_cavIntegQ[NUM_CAV][NUM_PROBE];
        pcavMon p_cavOutPhase[NUM_CAV][NUM_PROBE];
        pcavMon p_cavOutAmpl[NUM_CAV][NUM_PROBE];

        pcavMon p_cavCompPhase[NUM_CAV][NUM_PROBE];

        int     p_cavFreqEvalStart[NUM_CAV];
        int     p_cavFreqEvalEnd[NUM_CAV];
        int     p_cavRegLatchPoint[NUM_CAV];

        int     p_cavCalibCoeff[NUM_CAV][NUM_PROBE];
        int     p_cavCalibCoeffRaw[NUM_CAV][NUM_PROBE];

        // 2 cavities
        int     p_cavNCOPhaseAdj[NUM_CAV];
        int     p_cavNCORaw[NUM_CAV];

        struct {
            int a;
            int b;
        } p_coeff_time[NUM_CAV], p_coeff_charge[NUM_CAV];

        struct {
            int time;
            int charge;
        } p_result[NUM_CAV];

        // DacSigGen, baseline I&Q
        int i_baseband_wf;
        int q_baseband_wf;

#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
        int lastPcavParam;
#define LAST_PCAV_PARAM   lastPcavParam
#endif /* ASYN VERSION CHECK under 4.32 */


};


#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
#define NUM_PCAV_DET_PARAMS ((int)(&LAST_PCAV_PARAM - &FIRST_PCAV_PARAM - 1))
#endif /* ASYN VERSION CHECK under 4.32 */

#define PCAV_VERSION_STR          "version"

/* rf reference */

#define RFREF_AMPL_STR            "rfRefAmpl"
#define RFREF_PHASE_STR           "rfRefPhase"
#define RFREF_I_STR               "rfRefI"
#define RFREF_Q_STR               "rfRefQ"
#define RFREF_SEL_STR             "rfRefSel"

#define WFDATA_SEL_STR            "wfDataSel%d"


/*2 cavties and 2 probes */
#define CAV_IF_AMPL_STR           "cav%dP%dIfAmpl"
#define CAV_IF_PHASE_STR          "cav%dP%dIfPhase"
#define CAV_IF_I_STR              "cav%dP%dIfI"
#define CAV_IF_Q_STR              "cav%dP%dIfQ"
#define CAV_CHANSEL_STR           "cav%dP%dChanSel"
#define CAV_DCREAL_STR            "cav%dP%dDCReal"
#define CAV_DCIMAGE_STR           "cav%dP%dDCImage"
#define CAV_WINDOW_START_STR      "cav%dP%dWindowStart"
#define CAV_WINDOW_END_STR        "cav%dP%dWindowEnd"
#define CAV_DCFREQ_STR            "cav%dP%dDCFreq"
#define CAV_INTEG_I_STR           "cav%dP%dIntegI"
#define CAV_INTEG_Q_STR           "cav%dP%dIntegQ"
#define CAV_OUT_PHASE_STR         "cav%dP%dOutPhase"
#define CAV_OUT_AMPL_STR          "cav%dP%dOutAmpl"

#define CAV_COMP_PHASE_STR        "cav%dP%dCompPhase"

#define CAV_FREQ_EVAL_START_STR   "cav%dFreqEvalStart"
#define CAV_FREQ_EVAL_END_STR     "cav%dFreqEvalEnd"
#define CAV_REG_LATCH_POINT_STR   "cav%dRegLatchPoint"

#define CAV_CALIB_COEFF_STR       "cav%dP%dCalibCoeff"
#define CAV_CALIB_COEFF_RAW_STR   "cav%dP%dCalibCoeffRaw"

/* 2 cavities */
#define CAV_NCO_PHASE_ADJ_STR     "cav%dNCOPhaseAdj"
#define CAV_NCO_RAW_STR           "cav%dNCORaw"          // NOC raw set value to verify

/* linear conversion for BLD data */
#define COEFF_TIME_A_STR         "coeffATime%d"
#define COEFF_TIME_B_STR         "coeffBTime%d"
#define COEFF_CHARGE_A_STR       "coeffACharge%d"
#define COEFF_CHARGE_B_STR       "coeffBCharge%d"
#define TIME_STR                 "time%d"
#define CHARGE_STR               "charge%d"


/* bandband I & Q waveforms */
#define I_BASEBAND_STR            "i_baseband_wf"         // baseband i waveform, length 4096
#define Q_BASEBAND_STR            "q_baseband_wf"         // baseband q waveform, length 4096

#define RAW_PARAM_STR             "%sRaw"

// BSA name
#define BSA_TIME_STR              "%s:TIME%d"             // bsa name for time measurement
#define BSA_CHARGE_STR            "%s:CHRG%d"             // bsa name for charge measurement



#endif /* _PCAVASYN_H */

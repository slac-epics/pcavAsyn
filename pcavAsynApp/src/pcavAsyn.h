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


#define NUM_CAV         2       // number of cavities
#define NUM_PROBE       2       // number of probes per cavity 
#define MAX_BSSS_BUF   36

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
  


class pcavAsynDriver
    :asynPortDriver {
    public:
        pcavAsynDriver(void *pDrv, const char *portName, const char *pathString, const char *bsaStream, const char *named_root = NULL);
        ~pcavAsynDriver();
        asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
        asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
        asynStatus writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements);
        void report(int interest);
        void poll(void);
        void pollStream(void);

    private:
        void    *pDrv;
        char    *port;
        char    *path;
        char    *stream;
        pcavFw  _pcav;
        dacSigGenFw _dacSigGen;
        Stream  _bstream;
        int32_t version;
        uint32_t pollCnt;
        uint32_t streamPollCnt;
        uint32_t stream_read_size;

        int             current_bsss;
        bsss_packet_t   bsss_buf[MAX_BSSS_BUF];

        void ParameterSetup(void);
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
        int     p_rfRefWindowStart;
        int     p_rfRefWindowEnd;

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
        pcavMon p_cavCompI[NUM_CAV][NUM_PROBE];
        pcavMon p_cavCompQ[NUM_CAV][NUM_PROBE];
        pcavMon p_cavCompPhase[NUM_CAV][NUM_PROBE];
        pcavMon p_cavIfWf[NUM_CAV][NUM_PROBE];       // TBD  cavIfWf
        int     p_cavCalibCoeff[NUM_CAV][NUM_PROBE]; // TBD  cavCalibCoeff

        // 2 cavities
        int     p_cavNCOPhaseAdj[NUM_CAV];

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
#define RFREF_WINDOW_START_STR    "rfRefWindowStart"
#define RFREF_WINDOW_END_STR      "rfRefWindowEnd"


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
#define CAV_COMP_I_STR            "cav%dP%dCompI"
#define CAV_COMP_Q_STR            "cav%dP%dCompQ"
#define CAV_COMP_PHASE_STR        "cav%dP%dCompPhase"
#define CAV_IF_WF_STR             "cav%dP%dIfWf"         // space holder TBD
#define CAV_CALIB_COEFF_STR       "cav%dP%dCalibCoeff"   // space holder TBD

/* 2 cavities */
#define CAV_NCO_PHASE_ADJ_STR     "cav%dNCOPhaseAdj"

/* bandband I & Q waveforms */
#define I_BASEBAND_STR            "i_baseband_wf"         // baseband i waveform, length 4096
#define Q_BASEBAND_STR            "q_baseband_wf"         // baseband q waveform, length 4096

#define RAW_PARAM_STR             "%sRaw"



#endif /* _PCAVASYN_H */

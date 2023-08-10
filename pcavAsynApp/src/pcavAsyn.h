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

#define FLTBUF_LEN     1024     // length of fault buffer
#define MAX_FLTBUF     4        // number of fulat buffer


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

  


class pcavAsynDriver
    :asynPortDriver {
    public:
        pcavAsynDriver(void *pDrv, const char *portName, const char *pathString, const char *bsaStream, const char *bsaPrefix,  const char *named_root = NULL);
        ~pcavAsynDriver();
        asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
        asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
        asynStatus writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements);
        void report(int interest);
        void fast_poll(void);
        void poll(void);
        void pollStream(void);
        void calcBldData(bsss_packet_t *p);
        void pushCircularBuffer(bsss_packet_t *p);
        void postCircularBuffer(bsss_packet_t *p);
        void bsssWf(void);
        void sendBldPacket(bsss_packet_t *p);
        void pushBsaValues(bsss_packet_t *p);
        void updateFastPVs(void);

        void setMode(char mode) { p_mode = mode; }
        char getMode(         ) { return p_mode; }

    private:
        double getPhaseOffset(int cav, int probe);

        void        *pDrv;
        const char  *port;
        const char  *path;
        const char  *stream;
        const char  *bsa_name;
        pcavFw      _pcav;
        dacSigGenFw _dacSigGen;
        Stream      _bstream;
        int32_t     version;
        uint32_t    fast_pollCnt;
        uint32_t    pollCnt;
        uint32_t    streamPollCnt;
        uint32_t    stream_read_size;

        int             current_bsss;
        bsss_packet_t   bsss_buf[MAX_BSSS_BUF];

        struct {
            double phase_offset;
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
            bool      reset;   /* reset every 1 second */
            int       validCnt0;
            int       validCnt1;
            int       invalidCnt0;
            int       invalidCnt1;

            double    thresholdChrg0;
            double    thresholdChrg1;
            double    var_gain0;
            double    var_gain1;

            double    avg_time0;
            double    avg_time1;

            double    avg_charge0;
            double    avg_charge1;

            double    var_time0;
            double    var_time1;

            double    var_charge0;
            double    var_charge1;

            double    rms_time0;
            double    rms_time1;

            double    rms_charge0;
            double    rms_charge1;
        } _st_data;

       struct {
           double a;
           double b;
       } _coeff_time[NUM_CAV], _coeff_charge[NUM_CAV];

      struct {
          double probe[NUM_PROBE];
      } _weight[NUM_CAV];

       struct {
           double var_gain;
           double kp, ki, kd;
           struct {
               bool   enable;
               double bias;
               double prev_err;
               double prev_intg;
               double err;
               double intg;
               double derv;
               double output;
           } pid;
           struct {
               bool      valid;
               uint32_t  validCnt;
               uint32_t  invCnt;

               int32_t  dc_freq_raw;
               int32_t  ampl_raw;

               double dc_freq;
               double avg_dc_freq;
               double var_dc_freq;
               double rms_dc_freq;
               double ampl;
               double charge;
           } probe[NUM_PROBE];

           double dc_freq;
       } _nco_ctrl[NUM_CAV];


       BsaChannel BsaChn_time[NUM_CAV];
       BsaChannel BsaChn_charge[NUM_CAV];
       BsaChannel BsaChn_refPhase;
       BsaChannel BsaChn_phase[NUM_CAV][NUM_PROBE];
        

        double bsss_wf[64];

        struct {
            bool  active;
            bool  empty;
            bool  force_freeze;
            int    wp;
            
            bool valid0;
            bool valid1;

            double threshold;
            double latch_ncoPhase[2];

            double last_time0;
            double last_time1;

            double phase_c0p0[FLTBUF_LEN * 2];
            double phase_c0p1[FLTBUF_LEN * 2];
            double phase_c1p0[FLTBUF_LEN * 2];
            double phase_c1p1[FLTBUF_LEN * 2];

            double  ampl_c0p0[FLTBUF_LEN * 2];
            double  ampl_c0p1[FLTBUF_LEN * 2];
            double  ampl_c1p0[FLTBUF_LEN * 2];
            double  ampl_c1p1[FLTBUF_LEN * 2];

            double   time0[FLTBUF_LEN * 2];
            double   time1[FLTBUF_LEN * 2];
            double charge0[FLTBUF_LEN * 2];
            double charge1[FLTBUF_LEN * 2];
            double ncoPhase0[FLTBUF_LEN * 2];
            double ncoPhase1[FLTBUF_LEN * 2];

            unsigned int pulseid[FLTBUF_LEN *2];

        } _circular_buffer;

        void ParameterSetup(void);
        void bsaSetup(void);
        void monitor(void);
        void ncoPidCtrl(int cav);


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
        int     p_phaseOffset[NUM_CAV][NUM_PROBE];
        int     p_cavOutPhaseIU[NUM_CAV][NUM_PROBE];

        // 2 cavities
        int     p_cavNCOPhaseAdj[NUM_CAV];
        int     p_cavNCORaw[NUM_CAV];

        struct {
            int a;
            int b;
        } p_coeff_time[NUM_CAV], p_coeff_charge[NUM_CAV];

       struct {
           int probe[NUM_PROBE];
       } p_weight[NUM_CAV];

       struct {
           int ncoPidEnable;
           int kp, ki, kd;
           int var_gain;

           struct {
               int bias;
               int err;
               int intg;
               int derv;
               int output;
           } pid;

           struct {
               int valid_cnt;
               int inv_cnt;
               int mean_dcfreq;
               int rms_dcfreq;
           } probe[NUM_PROBE];

           int avg_dcfreq;
       } p_nco_ctrl[NUM_CAV];

        struct {
            int raw_time;
            int raw_charge;
            int avg_time;
            int avg_charge;
            int rms_time;
            int rms_charge;
            int validCnt;
            int invalidCnt;
            int threshold;
            int var_gain;
        } p_result[NUM_CAV];

        int p_reset;

        char p_mode;

        // DacSigGen, baseline I&Q
        int i_baseband_wf;
        int q_baseband_wf;


        int p_bsss_wf;


        int p_freeze_fltbuf;
        int p_clear_fltbuf;
        int p_thredtime_fltbuf;
        int p_fltbuf_phase[NUM_CAV][NUM_PROBE];
        int p_fltbuf_ampl[NUM_CAV][NUM_PROBE];
        int p_fltbuf_time[NUM_CAV];
        int p_fltbuf_charge[NUM_CAV];
        int p_fltbuf_ncoPhase[NUM_CAV];
        int p_fltbuf_pulseid;
        int p_fltbuf_status;
        int p_fltbuf_wrtpt;


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
#define CAV_OUT_PHASEIU_STR       "cav%dP%dOutPhaseIU"
#define CAV_OUT_AMPL_STR          "cav%dP%dOutAmpl"

#define CAV_COMP_PHASE_STR        "cav%dP%dCompPhase"

#define CAV_FREQ_EVAL_START_STR   "cav%dFreqEvalStart"
#define CAV_FREQ_EVAL_END_STR     "cav%dFreqEvalEnd"
#define CAV_REG_LATCH_POINT_STR   "cav%dRegLatchPoint"

#define CAV_CALIB_COEFF_STR       "cav%dP%dCalibCoeff"
#define CAV_CALIB_COEFF_RAW_STR   "cav%dP%dCalibCoeffRaw"
#define PHASE_OFFSET_STR          "cav%dP%dPhaseOffset"
/* 2 cavities */
#define CAV_NCO_PHASE_ADJ_STR     "cav%dNCOPhaseAdj"
#define CAV_NCO_RAW_STR           "cav%dNCORaw"          // NOC raw set value to verify

/* linear conversion for BLD data */
#define COEFF_TIME_A_STR         "coeffATime%d"
#define COEFF_TIME_B_STR         "coeffBTime%d"
#define COEFF_CHARGE_A_STR       "coeffACharge%d"
#define COEFF_CHARGE_B_STR       "coeffBCharge%d"

#define WEIGHT_CAV_PROBE_STR     "weightCav%dP%d"

#define TIME_STR                 "time%d"
#define CHARGE_STR               "charge%d"
#define RAW_TIME_STR             "raw_time%d"
#define RAW_CHARGE_STR           "raw_charge%d"
#define RMS_TIME_STR             "rms_time%d"
#define RMS_CHARGE_STR           "rms_charge%d"

#define VALID_CNT_STR            "validCnt%d"
#define INVALID_CNT_STR          "invalidCnt%d"

#define THRESHOLD_CHRG_STR       "thresholdChrg%d"
#define VAR_GAIN_STR             "var_gain%d"
#define RESET_STR                "reset"

/* bandband I & Q waveforms */
#define I_BASEBAND_STR            "i_baseband_wf"         // baseband i waveform, length 4096
#define Q_BASEBAND_STR            "q_baseband_wf"         // baseband q waveform, length 4096

#define RAW_PARAM_STR             "%sRaw"

// BSA name
#define BSA_TIME_STR              "%s:TIME%d"             // bsa name for time measurement
#define BSA_CHARGE_STR            "%s:CHRG%d"             // bsa name for charge measurement
#define BSA_PHASE_STR             "%s:PHS%d%d"            // bsa name for phase (cav,probe)
#define BSA_REF_PHASE_STR         "%s:REFPHS"             // bsa name for reference phase

// NCO PID control
#define KP_NCOPID_STR            "kp_nco_cav%d"
#define KI_NCOPID_STR            "ki_nco_cav%d"
#define KD_NCOPID_STR            "kd_nco_cav%d"

#define VAR_GAIN_NCOPID_STR            "var_gain_nco_cav%d"
#define VALIDCNT_NCOPID_PROBE_STR      "valid_cnt_nco_cav%dP%d"     // valid counter for each probe
#define INVCNT_NCOPID_PROBE_STR        "inv_cnt_nco_cav%dP%d"       // invalid counter for each probe
#define MEAN_DCFREQ_NCOPID_PROBE_STR   "mean_dcfreq_nco_cav%dP%d"   // mean of DC frequency for each probe
#define RMS_DCFREQ_NCOPID_PROBE_STR    "rms_dcfreq_nco_cav%dP%d"    // rms of DC freqency for reach probe

#define AVG_DCFREQ_NCOPID_STR          "avg_dcfreq_nco_cav%d"       // average DC frequency for each cavity

#define BIAS_NCOPID_STR                "bias_nco_cav%d"
#define ERR_NCOPID_STR                 "err_nco_cav%d"
#define INTG_NCOPID_STR                "intg_nco_cav%d"
#define DERV_NCOPID_STR                "derv_nco_cav%d"
#define OUTPUT_NCOPID_STR              "output_nco_cav%d"
#define NCOPID_ENABLE_STR              "pid_enable_nco_cav%d"


#define BSSS_WF_STR                    "bsss_wf"        // BSSS waveform beam rate update


#define FREEZE_FLTBUF_STR              "freeze_fltbuf"            // force freeze circular buffer
#define CLEAR_FLTBUF_STR               "clear_fltbuf"             // clear fault buffer
#define THREDTIME_FLTBUF_STR           "thredtime_fltbuf"         // threshold time jump
#define FLTBUF_PHASE_STR               "fltBuf_phase_cav%dP%d"    // fault buffer for phase
#define FLTBUF_AMPL_STR                "fltBuf_ampl_cav%dP%d"     // fault buffer for amplitude
#define FLTBUF_TIME_STR                "fltBuf_time_cav%d"        // fault buffer for arrival time
#define FLTBUF_CHARGE_STR              "fltBuf_charge_cav%d"      // fault buffer for charge
#define FLTBUF_NCOPHASE_STR            "fltBuf_ncoPhase_cav%d"    // fault buffer for NCO phase 
#define FLTBUF_PULSEID_STR             "fltBuf_pulseid"           // fault buffer for pulse id
#define FLTBUF_STATUS_STR              "fltBuf_status"            // status of fault buffer
#define FLTBUF_WRTPT_STR               "fltBuf_wrtpt"             // write pointer of fault buffer


#endif /* _PCAVASYN_H */

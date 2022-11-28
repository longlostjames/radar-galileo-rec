//  Include file for radar-galileo-rec.c
// MTF 2019-Oct-02 Changed config file name back to radar-galileo.conf 
#include <radar.h>

#define CHAN_Ic       0
#define CHAN_Qc       1
#define CHAN_Ix       2
#define CHAN_Qx       3
#define CHAN_T1       4
#define CHAN_T2       5
#define CHAN_INC      6
#define CHAN_V_not_H  7

// Trying swapped over co and cross-channels below
#define SWAP_CHAN_Ic       2
#define SWAP_CHAN_Qc       3
#define SWAP_CHAN_Ix       0
#define SWAP_CHAN_Qx       1

#define V_not_H_SAMPLE     0  /* Tx Pulse somewhere between gates 2 and 8             */
#define PowerAvStartSample 20 /* Power monitor sample holds have settled by this gate */
#define PowerAvEndSample   35
//#define PowerAvStartSample 3 /* Power monitor sample holds have settled by this gate */
//#define PowerAvEndSample   4

#define CAL_FILE    RADAR_DATA_PATH "radar-galileo/etc/radar-galileo.cal"
#define CONFIG_FILE RADAR_DATA_PATH "radar-galileo/etc/radar-galileo.conf"



enum PulseMode_en
{
    PM_Undefined0,
    PM_Single_H,     /* 1 */
    PM_Single_V,     /* 2 */
    PM_Single_HV,    /* 3 */
    PM_Double_H,     /* 4 */
    PM_Double_V,     /* 5 */
    PM_Double_HV_VH, /* 6 */
    PM_Double_HV_HV, /* 7 */
};

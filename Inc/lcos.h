#ifndef __LCOS_H
#define __LCOS_H

#include <stdint.h>
#include "stdio.h"

//#define CONFIG_PCLK_755M

#define CXD3554_I2C_ADDRESS 0x74
#define SXRD241_I2C_ADDRESS 0x92

#define SXRD241_REG_NUM		256
#define LED_REG_NUM       460
#define FRC_REG_NUM				512
#define MISC_REG_NUM			256

#define KST_REG_NUM				16
#define WP_REG_NUM				438
#define WEC_REG_NUM				912
#define BCHS_REG_NUM			73
#define CE_1D_REG_NUM		  16
#define CE_BC_REG_NUM		  4
#define CE_ACC_REG_NUM		61

#define CXD3554_MISC_BASEADDRESS 0x0000
#define CXD3554_INP_BASEADDRESS 0x1430
#define CXD3554_FRC_BASEADDRESS 0x2800
#define CXD3554_LED_BASEADDRESS 0x3200
#define CXD3554_LPDDR2_BASEADDRESS 0x0C00
#define CXD3554_DDR2_BASEADDRESS 0x0800
#define CXD3554_BCHS_BASEADDRESS 0x1600
#define CXD3554_CE1D_BASEADDRESS 0x1550
#define CXD3554_CEBC_BASEADDRESS 0x1580
#define CXD3554_CEACC_BASEADDRESS 0x1500

#define INTERNAL_PATTERN_RASTER 0x00
#define INTERNAL_PATTERN_WINDOW 0x01
#define INTERNAL_PATTERN_VERTICAL_stripe 0x02
#define INTERNAL_PATTERN_HORIZONTAL_STRIPE 0x03
#define INTERNAL_PATTERN_CROSSHATCH 0x04
#define INTERNAL_PATTERN_DOT 0x05
#define INTERNAL_PATTERN_CROSSHATCH_DOT 0x06
#define INTERNAL_PATTERN_HRAMP 0x07
#define INTERNAL_PATTERN_VRAMP 0x08
#define INTERNAL_PATTERN_FRAME 0x09
#define INTERNAL_PATTERN_CHECKBOX 0x0A
#define INTERNAL_PATTERN_COLORBAR 0x22


#define H2_FRC_BASEADDRESS 0x2800
#define H2_FRC_SF_P_COLOR0 (H2_FRC_BASEADDRESS + 0x1D)
#define H2_FRC_SF_P_COLOR1 (H2_FRC_BASEADDRESS + 0x1E)
#define H2_FRC_SF_P_COLOR2 (H2_FRC_BASEADDRESS + 0x1F)
#define H2_FRC_SF_P_COLOR3 (H2_FRC_BASEADDRESS + 0x20)
#define H2_FRC_SF_P_COLOR4 (H2_FRC_BASEADDRESS + 0x21)
#define H2_FRC_SF_P_COLOR5 (H2_FRC_BASEADDRESS + 0x22)
#define H2_FRC_SF_P_COLOR6 (H2_FRC_BASEADDRESS + 0x23)
#define H2_FRC_SF_P_COLOR7 (H2_FRC_BASEADDRESS + 0x24)

#define H2_FRC_SF_P_VTOTAL0_L (H2_FRC_BASEADDRESS + 0x4D)
#define H2_FRC_SF_P_VTOTAL0_H (H2_FRC_BASEADDRESS + 0x4E)
#define H2_FRC_SF_P_VTOTAL1_L (H2_FRC_BASEADDRESS + 0x4F)
#define H2_FRC_SF_P_VTOTAL1_H (H2_FRC_BASEADDRESS + 0x50)
#define H2_FRC_SF_P_VTOTAL2_L (H2_FRC_BASEADDRESS + 0x51)
#define H2_FRC_SF_P_VTOTAL2_H (H2_FRC_BASEADDRESS + 0x52)
#define H2_FRC_SF_P_VTOTAL3_L (H2_FRC_BASEADDRESS + 0x53)
#define H2_FRC_SF_P_VTOTAL3_H (H2_FRC_BASEADDRESS + 0x54)
#define H2_FRC_SF_P_VTOTAL4_L (H2_FRC_BASEADDRESS + 0x55)
#define H2_FRC_SF_P_VTOTAL4_H (H2_FRC_BASEADDRESS + 0x56)
#define H2_FRC_SF_P_VTOTAL5_L (H2_FRC_BASEADDRESS + 0x57)
#define H2_FRC_SF_P_VTOTAL5_H (H2_FRC_BASEADDRESS + 0x58)
#define H2_FRC_SF_P_VTOTAL6_L (H2_FRC_BASEADDRESS + 0x59)
#define H2_FRC_SF_P_VTOTAL6_H (H2_FRC_BASEADDRESS + 0x5A)
#define H2_FRC_SF_P_VTOTAL7_L (H2_FRC_BASEADDRESS + 0x5B)
#define H2_FRC_SF_P_VTOTAL7_H (H2_FRC_BASEADDRESS + 0x5C)

#define H3_LED_BASEADDRESS 0X3200
#define H3_LED_P0_COLOR_SEL (H3_LED_BASEADDRESS + 0x1C)
#define H3_LED_P1_COLOR_SEL (H3_LED_BASEADDRESS + 0x1D)
#define H3_LED_P2_COLOR_SEL (H3_LED_BASEADDRESS + 0x1E)
#define H3_LED_P3_COLOR_SEL (H3_LED_BASEADDRESS + 0x1F)
#define H3_LED_P4_COLOR_SEL (H3_LED_BASEADDRESS + 0x20)
#define H3_LED_P5_COLOR_SEL (H3_LED_BASEADDRESS + 0x21)
#define H3_LED_P6_COLOR_SEL (H3_LED_BASEADDRESS + 0x22)
#define H3_LED_P7_COLOR_SEL (H3_LED_BASEADDRESS + 0x23)
#define H3_LED_P8_COLOR_SEL (H3_LED_BASEADDRESS + 0x24)

#define H3_LED_P0_ON_ADD_L  (H3_LED_BASEADDRESS + 0x34)
#define H3_LED_P0_ON_ADD_H  (H3_LED_BASEADDRESS + 0x35)
#define H3_LED_P0_OFF_ADD_L (H3_LED_BASEADDRESS + 0x36)
#define H3_LED_P0_OFF_ADD_H (H3_LED_BASEADDRESS + 0x37)
#define H3_LED_P1_ON_ADD_L  (H3_LED_BASEADDRESS + 0x38)
#define H3_LED_P1_ON_ADD_H  (H3_LED_BASEADDRESS + 0x39)
#define H3_LED_P1_OFF_ADD_L (H3_LED_BASEADDRESS + 0x3A)
#define H3_LED_P1_OFF_ADD_H (H3_LED_BASEADDRESS + 0x3B)
#define H3_LED_P2_ON_ADD_L  (H3_LED_BASEADDRESS + 0x3C)
#define H3_LED_P2_ON_ADD_H  (H3_LED_BASEADDRESS + 0x3D)
#define H3_LED_P2_OFF_ADD_L (H3_LED_BASEADDRESS + 0x3E)
#define H3_LED_P2_OFF_ADD_H (H3_LED_BASEADDRESS + 0x3F)
#define H3_LED_P3_ON_ADD_L  (H3_LED_BASEADDRESS + 0x40)
#define H3_LED_P3_ON_ADD_H  (H3_LED_BASEADDRESS + 0x41)
#define H3_LED_P3_OFF_ADD_L (H3_LED_BASEADDRESS + 0x42)
#define H3_LED_P3_OFF_ADD_H (H3_LED_BASEADDRESS + 0x43)
#define H3_LED_P4_ON_ADD_L  (H3_LED_BASEADDRESS + 0x44)
#define H3_LED_P4_ON_ADD_H  (H3_LED_BASEADDRESS + 0x45)
#define H3_LED_P4_OFF_ADD_L (H3_LED_BASEADDRESS + 0x46)
#define H3_LED_P4_OFF_ADD_H (H3_LED_BASEADDRESS + 0x47)
#define H3_LED_P5_ON_ADD_L  (H3_LED_BASEADDRESS + 0x48)
#define H3_LED_P5_ON_ADD_H  (H3_LED_BASEADDRESS + 0x49)
#define H3_LED_P5_OFF_ADD_L (H3_LED_BASEADDRESS + 0x4A)
#define H3_LED_P5_OFF_ADD_H (H3_LED_BASEADDRESS + 0x4B)
#define H3_LED_P6_ON_ADD_L  (H3_LED_BASEADDRESS + 0x4C)
#define H3_LED_P6_ON_ADD_H  (H3_LED_BASEADDRESS + 0x4D)
#define H3_LED_P6_OFF_ADD_L (H3_LED_BASEADDRESS + 0x4E)
#define H3_LED_P6_OFF_ADD_H (H3_LED_BASEADDRESS + 0x4F)
#define H3_LED_P7_ON_ADD_L  (H3_LED_BASEADDRESS + 0x50)
#define H3_LED_P7_ON_ADD_H  (H3_LED_BASEADDRESS + 0x51)
#define H3_LED_P7_OFF_ADD_L (H3_LED_BASEADDRESS + 0x52)
#define H3_LED_P7_OFF_ADD_H (H3_LED_BASEADDRESS + 0x53)
#define H3_LED_P8_ON_ADD_L  (H3_LED_BASEADDRESS + 0x54)
#define H3_LED_P8_ON_ADD_H  (H3_LED_BASEADDRESS + 0x55)
#define H3_LED_P8_OFF_ADD_L (H3_LED_BASEADDRESS + 0x56)
#define H3_LED_P8_OFF_ADD_H (H3_LED_BASEADDRESS + 0x57)

typedef enum
{
  VIDEO_MUTE      = 0,
  VIDEO_UNMUTE    = !VIDEO_MUTE
}CXD3554VIDEO_ONOFF;


extern uint8_t ErrBuf0[10][16];
extern uint8_t ErrBuf1[10][16];
extern uint8_t ErrBuf2[10][16];
extern uint8_t FlagCheckLcosRegError[10];

void CXD3554_CKGBlock_Setting(void);
void CXD3554_MIPIBlock_settint(void);
void CXD3554_FRCAgentBlock_Setting(void);
void CXD3554_GMPORTBlock_Setting(void);
void CXD3554_PLLMem1Block_Setting(void);
void CXD3554_MINIDSIBlock_Setting(void);
void CXD3554_LPDDR2Block_Setting(void);
void CXD3554_DDR2Block_Setting(void);
void CXD3554_TXBlock_Setting(void);
void CXD3554_VIDEOBlock_Setting(void);


void Sxrd241_SC_Enable(void);
void Sxrd241_SC_Disable(void);
void Sxrd241_Video_Enable(void);
void CXD3554_FRCBlock_Start(void);


void CXD3554_TxBlock_Setting2(void);

void Sxrd241_Poweron(void);

void XD3554_FRCBlock_Start_Setting(void);
void Sxrd241_Register_setting(void);
void CXD3554_FRC_FrameLock(void);
void LcosInitSequence(void);


void LcosSetPatternSize(void);

void LcosSetIntPattern(void);
void LcosInit(void);
void Cxd3554Init(void);
void Sxrd241Init(void);
void Sxrd241_EfuseSetting(void);
void Cxd3554Init_Sequence2_Step1(void);
void Cxd3554Init_Sequence2_Step2(void);
void Cxd3554Init_Sequence2_Step3(void);
void Cxd3554Init_Sequence2_Step4(void);
void Cxd3554Init_Sequence4_Step1(void);
void Cxd3554Init_Sequence4_Step2(void);
void Cxd3554Init_Sequence1(void);
void Cxd3554Init_Sequence2(void);
void Cxd3554Init_Sequence3(void);
void Cxd3554Init_Sequence4(void);
void Cxd3554Init_Sequence5(void);
void LcosVideoMute(CXD3554VIDEO_ONOFF);
void LcosSetRGBWDutyMode(void);
void Lcos_Inp_ClearMipiTrnsErr(void);
void Lcos_CheckRegError(uint8_t local);
void SetDisplay_Size(void);
void LcosInitGamma(void);
void LcosSetRRGGBBGGMode(void);
#endif



	 


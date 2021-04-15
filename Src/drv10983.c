/******************************************************************************
  * @project: A100
  * @file: drv10983.c
  * @author: zyz
  * @company: ASU
  * @date: 2021.04.08
	*****************************************************************************/
#include "i2c.h"
#include "stdio.h"

#define DRV10983_I2C_ADDR 0xA4

#define SLAVE_I2C_ADDR DRV10983_I2C_ADDR
#define RETRY_CNT 5
//****************************************
// DRV10983
//****************************************
#define SpeedCtrl1 0x00
#define SpeedCtrl2 0x01
#define DevCtrl    0x02 //use to enable programming in the EEPROM  enProgKey = 1011 0110 (0xB6)
#define EECtrl     0x03 //bit7:sleepDis bit6:SIdata bit5:Copy EEPROM data to register bit4:Bit used to program (write) to the EEPROM

//Read only
#define Status  0x10
#define MotorSpeed1  0x11
#define MotorSpeed2  0x12
#define MotorPeriod1 0x13
#define MotorPeriod2 0x14
#define MotorKt1     0x15
#define MotorKt2     0x16
#define IPDPosition  0x19
#define SupplyVoltage 0x1A
#define SpeedCmd     0x1B
#define spdCmdBuffer 0x1C
#define FaultCode    0x1E

//EEPROM
/************************
MotorParam1 :bit7 0-Set driver output frequency to 25 kHz;1-50kHz
             bit6:0(Rm)-> Rm[6:4] : Number of the Shift bits of the motor phase resistance ??????
						              Rm[3:0] : Significant value of the motor phase resistance ?????? Rmdig = R_(ph_ct) / 0.00967  Rmdig = Rm[3:0]<<Rm[6:4]
MotorParam2 :bit7 Closed loop adjustment mode setting,0 = Full cycle adjustment,1 = Half cycle adjustment
             bit6:0(Kt)-> Kt[6:4] = Number of the Shift bits of BEMF constant ???????????
                          Kt[3:0] = Significant value of the BEMF constant						 
MotorParam3 :bit7 Motor commutate control advance,0-Fixed time???? 1-Variable time relative to the motor speed and VCC????????VCC?????
             bit6:0(Tdelay)?? [6:4] = Number of the Shift bits of LRTIME,[3:0] = Significant value of LRTIME,tSETTING = 2.5us*{TCtrlAdv[3:0] << TCtrlAdv[6:4]}
SysOpt1 :[7:6]ISD stationary judgment threshold ISD?????,00-6Hz(80 ms, no zero cross),01-3Hz (160 ms),10-1.6Hz (320ms),11-0.8Hz(640ms)
         [5:4]Advancing angle after inductive sense 00-30deg,01-60deg,10-90deg,11-120deg
         bit3 0-Initial speed detect (ISD) disable,1-ISD enable
				 bit2 0-Reverse drive disable,1-Reverse drive enable
				 [1:0]The threshold where device starts to process reverse drive (RvsDr) or brake.00-6.3Hz,01-13Hz,10-26Hz,11-51Hz
SysOpt2 :[7:6]Open loop current setting,00-0.2A,01-0.4A,10-0.8A,11-1.6A
         [5:3]Open-loop current ramp-up rate setting?????????? 000-6Vcc/s,001-3Vcc/s,010-1.5Vcc/s,011-0.7Vcc/s,100-0.34Vcc/s,101-0.16Vcc/s,110-0.07Vcc/s,111-0.023Vcc/s
         [2:0]Braking mode setting,000-No break,001-2.7s,010-1.3s,011-0.67s,100-0.33s,101-0.16s,110-0.08s,111-0.04s
SysOpt3 :[7:6]Control coefficient????,00-0.25,01-0.5,10-0.75,11-1
         [5:3]Open loop start-up accelerate (second order)???????(????),000-57Hz/s2,001-29 Hz/s2,010-14 Hz/s2,011-6.9 Hz/s2,100-3.3 Hz/s2,101-1.6 Hz/s2,110-0.66 Hz/s2,111-0.22 Hz/s2
         [2:0]Open loop start-up accelerate (first order)???????,000-76Hz/s2,001-38 Hz/s2,010-19 Hz/s2,011-9.2 Hz/s2,100-4.5 Hz/s2,101-2.1 Hz/s2,110-0.9 Hz/s2,111-0.3 Hz/s2
SysOpt4 :[7:3]Open to closed loop threshold ?????,0xxxx = Range 0:n*0.8Hz   00000=N/A,00001=0.8Hz,00111=5.6Hz,01111=12Hz;  1xxxx = Range 1: (n + 1)*12.8 Hz,10000 = 12.8 Hz,10001 = 25.6 Hz,10111 = 192 Hz,11111 = 204.8 Hz
         [2:0]Align time????, 000=5.3s,001=2.7s,010=1.3s,011=0.67s,100=0.33s,101=0.16s,110=0.08s,111=0.04s
SysOpt5 :bit7:No motor fault. Enabled when high
         bit6:Abnormal Kt. Enabled when high
				 bit5:Abnormal speed. Enabled when high
				 bit4:Lock detection current limit. Enabled when high
				 bit3:Inductive AVS enable. Enabled when high.
				 bit2:Mechanical AVS enable. Enabled when high
				 bit1:Mechanical AVS mode   0= AVS to VCC,1 = AVS to 24 V
				 bit0:IPD release mode      0 = Brake when inductive release,1 = Hi-z when inductive release
SysOpt6 :[7:4]Acceleration current limit threshold ??????,0000 = No acceleration current limit   0001 = 0.2A current limit,xxxx = n*0.2 A current limit
         [3:1]Lock detection current limit threshold????????, (n + 1)*0.4 A
SysOpt7 :bit7 Stuck in closed loop (no zero cross detected).??????? Enabled when high
				 [6:4]Closed loop accelerate ????,000 =Inf fast 001 = 48 VCC/s,010 = 48 VCC/s,011 = 0.77 VCC/s,100 = 0.37 VCC/s,101 = 0.19 VCC/s,110 = 0.091 VCC/s,111 = 0.045 VCC/s
         [3:0]Dead time between HS and LS gate drive for motor phases???? 0000 = 40 ns,xxxx = (n + 1)*40 ns  24V:400ns 12V:360ns
SysOpt8 :[7:4]IPD (inductive sense) current threshold ??????,0000 = No IPD function. 0001 = 0.4A,xxxx = 0.2 A*(n + 1) current threshold.
         bit3 Open loop stuck (no zero cross detected).?????? Enabled when high
				 bit2 Buck regulator voltage select ????????? 0:5V  1:3.3V
				 [1:0]Inductive sense clock??????  00=12Hz,01 = 24 Hz;10 = 47 Hz;11 = 95 Hz
SysOpt9 :[7:6]FG open loop output select ??????,00 = FG outputs in both open loop and closed loop. 01 = FG outputs only in closed loop.10 = FG outputs closed loop and the first open loop.
         [5:4]FG cycle select FG???? 00 = 1 pulse output per electrical cycle.01 = 2 pulses output per 3 electrical cycles.10 = 1 pulse output per 2 electrical cycles.11 = 1 pulse output per 3 electrical cycles
				 [3:2]Abnormal Kt lock detect threshold Kt???????? 00 = Kt_high = 3/2Kt. Kt_low = 3/4Kt;01 = Kt_high = 2Kt. Kt_low = 3/4Kt;10 = Kt_high = 3/2Kt. Kt_low = 1/2Kt;11 = Kt_high = 2Kt. Kt_low = 1/2Kt
				 bit1 Speed input mode??????  0->????  1->PWM
				 bit0 0-Transfer to closed loop at Op2ClsThr speed;1-No transfer to closed loop. Keep in open loop
************************/
#define MotorParam1  0x20 //
#define MotorParam2  0x21
#define MotorParam3  0x22
#define SysOpt1      0x23
#define SysOpt2      0x24
#define SysOpt3      0x25
#define SysOpt4      0x26
#define SysOpt5      0x27
#define SysOpt6      0x28
#define SysOpt7      0x29
#define SysOpt8      0x2A
#define SysOpt9      0x2B

static uint8_t ReadI2C_Byte(uint8_t RegAddr)
{
	uint8_t data;
	uint8_t ret,retry_cnt=RETRY_CNT;
		
	while(retry_cnt --) {	
		ret = HAL_I2C_Mem_Read(&hi2c2, (uint16_t)SLAVE_I2C_ADDR, (uint16_t)RegAddr, I2C_MEMADD_SIZE_8BIT, &data, 2, 300);
		if(ret == HAL_OK) break;
	}
	if(ret != HAL_OK) printf("ReadI2C_Byte 0x%x faild. ret=%d\r\n", RegAddr, ret);
	
	return data;
}

static uint8_t WriteI2C_reg(uint8_t RegAddr, uint8_t data)
{
		uint8_t ret, retry_cnt = RETRY_CNT;

		while(retry_cnt --) {
			ret = HAL_I2C_Mem_Write(&hi2c2, (uint16_t)SLAVE_I2C_ADDR, (uint16_t)RegAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, 300);
			if(ret == HAL_OK) break;
		}
		if(ret != HAL_OK) printf("WriteI2C_Byte 0x%x->0x%x  faild. ret=%d\r\n",RegAddr, data, ret);
	
		return ret;
}

void EEROM_Write_10983(uint8_t RegAddr,uint8_t Value)
{
		WriteI2C_reg(EECtrl, 0x40);
		WriteI2C_reg(RegAddr, Value);
		//WriteI2C_reg(DevCtrl, 0xB6);
		//WriteI2C_reg(EECtrl, 0x10);
}

uint8_t ReadReg_10983(uint8_t RegAddr)
{
		return ReadI2C_Byte(RegAddr);
}

//DRV10983 sleep en:1  DIS:0
void DRV10983_sleep(uint8_t i)
{	
  if(i)WriteI2C_reg(EECtrl, 0x00);
	else WriteI2C_reg(EECtrl, 0x80);
}

#define BASE_ADDR  0x20
uint8_t init_data[] =
{
	0xCC,
	0x6,
	0x2,
	0x0,
	0x58,
	0xD2,
	0xFC,
	0xBC,
	0x65,
	0xC8,
	0xD,
	0xC
};

void ThreePhaseMotorDriver_init(void)
{
	uint16_t i;
	
	for(i=0;i<sizeof(init_data)/sizeof(uint8_t);i++) {
		EEROM_Write_10983(BASE_ADDR + i, init_data[i]);	
		//printf("ThreePhaseMotorDriver_init write reg 0x%x->0x%x \r\n",BASE_ADDR+i, init_data[i]);			
	}
#if 0
	for(i=0; i<0x2c; i++) {
		uint16_t data = ReadI2C_Byte((uint8_t)i);
		printf("ThreePhaseMotorDriver_init read reg 0x%x->0x%x \r\n",i, data);
	} 
#endif
}

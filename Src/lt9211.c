/******************************************************************************
  * @project: LT9211
  * @file: lt9211.c
  * @author: zll
  * @company: LONTIUM COPYRIGHT and CONFIDENTIAL
  * @date: 2019.04.10
	*****************************************************************************/
#ifdef USE_LT9211_LVDS2MIPI
#include	"lt9211.h"
#include "stdio.h"

uint16_t hact, vact;
uint16_t hs, vs;
uint16_t hbp, vbp;
uint16_t htotal, vtotal;
uint16_t hfp, vfp;
uint8_t VideoFormat=0;
uint32_t lvds_clk_in = 0;
uint8_t Flag_LT9211_Pattern = 0;
enum VideoFormat Video_Format;
//hfp, hs, hbp,hact,htotal,vfp, vs, vbp, vact,vtotal,
/*
struct video_timing video_640x480_60Hz     ={ 8, 96,  40, 640,   800, 33,  2,  10, 480,   525,  25000};
struct video_timing video_720x480_60Hz     ={16, 62,  60, 720,   858,  9,  6,  30, 480,   525,  27000};
struct video_timing video_1280x720_60Hz    ={110,40, 220,1280,  1650,  5,  5,  20, 720,   750,  74250};
struct video_timing video_1280x720_30Hz    ={110,40, 220,1280,  1650,  5,  5,  20, 720,   750,  74250};
struct video_timing video_1366x768_60Hz    ={26, 110,110,1366,  1592,  13, 6,  13, 768,   800,  81000};
struct video_timing video_1920x1080_30Hz   ={88, 44, 148,1920,  2200,  4,  5,  36, 1080, 1125,  74250};
struct video_timing video_1920x1080_60Hz   ={88, 44, 148,1920,  2200,  4,  5,  36, 1080, 1125, 148500};
struct video_timing video_3840x1080_60Hz   ={176,88, 296,3840,  4400,  4,  5,  36, 1080, 1125, 297000};
struct video_timing video_1920x1200_60Hz   ={48, 32,  80,1920,  2080,  3,  6,  26, 1200, 1235, 154000};
struct video_timing video_3840x2160_30Hz   ={176,88, 296,3840,  4400,  8,  10, 72, 2160, 2250, 297000};
struct video_timing video_3840x2160_60Hz   ={176,88, 296,3840,  4400,  8,  10, 72, 2160, 2250, 594000};
*/
struct video_timing video ={88, 44, 148,1920,  2200,  4,  5,  36, 1080, 1125, 148500};
/*******************************************************************************************************************************************/
#define LT9211_I2C_ADDR 0x5a
#define RETRY_CNT 2
uint8_t HDMI_WriteI2C_Byte(uint8_t RegAddr, uint8_t data)
{
		uint8_t ret, retry_cnt = RETRY_CNT;
		
		while(retry_cnt --) {
			ret = HAL_I2C_Mem_Write(&hi2c2, (uint16_t)LT9211_I2C_ADDR, (uint16_t)RegAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&data, 1, 300);
			if(ret == HAL_OK) break;
		}
		if(ret != HAL_OK) printf("HDMI_WriteI2C_Byte 0x%x->0x%x  faild. ret=%d\r\n",RegAddr, data, ret);
	
		return ret;
}

uint8_t HDMI_ReadI2C_Byte(uint8_t RegAddr)
{
	uint8_t data = 0,ret,retry_cnt=RETRY_CNT;
		
	while(retry_cnt --) {	
		ret = HAL_I2C_Mem_Read(&hi2c2, (uint16_t)LT9211_I2C_ADDR, (uint16_t)RegAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&data, 1, 300);
		if(ret == HAL_OK) break;
	}
	if(ret != HAL_OK) printf("HDMI_ReadI2C_Byte 0x%x faild. ret=%d\r\n", RegAddr, ret);
	
	return data;
}

uint8_t LT9211_ChipID(void)
{
		uint8_t v1=0,v2=0,v3=0;
	
    HDMI_WriteI2C_Byte(0xff,0x81);//register bank
		v1 = HDMI_ReadI2C_Byte(0x00);
		v2 = HDMI_ReadI2C_Byte(0x01);
		v3 = HDMI_ReadI2C_Byte(0x02);
    printf("LT9211 Chip ID:0x%x,0x%x,0x%x \r\n",v1,v2,v3);
	
		return 1;
	
}

/** video chk soft rst **/
void lt9211_vid_chk_rst(void)       
{
    HDMI_WriteI2C_Byte(0xff,0x81);	  
    HDMI_WriteI2C_Byte(0x10,0xbe); 
    HAL_Delay(10);
    HDMI_WriteI2C_Byte(0x10,0xfe); 
}

/** lvds rx logic rst **/
void lt9211_lvdsrx_logic_rst(void)       
{
    HDMI_WriteI2C_Byte(0xff,0x81);	   
    HDMI_WriteI2C_Byte(0x0c,0xeb);
    HAL_Delay(10);
    HDMI_WriteI2C_Byte(0x0c,0xfb);
}

void LT9211_SystemInt(void)
{
    /* system clock init */		   
    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x01,0x18);
    
    HDMI_WriteI2C_Byte(0xff,0x86);
    HDMI_WriteI2C_Byte(0x06,0x61); 	
    HDMI_WriteI2C_Byte(0x07,0xa8); //fm for sys_clk
    
    HDMI_WriteI2C_Byte(0xff,0x87); //初始化 txpll 寄存器列表默认值给错了
    HDMI_WriteI2C_Byte(0x14,0x08); //default value
    HDMI_WriteI2C_Byte(0x15,0x00); //default value
    HDMI_WriteI2C_Byte(0x18,0x0f);
    HDMI_WriteI2C_Byte(0x22,0x08); //default value
    HDMI_WriteI2C_Byte(0x23,0x00); //default value
    HDMI_WriteI2C_Byte(0x26,0x0f); 
}

void LT9211_LvdsRxPhy(void)
{
    #ifdef INPUT_PORTA
    //printf("Port A PHY Config \r\n");
    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x02,0x8B);   //Port A LVDS mode enable
    HDMI_WriteI2C_Byte(0x05,0x21);   //port A CLK lane swap
    HDMI_WriteI2C_Byte(0x07,0x1f);   //port A clk enable
    HDMI_WriteI2C_Byte(0x04,0xa0);   //select port A clk as byteclk
    //HDMI_WriteI2C_Byte(0x09,0xFC); //port A P/N swap
        
    HDMI_WriteI2C_Byte(0xff,0x86);		
    HDMI_WriteI2C_Byte(0x33,0xe4);   //Port A Lane swap
    #endif
        
    #ifdef INPUT_PORTB
    //printf("Port B PHY Config \r\n");
    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x02,0x88);   //Port A/B LVDS mode enable
    HDMI_WriteI2C_Byte(0x05,0x21);   //port A CLK lane swap and rterm turn-off
    HDMI_WriteI2C_Byte(0x0d,0x21);   //port B CLK lane swap
    HDMI_WriteI2C_Byte(0x07,0x1f);   //port A clk enable  (只开Portb时,porta的lane0 clk要打开)
    HDMI_WriteI2C_Byte(0x0f,0x1f);   //port B clk enable
    //HDMI_WriteI2C_Byte(0x10,0x00);   //select port B clk as byteclk
    HDMI_WriteI2C_Byte(0x04,0xa1);   //reserve
    //HDMI_WriteI2C_Byte(0x11,0x01);   //port B P/N swap
    HDMI_WriteI2C_Byte(0x10,0xfc);
    
    HDMI_WriteI2C_Byte(0xff,0x86);		
    HDMI_WriteI2C_Byte(0x34,0xe4);   //Port B Lane swap
    
    HDMI_WriteI2C_Byte(0xff,0xd8);		
    HDMI_WriteI2C_Byte(0x16,0x80);
    #endif
    
    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x20,0x7f); 	
    HDMI_WriteI2C_Byte(0x20,0xff);  //mlrx calib reset
}

void LT9211_LvdsRxDigital(void)
{	  
    HDMI_WriteI2C_Byte(0xff,0x85);
    HDMI_WriteI2C_Byte(0x88,0x10);      //LVDS input, MIPI output
    
    HDMI_WriteI2C_Byte(0xff,0xd8);

    if(INPUT_PORT_NUM == 1)             //1Port LVDS Input
    {
        HDMI_WriteI2C_Byte(0x10,0x80);
        printf("LVDS Port Num: 1 \r\n");
    }
    else if(INPUT_PORT_NUM == 2)        //2Port LVDS Input
    {
        HDMI_WriteI2C_Byte(0x10,0x00);
        printf("LVDS Port Num: 2 \r\n");
    }
    else
    {
        printf("Port Num Set Error \r\n");
    } 	

    lt9211_vid_chk_rst();              //video chk soft rst
    lt9211_lvdsrx_logic_rst();         //lvds rx logic rst		
	
    HDMI_WriteI2C_Byte(0xff,0x86);
    HDMI_WriteI2C_Byte(0x30,0x45);     //port AB input port sel	 
	
    if(LVDS_FORMAT == JEDIA_FORMAT)
    {
        HDMI_WriteI2C_Byte(0xff,0x85);
        HDMI_WriteI2C_Byte(0x59,0xd0); 	
        HDMI_WriteI2C_Byte(0xff,0xd8);
        HDMI_WriteI2C_Byte(0x11,0x40);
    }
}

int lt9211_lvds_clkstb_check(void)
{
    uint8_t porta_clk_state = 0;
    uint8_t portb_clk_state = 0;

    HDMI_WriteI2C_Byte(0xff,0x86);
    HDMI_WriteI2C_Byte(0x00,0x01);
    HAL_Delay(200);
    porta_clk_state = (HDMI_ReadI2C_Byte(0x08) & (0x20));
    
    HDMI_WriteI2C_Byte(0xff,0x86);
    HDMI_WriteI2C_Byte(0x00,0x02);
    HAL_Delay(200);
    portb_clk_state = (HDMI_ReadI2C_Byte(0x08) & (0x20));
#if 0   
    if(INPUT_PORT_NUM == 1)
    {
        #ifdef INPUT_PORTA
        if( porta_clk_state )
        {
            return 1;
        }
        else
        {
            return 0;
        }
        #endif
				
        #ifdef INPUT_PORTB
        if( portb_clk_state )
        {
            return 1;
        }
        else
        {
            return 0;
        }
        #endif
    }
    else if(INPUT_PORT_NUM == 2)
#endif			
    {
        if(porta_clk_state && portb_clk_state)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
}

void LT9211_ClockCheckDebug(void)
{
#ifdef _uart_debug_
    uint32_t fm_value;

    lvds_clk_in = 0;
    #ifdef INPUT_PORTA
    HDMI_WriteI2C_Byte(0xff,0x86);
    HDMI_WriteI2C_Byte(0x00,0x01);
    HAL_Delay(50);
    fm_value = 0;
    fm_value = (HDMI_ReadI2C_Byte(0x08) &(0x0f));
    fm_value = (fm_value<<8) ;
    fm_value = fm_value + HDMI_ReadI2C_Byte(0x09);
    fm_value = (fm_value<<8) ;
    fm_value = fm_value + HDMI_ReadI2C_Byte(0x0a);
    printf("Port A lvds clock: %d \r\n", fm_value);

    lvds_clk_in = fm_value;
    #endif
	
    #ifdef INPUT_PORTB
    HDMI_WriteI2C_Byte(0xff,0x86);
    HDMI_WriteI2C_Byte(0x00,0x02);
    HAL_Delay(50);
    fm_value = 0;
    fm_value = (HDMI_ReadI2C_Byte(0x08) &(0x0f));
    fm_value = (fm_value<<8) ;
    fm_value = fm_value + HDMI_ReadI2C_Byte(0x09);
    fm_value = (fm_value<<8) ;
    fm_value = fm_value + HDMI_ReadI2C_Byte(0x0a);
    printf("Port B lvds clock: %d \r\n",fm_value);

    lvds_clk_in = fm_value;
    #endif

#endif
}

void LT9211_LvdsRxPll(void)
{
    uint8_t loopx = 0;
    
    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x25,0x05);
    HDMI_WriteI2C_Byte(0x27,0x02);	
	
    if(INPUT_PORT_NUM == 1)             //1Port LVDS Input
    {
        HDMI_WriteI2C_Byte(0x24,0x24); //RXPLL_LVDSCLK_MUXSEL,PIXCLK_MUXSEL	0x2c.
        HDMI_WriteI2C_Byte(0x28,0x44); //0x64
    }
    else if(INPUT_PORT_NUM == 2)        //2Port LVDS Input
    {
        HDMI_WriteI2C_Byte(0x24,0x2c); //RXPLL_LVDSCLK_MUXSEL,PIXCLK_MUXSEL	0x2c.
        HDMI_WriteI2C_Byte(0x28,0x64); //0x64
    }
    else
    {
        printf("LvdsRxPll: lvds port count error \r\n");
    }
    HAL_Delay(10);
    HDMI_WriteI2C_Byte(0xff,0x87);
    HDMI_WriteI2C_Byte(0x05,0x00);
    HDMI_WriteI2C_Byte(0x05,0x80);
    HAL_Delay(100);
    for(loopx = 0; loopx < 10; loopx++) //Check Rx PLL cal
    {
        HDMI_WriteI2C_Byte(0xff,0x87);			
        if(HDMI_ReadI2C_Byte(0x12)& 0x80)
        {
            if(HDMI_ReadI2C_Byte(0x11)& 0x80)
            {
                printf("LT9211 rx pll lock cal done \r\n");
            }
            else
            {
                printf("LT9211 rx pll lock cal undone!! \r\n");
            }					
            //printf("LT9211 rx pll lock \r\n");
            break;
        }
        else
        {
            printf("LT9211 rx pll unlocked \r\n");
        }
    }
}

void LT9211_CSC(void)
{
    if(LT9211_OutputMode == MIPI_CSI)
    {
        HDMI_WriteI2C_Byte(0xff,0xf9);
        HDMI_WriteI2C_Byte(0x86,0x0f);   //03/00
        HDMI_WriteI2C_Byte(0x87,0x30);   //4f/4c
    }
}

void LT9211_VideoGet(void)
{
    uint8_t sync_polarity = 0;

    HDMI_WriteI2C_Byte(0xff,0x86);
    HDMI_WriteI2C_Byte(0x20,0x00);
	
    sync_polarity = HDMI_ReadI2C_Byte(0x70);
    vs = HDMI_ReadI2C_Byte(0x71);

    hs = HDMI_ReadI2C_Byte(0x72);
    hs = (hs<<8) + HDMI_ReadI2C_Byte(0x73);
	
    vbp = HDMI_ReadI2C_Byte(0x74);
    vfp = HDMI_ReadI2C_Byte(0x75);

    hbp = HDMI_ReadI2C_Byte(0x76);
    hbp = (hbp<<8) + HDMI_ReadI2C_Byte(0x77);

    hfp = HDMI_ReadI2C_Byte(0x78);
    hfp = (hfp<<8) + HDMI_ReadI2C_Byte(0x79);

    vtotal = HDMI_ReadI2C_Byte(0x7A);
    vtotal = (vtotal<<8) + HDMI_ReadI2C_Byte(0x7B);

    htotal = HDMI_ReadI2C_Byte(0x7C);
    htotal = (htotal<<8) + HDMI_ReadI2C_Byte(0x7D);

    vact = HDMI_ReadI2C_Byte(0x7E);
    vact = (vact<<8)+ HDMI_ReadI2C_Byte(0x7F);

    hact = HDMI_ReadI2C_Byte(0x80);
    hact = (hact<<8) + HDMI_ReadI2C_Byte(0x81);

  printf("sync_polarity = %x \r\n", sync_polarity);
  printf("\r\nhfp, hs, hbp, hact, htotal = %d %d %d %d %d \r\n",hfp,hs,hbp,hact,htotal);
	printf("\r\nvfp, vs, vbp, vact, vtotal = %d %d %d %d %d \r\n", vfp,vs,vbp,vact,vtotal);
}

void LT9211_VideoCheck(void)
{
    uint8_t sync_polarity = 0;

    HDMI_WriteI2C_Byte(0xff,0x86);
    HDMI_WriteI2C_Byte(0x20,0x00);
	
    sync_polarity = HDMI_ReadI2C_Byte(0x70);
    vs = HDMI_ReadI2C_Byte(0x71);

    hs = HDMI_ReadI2C_Byte(0x72);
    hs = (hs<<8) + HDMI_ReadI2C_Byte(0x73);
	
    vbp = HDMI_ReadI2C_Byte(0x74);
    vfp = HDMI_ReadI2C_Byte(0x75);

    hbp = HDMI_ReadI2C_Byte(0x76);
    hbp = (hbp<<8) + HDMI_ReadI2C_Byte(0x77);

    hfp = HDMI_ReadI2C_Byte(0x78);
    hfp = (hfp<<8) + HDMI_ReadI2C_Byte(0x79);

    vtotal = HDMI_ReadI2C_Byte(0x7A);
    vtotal = (vtotal<<8) + HDMI_ReadI2C_Byte(0x7B);

    htotal = HDMI_ReadI2C_Byte(0x7C);
    htotal = (htotal<<8) + HDMI_ReadI2C_Byte(0x7D);

    vact = HDMI_ReadI2C_Byte(0x7E);
    vact = (vact<<8)+ HDMI_ReadI2C_Byte(0x7F);

    hact = HDMI_ReadI2C_Byte(0x80);
    hact = (hact<<8) + HDMI_ReadI2C_Byte(0x81);

    printf("sync_polarity = %x \r\n", sync_polarity);
	if(!(sync_polarity & 0x01)) //hsync
	{
        HDMI_WriteI2C_Byte(0xff,0xd8);
        HDMI_WriteI2C_Byte(0x10, (HDMI_ReadI2C_Byte(0x10)| 0x10));
	}
	if(!(sync_polarity & 0x02)) //vsync
	{
        HDMI_WriteI2C_Byte(0xff,0xd8);
        HDMI_WriteI2C_Byte(0x10, (HDMI_ReadI2C_Byte(0x10)| 0x20));
	}

  printf("\r\nhfp, hs, hbp, hact, htotal = %d %d %d %d %d \r\n",hfp,hs,hbp,hact,htotal);
	printf("\r\nvfp, vs, vbp, vact, vtotal = %d %d %d %d %d \r\n", vfp,vs,vbp,vact,vtotal);
#if 1
	hfp = 84;
	hs = 44;
	hbp = 152;
	hact = 1920;
	htotal = 2200;
	vfp = 3;
	vs = 5;
	hbp = 37;
	vact = 1080;
	vtotal = 1125;
#endif
}

void LT9211_MipiTxpll(void)
{
    uint8_t loopx;
	
    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x36,0x03); //b7:txpll_pd
    HDMI_WriteI2C_Byte(0x37,0x28);
    HDMI_WriteI2C_Byte(0x38,0x44);
    HDMI_WriteI2C_Byte(0x3a,0x93);
//		HDMI_WriteI2C_Byte(0x3b,0x44);	
	
    HDMI_WriteI2C_Byte(0xff,0x87);
    HDMI_WriteI2C_Byte(0x13,0x00);
    HDMI_WriteI2C_Byte(0x13,0x80);
    HAL_Delay(100);
    for(loopx = 0; loopx < 10; loopx++) //Check Tx PLL cal done
    {
			
        HDMI_WriteI2C_Byte(0xff,0x87);			
        if(HDMI_ReadI2C_Byte(0x1f)& 0x80)
        {
            if(HDMI_ReadI2C_Byte(0x20)& 0x80)
            {
                printf("LT9211 tx pll lock \r\n");
            }
            else
            {
                printf("LT9211 tx pll unlocked \r\n");
            }					
            //printf("LT9211 tx pll cal done \r\n");
            break;
        }
        else
        {
            printf("LT9211 tx pll unlocked \r\n");
        }
    }	 		
}


void LT9211_MipiTxPhy(void)
{		
	//printf("\r\nLT9211 tx phy \r\n");
    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x62,0x00); //ttl output disable
    HDMI_WriteI2C_Byte(0x3b,0x32); //mipi en
	
//    HDMI_WriteI2C_Byte(0x48,0x5f); //Port A Lane P/N Swap
//    HDMI_WriteI2C_Byte(0x49,0x92); 
//    HDMI_WriteI2C_Byte(0x52,0x5f); //Port B Lane P/N Swap
//    HDMI_WriteI2C_Byte(0x53,0x92); 
	
    HDMI_WriteI2C_Byte(0xff,0x86);	
    HDMI_WriteI2C_Byte(0x40,0x80); //tx_src_sel
    /*port src sel*/
#ifdef MIPI_DATA_LANE_SWAP
    HDMI_WriteI2C_Byte(0x41,0x32);	//clock no swap
    HDMI_WriteI2C_Byte(0x42,0x10);
    HDMI_WriteI2C_Byte(0x43,0x43); //Port A MIPI Lane Swap
    HDMI_WriteI2C_Byte(0x44,0x21);
    HDMI_WriteI2C_Byte(0x45,0x04); //Port B MIPI Lane Swap
#else
    HDMI_WriteI2C_Byte(0x41,0x01);	
    HDMI_WriteI2C_Byte(0x42,0x23);
    HDMI_WriteI2C_Byte(0x43,0x40); //Port A MIPI Lane Swap
    HDMI_WriteI2C_Byte(0x44,0x12);
    HDMI_WriteI2C_Byte(0x45,0x34); //Port B MIPI Lane Swap
#endif
}

void LT9211_MipiTxDigital(void)
{	
	//printf("\r\nLT9211 tx digital \r\n");
    HDMI_WriteI2C_Byte(0xff,0xd4);
		HDMI_WriteI2C_Byte(0x1c,0x30);  //hs_rqst_pre
		HDMI_WriteI2C_Byte(0x1d,0x0a);  //lpx
		HDMI_WriteI2C_Byte(0x1e,0x06);  //prpr
		HDMI_WriteI2C_Byte(0x1f,0x0a);  //trail
		HDMI_WriteI2C_Byte(0x21,0x00);  //[5]byte_swap,[0]burst_clk

    HDMI_WriteI2C_Byte(0xff,0xd4);	
    HDMI_WriteI2C_Byte(0x16,0x55);	
    HDMI_WriteI2C_Byte(0x10,0x01);
    HDMI_WriteI2C_Byte(0x11,0x50); //read byteclk ??,???
    HDMI_WriteI2C_Byte(0x13,0x0f);	//bit[5:4]:lane num, bit[2]:bllp,bit[1:0]:vid_mode
    HDMI_WriteI2C_Byte(0x14,0x20); //bit[5:4]:data typ,bit[2:0]:fmt sel 000:rgb888
    HDMI_WriteI2C_Byte(0x21,0x00);
}

#if 1
void LT9211_SetTxTiming2(void)
{
    HDMI_WriteI2C_Byte(0xff,0xd4);
    HDMI_WriteI2C_Byte(0x04,0x08); //hs[7:0] not care
    HDMI_WriteI2C_Byte(0x05,0x08); //hbp[7:0] not care
    HDMI_WriteI2C_Byte(0x06,0x08); //hfp[7:0] not care
    HDMI_WriteI2C_Byte(0x07,(uint8_t)(hact>>8)); //hactive[15:8]
    HDMI_WriteI2C_Byte(0x08,(uint8_t)(hact)); //hactive[7:0]
    
    HDMI_WriteI2C_Byte(0x09,(uint8_t)(vs)); //vfp[7:0]
    HDMI_WriteI2C_Byte(0x0a,0x00); //bit[3:0]:vbp[11:8]
    HDMI_WriteI2C_Byte(0x0b,(uint8_t)(vbp)); //vbp[7:0]
    HDMI_WriteI2C_Byte(0x0c,(uint8_t)(vact>>8)); //vcat[15:8]
    HDMI_WriteI2C_Byte(0x0d,(uint8_t)(vact)); //vcat[7:0]
    HDMI_WriteI2C_Byte(0x0e,(uint8_t)(vfp>>8)); //vfp[11:8]
    HDMI_WriteI2C_Byte(0x0f,(uint8_t)(vfp)); //vfp[7:0]
}
#else
void LT9211_SetTxTiming2(void)
{
		HDMI_WriteI2C_Byte(0xff,0xd4);
		HDMI_WriteI2C_Byte(0x04,hs); //hs[7:0] not care
		HDMI_WriteI2C_Byte(0x05,hbp); //hbp[7:0] not care
		HDMI_WriteI2C_Byte(0x06,hfp); //hfp[7:0] not care
		HDMI_WriteI2C_Byte(0x07,(uint8_t)(hact>>8)); //hactive[15:8]
		HDMI_WriteI2C_Byte(0x08,(uint8_t)(hact)); //hactive[7:0]
		
		HDMI_WriteI2C_Byte(0x09,(uint8_t)(vs)); //vfp[7:0]
		HDMI_WriteI2C_Byte(0x0a,(uint8_t)(vbp>>8)); //bit[3:0]:vbp[11:8]
		HDMI_WriteI2C_Byte(0x0b,(uint8_t)(vbp)); //vbp[7:0]
		HDMI_WriteI2C_Byte(0x0c,(uint8_t)(vact>>8)); //vcat[15:8]
		HDMI_WriteI2C_Byte(0x0d,(uint8_t)(vact)); //vcat[7:0]
		HDMI_WriteI2C_Byte(0x0e,(uint8_t)(vfp>>8)); //vfp[11:8]
		HDMI_WriteI2C_Byte(0x0f,(uint8_t)(vfp)); //vfp[7:0]
}
#endif

void LT9211_SetTxTiming(struct video_timing *video_format)
{
	//uint8_t sync_polarity;
	uint16_t hact, vact;
  uint16_t hs, vs;
  uint16_t hbp, vbp;
  //uint16_t htotal, vtotal;
  uint16_t hfp, vfp;
  //struct Timing TimingStr;		

	vs = video_format->vs;
    hs = video_format->hs;
	vbp = video_format->vbp;
    vfp = video_format->vfp;
	hbp = video_format->hbp;
	hfp = video_format->hfp;
	//vtotal = video_format->vtotal;
	//htotal = video_format->htotal;
	vact = video_format->vact;
	hact = video_format->hact;
	
	//TimingStr.hact = hact;
	//TimingStr.vact = vact;
	//TimingStr.vs = vs;
	//TimingStr.vbp = vbp;
	//TimingStr.vfp = vfp;
	
	HDMI_WriteI2C_Byte(0xff,0xd4);
	//HDMI_WriteI2C_Byte(0x04,0x08); //hs[7:0] not care
	//HDMI_WriteI2C_Byte(0x05,0x08); //hbp[7:0] not care
	//HDMI_WriteI2C_Byte(0x06,0x08); //hfp[7:0] not care
	HDMI_WriteI2C_Byte(0x04,hs); //hs[7:0] not care
	HDMI_WriteI2C_Byte(0x05,hbp); //hbp[7:0] not care
	HDMI_WriteI2C_Byte(0x06,hfp); //hfp[7:0] not care
	HDMI_WriteI2C_Byte(0x07,(uint8_t)(hact>>8)); //hactive[15:8]
	HDMI_WriteI2C_Byte(0x08,(uint8_t)(hact)); //hactive[7:0]
	
	HDMI_WriteI2C_Byte(0x09,(uint8_t)(vs)); //vfp[7:0]
	HDMI_WriteI2C_Byte(0x0a,(uint8_t)(vbp>>8)); //bit[3:0]:vbp[11:8]
	HDMI_WriteI2C_Byte(0x0b,(uint8_t)(vbp)); //vbp[7:0]
	HDMI_WriteI2C_Byte(0x0c,(uint8_t)(vact>>8)); //vcat[15:8]
	HDMI_WriteI2C_Byte(0x0d,(uint8_t)(vact)); //vcat[7:0]
	HDMI_WriteI2C_Byte(0x0e,(uint8_t)(vfp>>8)); //vfp[11:8]
	HDMI_WriteI2C_Byte(0x0f,(uint8_t)(vfp)); //vfp[7:0]
}

void LT9211_Config(void)
{
    //LT9211_ChipID();
    LT9211_SystemInt();
    LT9211_LvdsRxPhy();
    LT9211_LvdsRxDigital();
    LT9211_LvdsRxPll();
}


/******************************************************************
*Founction: 测试 Pattern	输出。
*根据结构体Video 中的timing，设置Video 和 pixel clk。
*******************************************************************/
void LT9211_Patten(struct video_timing *video_format)
{
    uint32_t pclk_khz;
    uint8_t dessc_pll_post_div;
    uint32_t pcr_m, pcr_k;

    pclk_khz = video_format->pclk_khz;     

    HDMI_WriteI2C_Byte(0xff,0xf9);
	HDMI_WriteI2C_Byte(0x3e,0x80);  

    HDMI_WriteI2C_Byte(0xff,0x85);
	HDMI_WriteI2C_Byte(0x88,0xd0);  

    HDMI_WriteI2C_Byte(0xa1,0x77); 
    HDMI_WriteI2C_Byte(0xa2,0xff); 

	HDMI_WriteI2C_Byte(0xa3,(uint8_t)((video_format->hs+video_format->hbp)/256));
	HDMI_WriteI2C_Byte(0xa4,(uint8_t)((video_format->hs+video_format->hbp)%256));//h_start

	HDMI_WriteI2C_Byte(0xa5,(uint8_t)((video_format->vs+video_format->vbp)%256));//v_start

   	HDMI_WriteI2C_Byte(0xa6,(uint8_t)(video_format->hact/256));
	HDMI_WriteI2C_Byte(0xa7,(uint8_t)(video_format->hact%256)); //hactive

	HDMI_WriteI2C_Byte(0xa8,(uint8_t)(video_format->vact/256));
	HDMI_WriteI2C_Byte(0xa9,(uint8_t)(video_format->vact%256));  //vactive

   	HDMI_WriteI2C_Byte(0xaa,(uint8_t)(video_format->htotal/256));
	HDMI_WriteI2C_Byte(0xab,(uint8_t)(video_format->htotal%256));//htotal

   	HDMI_WriteI2C_Byte(0xac,(uint8_t)(video_format->vtotal/256));
	HDMI_WriteI2C_Byte(0xad,(uint8_t)(video_format->vtotal%256));//vtotal

   	HDMI_WriteI2C_Byte(0xae,(uint8_t)(video_format->hs/256)); 
	HDMI_WriteI2C_Byte(0xaf,(uint8_t)(video_format->hs%256));   //hsa

    HDMI_WriteI2C_Byte(0xb0,(uint8_t)(video_format->vs%256));    //vsa

    //dessc pll to generate pixel clk
	HDMI_WriteI2C_Byte(0xff,0x82); //dessc pll
	HDMI_WriteI2C_Byte(0x2d,0x48); //pll ref select xtal 

	if(pclk_khz < 44000)
	{
        HDMI_WriteI2C_Byte(0x35,0x83);
		dessc_pll_post_div = 16;
	}

	else if(pclk_khz < 88000)
	{
	  	HDMI_WriteI2C_Byte(0x35,0x82);
		dessc_pll_post_div = 8;
	}

	else if(pclk_khz < 176000)
	{
	  	HDMI_WriteI2C_Byte(0x35,0x81);
		dessc_pll_post_div = 4;
	}

	else if(pclk_khz < 352000)
	{
	  	HDMI_WriteI2C_Byte(0x35,0x80);
		dessc_pll_post_div = 0;
	}

	pcr_m = (pclk_khz * dessc_pll_post_div) /25;
	pcr_k = pcr_m%1000;
	pcr_m = pcr_m/1000;

	pcr_k <<= 14; 

	//pixel clk
 	HDMI_WriteI2C_Byte(0xff,0xd0); //pcr
	HDMI_WriteI2C_Byte(0x2d,0x7f);
	HDMI_WriteI2C_Byte(0x31,0x00);

	HDMI_WriteI2C_Byte(0x26,0x80|((uint8_t)pcr_m));
	HDMI_WriteI2C_Byte(0x27,(uint8_t)((pcr_k>>16)&0xff)); //K
	HDMI_WriteI2C_Byte(0x28,(uint8_t)((pcr_k>>8)&0xff)); //K
	HDMI_WriteI2C_Byte(0x29,(uint8_t)(pcr_k&0xff)); //K
}

void LT9211_Pattern_Config(void)
{
    if(!LT9211_ChipID())		
			return;
		
    LT9211_SystemInt();
		LT9211_Patten(&video);
		LT9211_MipiTxpll();	
		LT9211_MipiTxPhy();
		LT9211_SetTxTiming(&video);
    //InitPanel();
		LT9211_MipiTxDigital();
    LT9211_ClockCheckDebug();
}

 void LT9211_Reset(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_Delay(50);		
}

void LT9211_Pattern_Init(void)
{
	LT9211_Reset();
	LT9211_Pattern_Config();
	Flag_LT9211_Pattern = 1;
	printf("LT9211_Pattern_Init \r\n");
}

int lt9211_get_lvds_clkstb(uint8_t* count)
{
    uint8_t porta_clk_state = 0;
    uint8_t portb_clk_state = 0;
		uint8_t cnt = *count;
		uint8_t ret;
	
		while(cnt--)
		{
				ret = HDMI_WriteI2C_Byte(0xff,0x86);
				if(ret != HAL_OK) {
					printf("lt9211_get_lvds_clkstb: I2c transfer error. \r\n");
					return 1;
				}
				
				HDMI_WriteI2C_Byte(0x00,0x01);
				HAL_Delay(1);
				porta_clk_state = (HDMI_ReadI2C_Byte(0x08) & (0x20));
			
				HDMI_WriteI2C_Byte(0xff,0x86);
				HDMI_WriteI2C_Byte(0x00,0x02);
				HAL_Delay(1);			
				portb_clk_state = (HDMI_ReadI2C_Byte(0x08) & (0x20));
				
				if(porta_clk_state && portb_clk_state)
				{
						*count = cnt;
						return 1;
				}
    }

		*count = 0;
    return 0;
}

void LT9211_Video_Reset(void)
{
			printf("\r\n LT9211_Video_Reset....... \r\n");
			LT9211_ClockCheckDebug();
			LT9211_LvdsRxPll();
			lt9211_vid_chk_rst();              //video chk soft rst
			lt9211_lvdsrx_logic_rst();
			HAL_Delay(200);
			LT9211_VideoCheck();

			//mipi tx set
			LT9211_MipiTxpll();					
			LT9211_MipiTxPhy();
			LT9211_SetTxTiming2();
			//InitPanel();
			LT9211_MipiTxDigital();		
}

void LT9211_Init(void)
{
		uint8_t count = 50;
		
		LT9211_Reset();
	
	  if(!LT9211_ChipID()) return;
	
		Flag_LT9211_Pattern = 0;
	
		LT9211_Config();

		if(lt9211_get_lvds_clkstb(&count))
		{
			printf("\r\n LT9211_Init: lvds clk stable count=%d\r\n", count);
			LT9211_Video_Reset();
		} else {
			printf("\r\n LT9211_Init: lvds clk not stable \r\n");
		}
}

uint8_t get_LT9211_Mode(void)
{
		return Flag_LT9211_Pattern;
}

#if 0
void LT9211_Pattern_Init2(void)
{
  static int flag_enter = 1;
	
	if(flag_enter == 1) {	
		LT9211_Reset();
		LT9211_Pattern_Config();
		flag_enter = 0;
		printf("LT9211_Pattern_Init \r\n");
	}
}


void LT9211_MainLoop(void)
{
    static int flag_lvds_chg = 1;
    
    if( lt9211_lvds_clkstb_check() )
    {
        if( flag_lvds_chg )
        {
            printf("\r\n lvds clk stable \r\n");
            LT9211_ClockCheckDebug();
            LT9211_LvdsRxPll();
            lt9211_vid_chk_rst();              //video chk soft rst
            lt9211_lvdsrx_logic_rst();
            HAL_Delay(200);
            LT9211_VideoCheck();

            //mipi tx set
            LT9211_MipiTxpll();					
            LT9211_MipiTxPhy();
						LT9211_SetTxTiming2();
            //InitPanel();
            LT9211_MipiTxDigital();
            
            flag_lvds_chg = 0;
        }
    }
    else
    {
        if( !flag_lvds_chg )
        {
            printf("\r\n lvds clk not stable \r\n");
            flag_lvds_chg = 1;
        }
    }
}

void LT9211_Init2(void)
{
  static int flag_enter = 1;
	
	if(flag_enter == 1) {
		LT9211_Reset();
		flag_enter = 0;
		LT9211_Config();
		printf("LT9211_Init \r\n");		
	}
	
	LT9211_MainLoop();
}
#endif

#endif

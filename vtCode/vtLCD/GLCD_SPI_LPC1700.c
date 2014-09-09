/******************************************************************************/
/* GLCD_SPI_LPC1700.c: LPC1700 low level Graphic LCD (240x320 pixels) driven  */
/*                     with SPI functions                                     */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2010 Keil Software. All rights reserved.                */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/

/*
Karl
2014-06-20
MDK-ARM 4.21 includes the file "GLCD_SPI_LPC1700.c", which works for LCD
driver "SPFD5408B".
It was then modified to work with FreeRTOS, and was then included in the
Virtual Machine distributed to students

In summer 2014, a new batch of boards were received that use a different
LCD driver, "HX8347-D".
The currently latest version of MDK-ARM, 4.74, has a different version of
"GLCD_SPI_LPC1700.c" which has support for both LCD drivers.
So I merged the 4.74 version with our FreeRTOS modified 4.21 version to end
up with this file, which should support both drivers.

Note that the "HX8347-D" LCDs have jumper resistors soldered on which
match the positioning of the old driver LCDs. On the old boards, the
jumpers configured it for 4-wire SPI. However, on the new LCDs the
same jumper configuration causes it to be in 3-wire SPI. Writing to
the LCD can be done using the SSP module in exactly the same way as before,
but reading from the LCD cannot be done without bit banging. So, this driver
file does not support the Get functions on the new LCDs.
*/

#include <lpc17xx.h>
#include "GLCD.h"
#include "Font_6x8_h.h"
#include "Font_16x24_h.h"
#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"
#include "semphr.h"

// SPI (and supporting) include files from NXP
#include "lpc17xx_ssp.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_clkpwr.h"

// Include the VT SPI interrupt code
#include "vtSSP.h"

#include "vtUtilities.h"

/************************** Orientation  configuration ************************/

//#define HORIZONTAL  1                   /* If vertical = 0, if horizontal = 1 */

//Karl
//I don't guarantee these will work with all functions
//especially for the new LCD driver
#define LANDSCAPE   1                   /* 1 for landscape, 0 for portrait    */
#define ROTATE180   0                   /* 1 to rotate the screen for 180 deg */

/*********************** Hardware specific configuration **********************/

/* SPI Interface: SPI3
   
   PINS: 
   - CS     = P0.6 (GPIO pin)
   - RS     = GND
   - WR/SCK = P0.7 (SCK1)
   - RD     = GND
   - SDO    = P0.8 (MISO1)
   - SDI    = P0.9 (MOSI1)                                                    */

#define PIN_CS      (1 << 6)

//Karl
#define PIN_CLK     (1 << 7)
#define PIN_DAT     (1 << 9)

//Karl
#define IN          0x00
#define OUT         0x01

/* SPI_SR - bit definitions                                                   */
#define TFE         0x01
#define RNE         0x04
#define BSY         0x10

/*------------------------- Speed dependant settings -------------------------*/

/* If processor works on high frequency delay has to be increased, it can be 
   increased by factor 2^N by this constant                                   */
// MTJ removed and replaced with a real RTOS delay #define DELAY_2N    18

/*---------------------- Graphic LCD size definitions ------------------------*/

#if (LANDSCAPE == 1)
#define WIDTH       320                 /* Screen Width (in pixels)           */
#define HEIGHT      240                 /* Screen Hight (in pixels)           */
#else
#define WIDTH       240                 /* Screen Width (in pixels)           */
#define HEIGHT      320                 /* Screen Hight (in pixels)           */
#endif
#define BPP         16                  /* Bits per pixel                     */
#define BYPP        ((BPP+7)/8)         /* Bytes per pixel                    */

/*--------------- Graphic LCD interface hardware definitions -----------------*/

/* Pin CS setting to 0 or 1                                                   */
//MTJ change 
//#define LCD_CS(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_CS) : (LPC_GPIO0->FIOCLR = PIN_CS));
//#define LCD_CS(x)

//Karl
#define LCD_CLK(x)  ((x) ? (LPC_GPIO0->FIOSET = PIN_CLK)   : (LPC_GPIO0->FIOCLR = PIN_CLK))
#define LCD_DAT(x)  ((x) ? (LPC_GPIO0->FIOSET = PIN_DAT)   : (LPC_GPIO0->FIOCLR = PIN_DAT))

//Karl
#define DAT_MODE(x) ((x == OUT) ? (LPC_GPIO0->FIODIR |= PIN_DAT) : (LPC_GPIO0->FIODIR &= ~PIN_DAT))
#define BUS_VAL()                ((LPC_GPIO0->FIOPIN  & PIN_DAT) != 0)

#define SPI_START   (0x70)              /* Start byte for SPI transfer        */
#define SPI_RD      (0x01)              /* WR bit 1 within start              */
#define SPI_WR      (0x00)              /* WR bit 0 within start              */
#define SPI_DATA    (0x02)              /* RS bit 1 within start byte         */
#define SPI_INDEX   (0x00)              /* RS bit 0 within start byte         */

//Karl
#define BG_COLOR  0                     /* Background color                   */
#define TXT_COLOR 1                     /* Text color                         */

/*---------------------------- Global variables ------------------------------*/

/******************************************************************************/
//Karl
//static volatile unsigned short TextColor = Black, BackColor = White;
static volatile unsigned short Color[2] = {White, Black};
static unsigned char Himax;

/************************ Local auxiliary functions ***************************/

/*******************************************************************************
* Delay in while loop cycles                                                   *
*   Parameter:    cnt:    number of while cycles to delay                      *
*   Return:                                                                    *
*******************************************************************************/

static void delay (int cnt) {

  /* MTJ modification for FreeRTOS 
  	CNT is actually in 10ms increments
	Instead of a busy loop (which is bad and unpredictable depending on compiler settings)
	I put in a FreeRTOS delay call 	   */
  /*
  cnt <<= DELAY_2N;
  while (cnt--);
  */
  vTaskDelay(cnt*10/portTICK_RATE_MS);
}

//MTJ
static unsigned char delay_val;
void LCD_CS(unsigned char val) {
	if (val == 0) {
		// Make sure that the unit is idle
		while (SSP_GetStatus(LPC_SSP1,SSP_STAT_BUSY)==SET);
		LPC_GPIO0->FIOCLR = PIN_CS;
	} else {
		// Delay before & after	 and make sure that the unit is idle
		while (SSP_GetStatus(LPC_SSP1,SSP_STAT_BUSY)==SET);
		delay_val = 10;
		while (delay_val--);
		LPC_GPIO0->FIOSET = PIN_CS;
		delay_val = 10;
		while (delay_val--); 
	}
		
}
#if 0
/*******************************************************************************
* Send 1 byte over the serial communication                                    *
*   Parameter:    byte:   byte to be sent                                      *
*   Return:               byte read while sending                              *
*******************************************************************************/

static unsigned char spi_send (unsigned char byte) {
  unsigned char retval;
  SSP_DATA_SETUP_Type dataCfg;
  
  dataCfg.tx_data = &byte;
  dataCfg.rx_data = &retval;
  dataCfg.length = 1;

  SSP_ReadWrite(LPC_SSP1,&dataCfg,SSP_TRANSFER_POLLING);
  return(retval);
#if 0
  LPC_SSP1->DR = byte;
  while (!(LPC_SSP1->SR & RNE));        /* Wait for send to finish            */
  return (LPC_SSP1->DR); 
#endif
#if 0
  SSP_SendData(LPC_SSP1,byte);
  while (!(SSP_GetStatus(LPC_SSP1,SSP_STAT_RXFIFO_NOTEMPTY)	== SET));
  return(SSP_ReceiveData(LPC_SSP1));  
#endif
}
#endif


/*******************************************************************************
* Write a command the LCD controller                                           *
*   Parameter:    cmd:    command to be written                                *
*   Return:                                                                    *
*******************************************************************************/

static void wr_cmd (unsigned char cmd) {

  LCD_CS(0);
  //spi_send(SPI_START | SPI_WR | SPI_INDEX);   /* Write : RS = 0, RW = 0       */
  //spi_send(0);
  //spi_send(cmd);
  SSP_DATA_SETUP_Type dataCfg;
  
  unsigned char tbuf[3];
  tbuf[0] = SPI_START | SPI_WR | SPI_INDEX;
  tbuf[1] = 0;
  tbuf[2] = cmd;
  dataCfg.tx_data = tbuf;
  dataCfg.rx_data = NULL;
  dataCfg.length = 3;
  SSP_ReadWrite(LPC_SSP1,&dataCfg,SSP_TRANSFER_POLLING);
  LCD_CS(1);
}


/*******************************************************************************
* Write data to the LCD controller                                             *
*   Parameter:    dat:    data to be written                                   *
*   Return:                                                                    *
*******************************************************************************/

static void wr_dat (unsigned short dat) {

  LCD_CS(0);
  //spi_send(SPI_START | SPI_WR | SPI_DATA);    /* Write : RS = 1, RW = 0       */
  //spi_send((dat >>   8));                     /* Write D8..D15                */
  //spi_send((dat & 0xFF));                     /* Write D0..D7                 */
  
  SSP_DATA_SETUP_Type dataCfg;
  unsigned char tbuf[3];
  tbuf[0] = SPI_START | SPI_WR | SPI_DATA;
  tbuf[1] = dat >> 8;
  tbuf[2] = dat & 0xFF;
  dataCfg.tx_data = tbuf;
  dataCfg.rx_data = NULL;
  dataCfg.length = 3;
  /*printf("WR: ");
  int i;
  for (i=0;i<3;i++) {
  	printf("%x ",tbuf[i]);
  }
  printf("\n");	*/
  SSP_ReadWrite(LPC_SSP1,&dataCfg,SSP_TRANSFER_POLLING);
  LCD_CS(1);
}


/*******************************************************************************
* Start of data writing to the LCD controller                                  *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

static void wr_dat_start (void) {
  unsigned char outData = SPI_START | SPI_WR | SPI_DATA;
  SSP_DATA_SETUP_Type dataCfg;
  LCD_CS(0);
  //spi_send(SPI_START | SPI_WR | SPI_DATA);    /* Write : RS = 1, RW = 0       */
  dataCfg.tx_data = &outData;
  dataCfg.rx_data = NULL;
  dataCfg.length = 1;
  SSP_ReadWrite(LPC_SSP1,&dataCfg,SSP_TRANSFER_POLLING);
}


/*******************************************************************************
* Stop of data writing to the LCD controller                                   *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

static void wr_dat_stop (void) {
  LCD_CS(1);
}


/*******************************************************************************
* Data writing to the LCD controller                                           *
*   Parameter:    dat:    data to be written                                   *
*   Return:                                                                    *
*******************************************************************************/

static void wr_dat_only (unsigned short dat) {
  //spi_send((dat >>   8));                     /* Write D8..D15                */
  //spi_send((dat & 0xFF));                     /* Write D0..D7                 */
  SSP_DATA_SETUP_Type dataCfg;
  
  unsigned char tbuf[2];
  tbuf[0] = dat >> 8;
  tbuf[1] = dat & 0xFF;
  dataCfg.tx_data = tbuf;
  dataCfg.rx_data = NULL;
  dataCfg.length = 2;
  SSP_ReadWrite(LPC_SSP1,&dataCfg,SSP_TRANSFER_POLLING);
}


/*******************************************************************************
* Read data from the LCD controller                                            *
*   Parameter:                                                                 *
*   Return:               read data                                            *
*******************************************************************************/

static unsigned short rd_dat (void) {
#if 0
  unsigned short val = 0;

  LCD_CS(0);
  spi_send(SPI_START | SPI_RD | SPI_DATA);    /* Read: RS = 1, RW = 1         */
  spi_send(0);                                /* Dummy read 1                 */
  val   = spi_send(0);                        /* Read D8..D15                 */
  val <<= 8;
  val  |= spi_send(0);                        /* Read D0..D7                  */
  LCD_CS(1);
#endif

  SSP_DATA_SETUP_Type dataCfg; 
  #define BUF_LEN 7
  unsigned char tbuf[BUF_LEN];
  unsigned char rbuf[BUF_LEN];
  unsigned short val;
  int i;
  LCD_CS(0);
  tbuf[0] = SPI_START | SPI_RD | SPI_DATA;
  for (i=1;i<BUF_LEN;i++) {
  	tbuf[i] = 0x0;
  }
  //for (i=0;i<BUF_LEN;i++) {
  //	rbuf[i] = 0xFD;
  //}	 
  dataCfg.tx_data = tbuf;
  dataCfg.rx_data = rbuf;
  dataCfg.length = BUF_LEN;
  SSP_ReadWrite(LPC_SSP1,&dataCfg,SSP_TRANSFER_POLLING);
  val = rbuf[6];
  val = val << 8;
  val = val + rbuf[5];
  LCD_CS(1);
  return (val);
}


/*******************************************************************************
* Write a value to the to LCD register                                         *
*   Parameter:    reg:    register to be written                               *
*                 val:    value to write to the register                       *
*******************************************************************************/

static void wr_reg (unsigned char reg, unsigned short val) {
  wr_cmd(reg);
  wr_dat(val);
}


/*******************************************************************************
* Read from the LCD register                                                   *
*   Parameter:    reg:    register to be read                                  *
*   Return:               value read from the register                         *
*******************************************************************************/

static unsigned short rd_reg (unsigned char reg) {
  wr_cmd(reg);
  return(rd_dat());
}


//Karl
//This function determines if the old or new LCD driver
//is present.
//This is done by bit-banging a read operation because the
//new LCD drivers are configured for 3-wire SPI.
/*******************************************************************************
* Transfer 1 byte over the serial communication                                *
*   Parameter:    byte:   byte to be sent                                      *
*                 mode:   OUT = transmit byte, IN = receive byte               *
*   Return:               byte read while sending                              *
*******************************************************************************/
static unsigned char spi_tran_man (unsigned char byte, unsigned int mode) {
  unsigned char val = 0;
  int i;

  if (mode == OUT) { DAT_MODE (OUT); }
  else             { DAT_MODE (IN);  }

  for (i = 7; i >= 0; i--) {
    LCD_CLK(0);
    delay(0.1);
    if (mode == OUT) {
      LCD_DAT((byte & (1 << i)) != 0);
    }
    else {
      val |= (BUS_VAL() << i);
    }
    LCD_CLK(1);
    delay(0.1);
  }
  return (val);
}

//Karl
/*******************************************************************************
* Read LCD controller ID (Himax GLCD)                                          *
*   Parameter:    (none)                                                       *
*   Return:       controller ID                                                *
*******************************************************************************/

static unsigned short rd_id_man (void) {
  unsigned short val;

  // Set MOSI, MISO and SCK as GPIO pins, with pull-down/pull-up disabled     
  LPC_PINCON->PINSEL0  &= ~((3 << 18) | (3 << 16) | (3 << 14));
  LPC_PINCON->PINMODE0 |= 0x000AA000;
  LPC_GPIO0->FIODIR    |= PIN_CLK;      // SCK pin is GPIO output             
  
  LCD_CS(1);                           // Set chip select high               
  LCD_CLK(1);                           // Set clock high                     
  
  LCD_CS(0);
  spi_tran_man (SPI_START | SPI_WR | SPI_INDEX, OUT);
  spi_tran_man (0x00, OUT);
  spi_tran_man (0x00, OUT);
  LCD_CS(1);
  
  LCD_CS(0);
  spi_tran_man (SPI_START | SPI_RD | SPI_DATA, OUT);
  val = spi_tran_man(0, IN);
  LCD_CS(1);
  
  // Connect MOSI, MISO, and SCK to SSP peripheral                            
  LPC_GPIO0->FIODIR    &= ~PIN_CLK; //drive CLK to 0
  LPC_PINCON->PINSEL0  |= (2 << 18) | (2 << 16) | (2 << 14);
  LPC_PINCON->PINMODE0 &= ~0x000FF000;
  
  return (val);
}

/************************ Exported functions **********************************/

/*******************************************************************************
* Initialize the Graphic LCD controller                                        *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_Init (void) { 
  unsigned short driverCode = 0x00;
  PINSEL_CFG_Type PinCfg;
  SSP_CFG_Type SSP_ConfigStruct;
  
  /* Enable clock for SSP1, clock = CCLK / 2                                  */
  //LPC_SC->PCONP       |= 0x00000400;
  //LPC_SC->PCLKSEL0    |= 0x00200000;
  CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_SSP1, 2);   

  /* Configure the LCD Control pins                                           */
  LPC_PINCON->PINSEL9 &= 0xF0FFFFFF;
  LPC_GPIO4->FIODIR   |= 0x30000000;
  LPC_GPIO4->FIOSET    = 0x20000000;

  /* SSEL1 is GPIO output set to high                                         */
  /* LPC_GPIO0->FIODIR   |= 0x00000040;
  LPC_GPIO0->FIOSET    = 0x00000040;   */
  /*LPC_PINCON->PINSEL0 &= 0xFFF03FFF;
  LPC_PINCON->PINSEL0 |= 0x000A8000; */

  /* Enable SPI in Master Mode, CPOL=1, CPHA=1                                */
  /* Max. 12.5 MBit used for Data Transfer @ 100MHz                           */
  /*LPC_SSP1->CR0        = 0x01C7;
  LPC_SSP1->CPSR       = 0x02;
  LPC_SSP1->CR1        = 0x02;	*/
  // MTJ: Here is the right way to initialize this unit
  // configure the pins for SSP1
  // P0.6 -- SSEL1
  // P0.7 -- SCK1
  // P0.8 -- MISO1
  // P0.9 -- MOSI1
  
  // Set P0.6 (as GPIO) to have an output of '1' (idle CS)
  GPIO_SetDir(0, 0x00000040, 1); //Port 0, Bit 6, Direction is Output
  GPIO_SetValue(0, 0x00000040);
  
  PinCfg.Funcnum = 0;
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL; //not open drain
  PinCfg.Pinmode = PINSEL_PINMODE_PULLUP; //Internal pull-up resistor
  //PinCfg.OpenDrain = 0;
  //PinCfg.Pinmode = 0;
  PinCfg.Portnum = 0;
  PinCfg.Pinnum = 6;
  PINSEL_ConfigPin(&PinCfg);
  
  PinCfg.Funcnum = 2;
  PinCfg.Pinnum = 7;
  PINSEL_ConfigPin(&PinCfg);
  
  PinCfg.Pinnum = 8;
  PINSEL_ConfigPin(&PinCfg);
  
  PinCfg.Pinnum = 9;
  PINSEL_ConfigPin(&PinCfg);
  
  // initialize the configuration struction with default values
  SSP_ConfigStructInit(&SSP_ConfigStruct);
  // then change the values we care about
  SSP_ConfigStruct.ClockRate = 0xBEBC20; // 12.5MHz
  SSP_ConfigStruct.CPOL = SSP_CPOL_LO;
  SSP_ConfigStruct.CPHA = SSP_CPHA_SECOND;
  // initialize the SSP1 unit
  SSP_Init(LPC_SSP1, &SSP_ConfigStruct);
  LPC_SSP1->IMSC = 0;
  // now turn it on
  SSP_Cmd(LPC_SSP1, ENABLE);
  // End of new initialization
 
  
  delay(5);                             /* Delay 50 ms                        */
  
  //Karl
  driverCode = rd_id_man();
  
  if (driverCode == 0x00) {
	  SSP_DATA_SETUP_Type dataCfg; 
	  unsigned char tbuf[4];
	  unsigned char rbuf[4];
	  int i;
	  LCD_CS(0);
	  tbuf[0] = SPI_START | SPI_RD | SPI_DATA;
	  for (i=1; i<4; i++) {
		tbuf[i] = 0x0;
	  }
	  dataCfg.tx_data = tbuf;
	  dataCfg.rx_data = rbuf;
	  dataCfg.length = 4;
	  SSP_ReadWrite(LPC_SSP1, &dataCfg, SSP_TRANSFER_POLLING);
	  driverCode = rbuf[2] << 8;
	  driverCode = driverCode + rbuf[3];
	  LCD_CS(1);
  }

  if (driverCode == 0x47) {             /* LCD with HX8347-D LCD Controller   */
	Himax = 1;                          /* Set Himax LCD controller flag      */
    /* Driving ability settings ----------------------------------------------*/
    wr_reg(0xEA, 0x00);                 /* Power control internal used (1)    */
    wr_reg(0xEB, 0x20);                 /* Power control internal used (2)    */
    wr_reg(0xEC, 0x0C);                 /* Source control internal used (1)   */
    wr_reg(0xED, 0xC7);                 /* Source control internal used (2)   */
    wr_reg(0xE8, 0x38);                 /* Source output period Normal mode   */
    wr_reg(0xE9, 0x10);                 /* Source output period Idle mode     */
    wr_reg(0xF1, 0x01);                 /* RGB 18-bit interface ;0x0110       */
    wr_reg(0xF2, 0x10);

    /* Adjust the Gamma Curve ------------------------------------------------*/
    wr_reg(0x40, 0x01);
    wr_reg(0x41, 0x00);
    wr_reg(0x42, 0x00);
    wr_reg(0x43, 0x10);
    wr_reg(0x44, 0x0E);
    wr_reg(0x45, 0x24);
    wr_reg(0x46, 0x04);
    wr_reg(0x47, 0x50);
    wr_reg(0x48, 0x02);
    wr_reg(0x49, 0x13);
    wr_reg(0x4A, 0x19);
    wr_reg(0x4B, 0x19);
    wr_reg(0x4C, 0x16);

    wr_reg(0x50, 0x1B);
    wr_reg(0x51, 0x31);
    wr_reg(0x52, 0x2F);
    wr_reg(0x53, 0x3F);
    wr_reg(0x54, 0x3F);
    wr_reg(0x55, 0x3E);
    wr_reg(0x56, 0x2F);
    wr_reg(0x57, 0x7B);
    wr_reg(0x58, 0x09);
    wr_reg(0x59, 0x06);
    wr_reg(0x5A, 0x06);
    wr_reg(0x5B, 0x0C);
    wr_reg(0x5C, 0x1D);
    wr_reg(0x5D, 0xCC);

    /* Power voltage setting -------------------------------------------------*/
    wr_reg(0x1B, 0x1B);
    wr_reg(0x1A, 0x01);
    wr_reg(0x24, 0x2F);
    wr_reg(0x25, 0x57);
    wr_reg(0x23, 0x88);

    /* Power on setting ------------------------------------------------------*/
    wr_reg(0x18, 0x36);                 /* Internal oscillator frequency adj  */
    wr_reg(0x19, 0x01);                 /* Enable internal oscillator         */
    wr_reg(0x01, 0x00);                 /* Normal mode, no scrool             */
    wr_reg(0x1F, 0x88);                 /* Power control 6 - DDVDH Off        */
    delay(20);
    wr_reg(0x1F, 0x82);                 /* Power control 6 - Step-up: 3 x VCI */
    delay(5);
    wr_reg(0x1F, 0x92);                 /* Power control 6 - Step-up: On      */
    delay(5);
    wr_reg(0x1F, 0xD2);                 /* Power control 6 - VCOML active     */
    delay(5);

    /* Color selection -------------------------------------------------------*/
    wr_reg(0x17, 0x55);                 /* RGB, System interface: 16 Bit/Pixel*/
    wr_reg(0x00, 0x00);                 /* Scrolling off, no standby          */

    /* Interface config ------------------------------------------------------*/
    wr_reg(0x2F, 0x11);                 /* LCD Drive: 1-line inversion        */
    wr_reg(0x31, 0x00);
    wr_reg(0x32, 0x00);                 /* DPL=0, HSPL=0, VSPL=0, EPL=0       */

    /* Display on setting ----------------------------------------------------*/
    wr_reg(0x28, 0x38);                 /* PT(0,0) active, VGL/VGL            */
    delay(20);
    wr_reg(0x28, 0x3C);                 /* Display active, VGL/VGL            */

   #if (LANDSCAPE == 1)
    #if (ROTATE180 == 0)
     wr_reg (0x16, 0xA8);
    #else
     wr_reg (0x16, 0x68);
    #endif
   #else
    #if (ROTATE180 == 0)
     wr_reg (0x16, 0x08);
    #else
     wr_reg (0x16, 0xC8);
    #endif
   #endif

    /* Display scrolling settings --------------------------------------------*/
    wr_reg(0x0E, 0x00);                 /* TFA MSB                            */
    wr_reg(0x0F, 0x00);                 /* TFA LSB                            */
    wr_reg(0x10, 320 >> 8);             /* VSA MSB                            */
    wr_reg(0x11, 320 &  0xFF);          /* VSA LSB                            */
    wr_reg(0x12, 0x00);                 /* BFA MSB                            */
    wr_reg(0x13, 0x00);                 /* BFA LSB                            */
	
   #if (LANDSCAPE == 1)
    wr_reg(0x06, 0x00);            // Row address start MSB              
    wr_reg(0x08, 0x00);            // Row address end MSB
   #else	
   	wr_reg(0x02, 0x00);            // Column address start MSB           
    wr_reg(0x04, 0x00);            // Column address end MSB
   #endif	
  }
  else {
	Himax = 0;                          /* This is not Himax LCD controller   */
    /* Start Initial Sequence ------------------------------------------------*/
   #if (ROTATE180 == 1)
    wr_reg(0x01, 0x0000);               /* Clear SS bit                       */
   #else
    wr_reg(0x01, 0x0100);               /* Set SS bit                         */
   #endif
    wr_reg(0x02, 0x0700);               /* Set 1 line inversion               */
    wr_reg(0x04, 0x0000);               /* Resize register                    */
    wr_reg(0x08, 0x0207);               /* 2 lines front, 7 back porch        */
    wr_reg(0x09, 0x0000);               /* Set non-disp area refresh cyc ISC  */
    wr_reg(0x0A, 0x0000);               /* FMARK function                     */
    wr_reg(0x0C, 0x0000);               /* RGB interface setting              */
    wr_reg(0x0D, 0x0000);               /* Frame marker Position              */
    wr_reg(0x0F, 0x0000);               /* RGB interface polarity             */
	
    /* Power On sequence -----------------------------------------------------*/
    wr_reg(0x10, 0x0000);               /* Reset Power Control 1              */
    wr_reg(0x11, 0x0000);               /* Reset Power Control 2              */
    wr_reg(0x12, 0x0000);               /* Reset Power Control 3              */
    wr_reg(0x13, 0x0000);               /* Reset Power Control 4              */
    //delay(20);                          /* Discharge cap power voltage (200ms)*/
    wr_reg(0x10, 0x12B0);               /* SAP, BT[3:0], AP, DSTB, SLP, STB   */
    wr_reg(0x11, 0x0007);               /* DC1[2:0], DC0[2:0], VC[2:0]        */
    //delay(5);                           /* Delay 50 ms                        */
    wr_reg(0x12, 0x01BD);               /* VREG1OUT voltage                   */
    //delay(5);                           /* Delay 50 ms                        */
    wr_reg(0x13, 0x1400);               /* VDV[4:0] for VCOM amplitude        */
    wr_reg(0x29, 0x000E);               /* VCM[4:0] for VCOMH                 */
    //delay(5);                           /* Delay 50 ms                        */
    wr_reg(0x20, 0x0000);               /* GRAM horizontal Address            */
    wr_reg(0x21, 0x0000);               /* GRAM Vertical Address              */

    /* Adjust the Gamma Curve ------------------------------------------------*/
    switch (driverCode) {
      case 0x5408:                      /* LCD with SPFD5408 LCD Controller   */
		wr_reg(0x30, 0x0B0D);
        wr_reg(0x31, 0x1923);
        wr_reg(0x32, 0x1C26);
        wr_reg(0x33, 0x261C);
        wr_reg(0x34, 0x2419);
        wr_reg(0x35, 0x0D0B);
        wr_reg(0x36, 0x1006);
        wr_reg(0x37, 0x0610);
        wr_reg(0x38, 0x0706);
        wr_reg(0x39, 0x0304);
        wr_reg(0x3A, 0x0E05);
        wr_reg(0x3B, 0x0E01);
        wr_reg(0x3C, 0x010E);
        wr_reg(0x3D, 0x050E);
        wr_reg(0x3E, 0x0403);
        wr_reg(0x3F, 0x0607);
        break;

      case 0x9325:                      /* LCD with RM68050 LCD Controller    */
        wr_reg(0x0030,0x0000);
        wr_reg(0x0031,0x0607);
        wr_reg(0x0032,0x0305);
        wr_reg(0x0035,0x0000);
        wr_reg(0x0036,0x1604);
        wr_reg(0x0037,0x0204);
        wr_reg(0x0038,0x0001);
        wr_reg(0x0039,0x0707);
        wr_reg(0x003C,0x0000);
        wr_reg(0x003D,0x000F);
        break;

      case 0x9320:                      /* LCD with ILI9320 LCD Controller    */
      default:                          /* LCD with other LCD Controller      */
        wr_reg(0x30, 0x0006);
        wr_reg(0x31, 0x0101);
        wr_reg(0x32, 0x0003);
        wr_reg(0x35, 0x0106);
        wr_reg(0x36, 0x0B02);
        wr_reg(0x37, 0x0302);
        wr_reg(0x38, 0x0707);
        wr_reg(0x39, 0x0007);
        wr_reg(0x3C, 0x0600);
        wr_reg(0x3D, 0x020B);
        break;
    }

    /* Set GRAM area ---------------------------------------------------------*/
    wr_reg(0x50, 0x0000);               /* Horizontal GRAM Start Address      */
    wr_reg(0x51, (HEIGHT-1));           /* Horizontal GRAM End   Address      */
    wr_reg(0x52, 0x0000);               /* Vertical   GRAM Start Address      */
    wr_reg(0x53, (WIDTH-1));            /* Vertical   GRAM End   Address      */

    /* Set Gate Scan Line ----------------------------------------------------*/
    switch (driverCode) {
      case 0x9325:                      /* LCD with RM68050 LCD Controller    */
       #if (LANDSCAPE ^ ROTATE180)
		wr_reg(0x60, 0x2700);
       #else
		wr_reg(0x60, 0xA700);
       #endif
        break;

      case 0x5408:                      /* LCD with SPFD5408 LCD Controller   */
      case 0x9320:                      /* LCD with ILI9320 LCD Controller    */
      default:                          /* LCD with other LCD Controller      */
       #if (LANDSCAPE ^ ROTATE180)
        wr_reg(0x60, 0xA700);
       #else
        wr_reg(0x60, 0x2700);
       #endif
        break;
    }
    wr_reg(0x61, 0x0001);               /* NDL,VLE, REV                       */
    wr_reg(0x6A, 0x0000);               /* Set scrolling line                 */

    /* Partial Display Control -----------------------------------------------*/
    wr_reg(0x80, 0x0000);
    wr_reg(0x81, 0x0000);
    wr_reg(0x82, 0x0000);
    wr_reg(0x83, 0x0000);
    wr_reg(0x84, 0x0000);
    wr_reg(0x85, 0x0000);

    /* Panel Control ---------------------------------------------------------*/
    wr_reg(0x90, 0x0010);
    wr_reg(0x92, 0x0000);
    wr_reg(0x93, 0x0003);
    wr_reg(0x95, 0x0110);
    wr_reg(0x97, 0x0000);
    wr_reg(0x98, 0x0000);

    /* Set GRAM write direction
       I/D=11 (Horizontal : increment, Vertical : increment)                  */
  #if (LANDSCAPE == 1)
    /* AM=1   (address is updated in vertical writing direction)              */
    wr_reg(0x03, 0x1038);
  #else
    /* AM=0   (address is updated in horizontal writing direction)            */
    wr_reg(0x03, 0x1030);
  #endif

    wr_reg(0x07, 0x0137);               /* 262K color and display ON          */
  }
  LPC_GPIO4->FIOSET = 0x10000000;       // Turn on the backlight
  
  // Initialize the interrupt driver for bulk SPI transfer on SSP1
  if (vtSSPIsrInit(1) != vtSSPInitSuccess) {
  	VT_HANDLE_FATAL_ERROR(0);
  }
}


/*******************************************************************************
* Set draw window region                                                       *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        window width in pixel                            *
*                   h:        window height in pixels                          *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_SetWindow (unsigned int x, unsigned int y, unsigned int w, unsigned int h) {
  unsigned int xe, ye;

  if (Himax) {
    xe = x+w-1;
    ye = y+h-1;
	
	wr_reg(0x02, y  >>    8);           // Column address start MSB           
    wr_reg(0x03, y  &  0xFF);           // Column address start LSB           
    wr_reg(0x04, ye >>    8);           // Column address end MSB             
    wr_reg(0x05, ye &  0xFF);           // Column address end LSB             
  
    wr_reg(0x06, x  >>    8);           // Row address start MSB              
    wr_reg(0x07, x  &  0xFF);           // Row address start LSB              
    wr_reg(0x08, xe >>    8);           // Row address end MSB                
    wr_reg(0x09, xe &  0xFF);           // Row address end LSB                
  }
  else {
   #if (LANDSCAPE == 1)
    wr_reg(0x50, x);                    /* Horizontal GRAM Start Address      */
    wr_reg(0x51, x+w-1);                /* Horizontal GRAM End   Address (-1) */
    wr_reg(0x52, y);                    /* Vertical   GRAM Start Address      */
    wr_reg(0x53, y+h-1);                /* Vertical   GRAM End   Address (-1) */
    wr_reg(0x20, x);
    wr_reg(0x21, y);
   #else
    wr_reg(0x50, y);                    /* Vertical   GRAM Start Address      */
    wr_reg(0x51, y+h-1);                /* Vertical   GRAM End   Address (-1) */
    wr_reg(0x52, x);                    /* Horizontal GRAM Start Address      */
    wr_reg(0x53, x+w-1);                /* Horizontal GRAM End   Address (-1) */
    wr_reg(0x20, y);
    wr_reg(0x21, x);
   #endif
  }
}


/*******************************************************************************
* Set draw window region to whole screen                                       *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_WindowMax (void) {
  GLCD_SetWindow (0, 0, HEIGHT, WIDTH);
}

/*******************************************************************************
* Read a row of pixels                                             *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position    
*                   width: number of pixels in the row                            *
*                   buffer: buffer in which to place the pixels
*   Return:                                                                    *
*******************************************************************************/

void GLCD_GetPixelRow (unsigned int x, unsigned int y, unsigned int width, unsigned short int *buffer) {
	#if (LANDSCAPE == 1)
	  wr_reg(0x20, y);
	#else
	  Not implemented
	#endif
	  int i;
	  unsigned int cur_col = WIDTH-width-x; // we'll go in reverse order
	  for (i=0;i<width;i++) {
		wr_reg(0x21,cur_col);
		// The bit ordering of the colors that are returned is funky, I'm not sure why, but here it what it is
		// B4 B3 B2 B1   B0 G5 G4 G3  G2 G1 G0 R4  R3 R2 R1 R0
		unsigned short int temp = rd_reg(0x22);
		vtLEDOff(0xFF);
        vtLEDOn(0x03);
		// Here is the ordering that we want to return
		// R4 R3 R2 R1   R0 G5 G4 G3  G2 G1 G0 B4  B3 B2 B1 B0
		/*
		red = temp & 0x001F;
		blue = temp & 0xF800;
		green = temp & 0x07E0; 
		*/
		(*buffer) = ((temp & 0x001F) << 11) + (temp & 0x07E0) + ((temp & 0xF800) >> 11);
		buffer++;
		cur_col++;
  }
}

/*******************************************************************************
* Read a pixel                                             *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*   Return:                                                                    *
*******************************************************************************/

unsigned short GLCD_GetPixel (unsigned int x, unsigned int y) {
#if (LANDSCAPE == 1)
  wr_reg(0x20, y);
  wr_reg(0x21, WIDTH-1-x);
#else
  wr_reg(0x20, x);
  wr_reg(0x21, y);
#endif
  unsigned short int temp = rd_reg(0x22);
  // The bit ordering of the colors that are returned is funky, I'm not sure why, but here it what it is
  // B4 B3 B2 B1   B0 G5 G4 G3  G2 G1 G0 R4  R3 R2 R1 R0
  unsigned short int retval;
  // Here is the ordering that we want to return
  // R4 R3 R2 R1   R0 G5 G4 G3  G2 G1 G0 B4  B3 B2 B1 B0
  // Red
  unsigned short red, green, blue;
  red = temp & 0x001F;
  blue = temp & 0xF800;
  green = temp & 0x07E0;
  retval = (red << 11) + green + (blue >> 11);
  return(retval);
}

/*******************************************************************************
* Draw a pixel in foreground color                                             *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_PutPixel (unsigned int x, unsigned int y) {
  if (Himax) {
   #if (LANDSCAPE == 1)
	wr_reg(0x02, x >>    8);            // Column address start MSB           
    wr_reg(0x03, x &  0xFF);            // Column address start LSB           
    //wr_reg(0x06, y >>    8);            // Row address start MSB              
    wr_reg(0x07, y &  0xFF);            // Row address start LSB              
   #else	    
	//wr_reg(0x02, x >>    8);            // Column address start MSB           
    wr_reg(0x03, x &  0xFF);            // Column address start LSB           
    wr_reg(0x06, y >>    8);            // Row address start MSB              
    wr_reg(0x07, y &  0xFF);            // Row address start LSB              
   #endif
  }
  else {
   #if (LANDSCAPE == 1)
    wr_reg(0x20, y);
    wr_reg(0x21, WIDTH-1-x);
   #else
    wr_reg(0x20, x);
    wr_reg(0x21, y);
   #endif
  }

  wr_reg(0x22, Color[TXT_COLOR]);
}


/*******************************************************************************
* Set foreground color                                                         *
*   Parameter:      color:    foreground color                                 *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_SetTextColor (unsigned short color) {
  Color[TXT_COLOR] = color;
}


/*******************************************************************************
* Set background color                                                         *
*   Parameter:      color:    background color                                 *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_SetBackColor (unsigned short color) {
  Color[BG_COLOR] = color;
}



/*******************************************************************************
* Clear display                                                                *
*   Parameter:      color:    display clearing color                           *
*   Return:                                                                    *
*******************************************************************************/
unsigned short colorBuf[WIDTH];
void GLCD_Clear (unsigned short color) {
#if LANDSCAPE
#else
  Not implemented
#endif
  
  unsigned int i;
  vtSSPIsrData dataCfg;  
  unsigned char *tptr = (unsigned char *) colorBuf;
  
  dataCfg.tx_data = colorBuf;
  dataCfg.length = WIDTH*sizeof(unsigned short);

  tptr[0] = color >> 8;
  tptr[1] = color & 0xFF;
  for (i=1;i<WIDTH;i++) {
	colorBuf[i] = colorBuf[0];
  }
  
  GLCD_WindowMax();
  wr_cmd(0x22);
  wr_dat_start();
  
  for (i=0; i<HEIGHT; i++) {
	vtSSPStartOperation(&dataCfg);
	if (vtSSPWaitComplete(portMAX_DELAY) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(0);
	}
  }
  wr_dat_stop();
}

void GLCD_ClearWindow (unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned short color) {
#if LANDSCAPE
#else
  Not implemented
#endif  
  unsigned int i;
  vtSSPIsrData dataCfg;  
  unsigned char *tptr = (unsigned char *) colorBuf;
  
  dataCfg.tx_data = colorBuf;
  dataCfg.length = width*sizeof(unsigned short);

  tptr[0] = color >> 8;
  tptr[1] = color & 0xFF;
  for (i=1;i<width;i++) colorBuf[i] = colorBuf[0];
  GLCD_SetWindow(y, WIDTH-x-width, height, width);
  wr_cmd(0x22);
  wr_dat_start();
  for (i=0;i<height;i++) {
	vtSSPStartOperation(&dataCfg);
	if (vtSSPWaitComplete(portMAX_DELAY) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(0);
	}
  }
  wr_dat_stop();
}



/*******************************************************************************
* Draw character on given position                                             *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   cw:       character width in pixel                         *
*                   ch:       character height in pixels                       *
*                   c:        pointer to character bitmap                      *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_DrawChar_U8 (unsigned int x, unsigned int y, unsigned int cw, unsigned int ch, unsigned char *c) {
  int i, j;

#if (LANDSCAPE == 1)
  x = WIDTH-x-cw;
  GLCD_SetWindow(y, x, ch, cw);
#else
  GLCD_SetWindow(x, y, cw, ch);
#endif
  wr_cmd(0x22);
  wr_dat_start();
  for (j = 0; j < ch; j++) {
#if (LANDSCAPE == 1)
    for (i = cw-1; i >= 0; i--) {
#else
    for (i = 0; i <= cw-1; i++) {
#endif
      if(((*c) & (1 << i)) == 0x00) {
        wr_dat_only(Color[BG_COLOR]);
      } else {
        wr_dat_only(Color[TXT_COLOR]);
      }
    }
    c++;
  }
  wr_dat_stop();
}


/*******************************************************************************
* Draw character on given position                                             *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   cw:       character width in pixel                         *
*                   ch:       character height in pixels                       *
*                   c:        pointer to character bitmap                      *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_DrawChar_U16 (unsigned int x, unsigned int y, unsigned int cw, unsigned int ch, unsigned short *c) {
  int i, j;
  int cnt;
  unsigned short revBackColor;
  unsigned short revTextColor;
  vtSSPIsrData dataCfg;
  unsigned short curBits;

  revBackColor = Color[BG_COLOR] >> 8;
  revBackColor += Color[BG_COLOR] << 8;
  revTextColor = Color[TXT_COLOR] >> 8;
  revTextColor += Color[TXT_COLOR] << 8;

  GLCD_PutPixel(x,y);
#if (LANDSCAPE == 1)
  if (!Himax) {
    x = WIDTH-x-cw;
  }
  if (x < 0) {
    // writing past the end of the line -- ignore it
  	return;
  }
  if (y+ch > HEIGHT) {
    // writing beyond the bottom of the screen -- ignore it
  	return;
  }
  GLCD_SetWindow(y, x, ch, cw);
#else
  if (x+cw > WIDTH)) {
    // writing past the end of the line -- ignore it
  	return;
  }
  if (y+ch > HEIGHT) {
    // writing beyond the bottom of the screen -- ignore it
  	return;
  }
  GLCD_SetWindow(x, y, cw, ch);
#endif
  dataCfg.tx_data = colorBuf;
  //dataCfg.length = cw*sizeof(unsigned short);

  wr_cmd(0x22);
  wr_dat_start();
  cnt = 0;
  
  for (j = 0; j < ch; j++) {
	curBits = (*c);
	c++;
#if (LANDSCAPE == 1)
    for (i = cw-1; i >= 0; i--) {
#else
    for (i = 0; i <= cw-1; i++) {
#endif
	  
	  if (Himax) {
	  	  if (curBits & 0x0001) {
			colorBuf[cnt] = revTextColor; 
		  } else {
			colorBuf[cnt] = revBackColor;
		  }
		  curBits = curBits >> 1;
	  }
	  else {
		  if (curBits & 0x8000) {
			colorBuf[cnt] = revTextColor; 
		  } else {
			colorBuf[cnt] = revBackColor;
		  }
	      curBits = curBits << 1;
	  }
	  cnt++;
    }
	if (cnt > WIDTH-cw) {
		// the buffer is nearly full, so write it out
		dataCfg.length = cnt*sizeof(unsigned short);
		vtSSPStartOperation(&dataCfg);
		if (vtSSPWaitComplete(portMAX_DELAY) != pdPASS) {
			VT_HANDLE_FATAL_ERROR(0);
		}
		cnt = 0;
	}
  }
  if (cnt > 0) {
  	// send out the rest of the buffer if it has not been written yet
	dataCfg.length = cnt*sizeof(unsigned short);
	vtSSPStartOperation(&dataCfg);
	if (vtSSPWaitComplete(portMAX_DELAY) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(0);
	}
  }
  wr_dat_stop();
}


/*******************************************************************************
* Disply character on given line                                               *
*   Parameter:      ln:       line number                                      *
*                   col:      column number                                    *
*                   fi:       font index (0 = 6x8, 1 = 16x24)                  *
*                   c:        ascii character                                  *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_DisplayChar (unsigned int ln, unsigned int col, unsigned char fi, unsigned char c) {

  c -= 32;
  switch (fi) {
    case 0:  /* Font 6 x 8 */
      GLCD_DrawChar_U8 (col *  6, ln *  8,  6,  8, (unsigned char  *)&Font_6x8_h  [c * 8]);
      break;
    case 1:  /* Font 16 x 24 */
      GLCD_DrawChar_U16(col * 16, ln * 24, 16, 24, (unsigned short *)&Font_16x24_h[c * 24]);
      break;
  }
}


/*******************************************************************************
* Disply string on given line                                                  *
*   Parameter:      ln:       line number                                      *
*                   col:      column number                                    *
*                   fi:       font index (0 = 6x8, 1 = 16x24)                  *
*                   s:        pointer to string                                *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_DisplayString (unsigned int ln, unsigned int col, unsigned char fi, unsigned char *s) {
  GLCD_WindowMax();
  while (*s) {
    GLCD_DisplayChar(ln, col++, fi, *s++);
  }
}


/*******************************************************************************
* Clear given line                                                             *
*   Parameter:      ln:       line number                                      *
*                   fi:       font index (0 = 6x8, 1 = 16x24)                  *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_ClearLn (unsigned int ln, unsigned char fi) {
  int i;
  unsigned int cHeight, pixHeight;
  unsigned char *tptr = (unsigned char *) colorBuf;
  vtSSPIsrData dataCfg;

  switch(fi)  {
  	case 0: {
	  cHeight = 8;
	};
	break;
	case 1: {
	  cHeight = 24;
	};
	break;
	default: {
	  return;
	}
  }
  pixHeight = ln * cHeight;
  if (pixHeight + cHeight > HEIGHT) {
    // The specified line is out of bounds
    return;
  }
#if (LANDSCAPE == 1)
  GLCD_SetWindow(pixHeight, 0, cHeight, WIDTH);
#else
  GLCD_SetWindow(0, pixHeight, WIDTH, cHeight);
#endif
  dataCfg.tx_data = colorBuf;
  dataCfg.length = WIDTH*sizeof(unsigned short);

  tptr[0] = Color[BG_COLOR] >> 8;
  tptr[1] = Color[BG_COLOR] & 0xFF;
  for (i=1;i<WIDTH;i++) colorBuf[i] = colorBuf[0];
  wr_cmd(0x22);
  wr_dat_start();
  for (i=0;i<cHeight;i++) {
	vtSSPStartOperation(&dataCfg);
	if (vtSSPWaitComplete(portMAX_DELAY) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(0);
	}
  }
  wr_dat_stop();

#if 0
  unsigned char i;
  unsigned char buf[60];
  int len;

  switch (fi) {
  	case 0:
		len = (WIDTH+5)/6

  GLCD_WindowMax();
  switch (fi) {
    case 0:  /* Font 6 x 8 */
      for (i = 0; i < (WIDTH+5)/6; i++)
        buf[i] = ' ';
      buf[i+1] = 0;
      break;
    case 1:  /* Font 16 x 24 */
      for (i = 0; i < (WIDTH+15)/16; i++)
        buf[i] = ' ';
      buf[i+1] = 0;
      break;
  }
  GLCD_DisplayString (ln, 0, fi, buf);
#endif
}

/*******************************************************************************
* Draw bargraph                                                                *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        maximum width of bargraph (in pixels)            *
*                   val:      value of active bargraph (in 1/1024)             *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_Bargraph (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int val) {
  int i,j;

  val = (val * w) >> 10;                /* Scale value                        */
#if (LANDSCAPE == 1)
  x = WIDTH-x-w;
  GLCD_SetWindow(y, x, h, w);
#else
  GLCD_SetWindow(x, y, w, h);
#endif
  wr_cmd(0x22);
  wr_dat_start();
  for (i = 0; i < h; i++) {
#if (LANDSCAPE == 1)
    for (j = w-1; j >= 0; j--) {
#else
    for (j = 0; j <= w-1; j++) {
#endif
      if(j >= val) {
        wr_dat_only(Color[BG_COLOR]);
      } else {
        wr_dat_only(Color[TXT_COLOR]);
      }
    }
  }
  wr_dat_stop();
}


/*******************************************************************************
* Display graphical bitmap image at position x horizontally and y vertically   *
* (This function is optimized for 16 bits per pixel format, it has to be       *
*  adapted for any other bits per pixel format)                                *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        width of bitmap                                  *
*                   h:        height of bitmap                                 *
*                   bitmap:   address at which the bitmap data resides         *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_Bitmap (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned char *bitmap) {
  unsigned int    i, j;
  unsigned short *bitmap_ptr = (unsigned short *)bitmap;

#if (LANDSCAPE == 1)
  x = WIDTH-x-w;
  GLCD_SetWindow(y, x, h, w);
#else
  GLCD_SetWindow(x, y, w, h);
#endif
  wr_cmd(0x22);
  wr_dat_start();
  for (j = 0; j < h; j++) {
#if (LANDSCAPE == 1)
    for (i = 0; i < w; i++) {
      wr_dat_only(*bitmap_ptr++);
    }
#else
    bitmap_ptr += w-1;
    for (i = 0; i < w; i++) {
      wr_dat_only(*bitmap_ptr--);
    }
    bitmap_ptr += w+1;
#endif
  }
  wr_dat_stop();
}


/*******************************************************************************
* Display graphical bmp file image at position x horizontally and y vertically *
* (This function is optimized for 16 bits per pixel format, it has to be       *
*  adapted for any other bits per pixel format)                                *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        width of bitmap                                  *
*                   h:        height of bitmap                                 *
*                   bmp:      address at which the bmp data resides            *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_Bmp (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned char *bmp) {
  unsigned int i, j;
  unsigned short *bitmap_ptr = (unsigned short *)bmp;
  vtSSPIsrData dataCfg;
  unsigned char *tbuf = (unsigned char *) colorBuf;
  unsigned int bufCnt;

#if (LANDSCAPE == 1)
  x = WIDTH-x-w;
  GLCD_SetWindow(y, x, h, w);
#else
  GLCD_SetWindow(x, y, w, h);
#endif
  dataCfg.tx_data = colorBuf;
  wr_cmd(0x22);
  wr_dat_start();
#if (LANDSCAPE == 1)
  bitmap_ptr += (h*w)-1;
  bufCnt = 0;
  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++) {
      //wr_dat_only(*bitmap_ptr--);
	  tbuf[bufCnt] = (*bitmap_ptr) >> 8; bufCnt++;
	  tbuf[bufCnt] = (*bitmap_ptr) & 0xFF; bufCnt++;
	  //colorBuf[bufCnt] = (*bitmap_ptr); bufCnt++;
	  if (bufCnt >= (WIDTH*2)-2) {
	    dataCfg.length = bufCnt;
		vtSSPStartOperation(&dataCfg);
		if (vtSSPWaitComplete(portMAX_DELAY) != pdPASS) {
			VT_HANDLE_FATAL_ERROR(0);
		}
		bufCnt = 0;
	  }
	  bitmap_ptr--;
    }
  }
  if (bufCnt > 0) {
    dataCfg.length = bufCnt;
	vtSSPStartOperation(&dataCfg);
	if (vtSSPWaitComplete(portMAX_DELAY) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(0);
	}
  }
#else
  bitmap_ptr += ((h-1)*w);
  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++) {
      wr_dat_only(*bitmap_ptr++);
    }
    bitmap_ptr -= 2*w;
  }
#endif
  wr_dat_stop();
}


/*******************************************************************************
* Scroll content of the whole display for dy pixels vertically                 *
*   Parameter:      dy:       number of pixels for vertical scroll             *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_ScrollVertical (unsigned int dy) {
#if (LANDSCAPE == 0)
  static unsigned int y = 0;

  y = y + dy;
  while (y >= HEIGHT) 
    y -= HEIGHT;

  //Karl
  if (Himax) {
    wr_reg(0x01, 0x08);
    wr_reg(0x14, y>>8);                 /* VSP MSB                            */
    wr_reg(0x15, y&0xFF);               /* VSP LSB                            */
  }
  else {
    wr_reg(0x6A, y);
    wr_reg(0x61, 3);
  }
#endif
}

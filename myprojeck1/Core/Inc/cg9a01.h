/*
 * lcd.h
 *
 *  Created on: 29 SEP, 2022
 *  Author: wenqb
 */

#ifndef INC_CG9A01_H_
#define INC_CG9A01_H_

 #include "main.h"

/*
 * How to wire：
 *
 * CG9A01	-- NRF       -- STM32F4
 * -----------------------------------------------------
 * RST   	-- NRF IRQ   -- PB8
 * CS    	-- NRF CS    -- PB7
 * DC   	-- NRF CE    -- PB6
 * SDA   	-- SPI1 MOSI -- PB5 / SPI1 MOSI
 *       	-- SPI1 MISO -- PB4 / SPI1 MISO //reserved
 * SCL   	-- SPI1 CLK  -- PB3 / SPI1 SCK
 * GND   	-- GND       -- GND
 * VCC   	-- 3V3       -- 3V3
 */

#define USE_HORIZONTAL 2  //Set the display direction 0,1,2,3	four directions


void GC9A01_Initial(void);
void Write_Cmd_Data(unsigned char);
void Write_Cmd(unsigned char);
void Write_Data(unsigned char DH,unsigned char DL);
void Write_Bytes(unsigned char * pbuff, unsigned short size);

void delayms(unsigned int tt);
void Write_Data_U16(unsigned int y);
 void LCD_SetPos(unsigned int Xstart,unsigned int Ystart,unsigned int Xend,unsigned int Yend);
void ClearScreen(unsigned int bColor);
void ClearScreen2(unsigned int bColor);
void ClearWindow(unsigned int startX, unsigned int startY, unsigned int endX, unsigned int endY, unsigned int bColor);

void Draw_Circle(unsigned int x0,unsigned int y0,unsigned char r,unsigned int color);
void LCD_DrawRectangle(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2,unsigned int color);
void LCD_DrawLine(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2,unsigned int color);
void LCD_DrawPoint(unsigned int x,unsigned int y,unsigned int color);
void showzifustr(unsigned int x,unsigned int y,unsigned char *str,unsigned int dcolor,unsigned int bgcolor);
void showzifu(unsigned int x,unsigned int y,unsigned char value,unsigned int dcolor,unsigned int bgcolor);

void inversPicData1a(void);
void inversPicData1b(void);
void inversPicData1c(void);
void inversPicData1(void);
void inversPicData2(void);

void show_picture(void);

//void show_picture2(void);
void show_picture2a(uint16_t x,uint16_t y,uint16_t b,uint16_t h);
void show_picture2b(uint16_t x,uint16_t y,uint16_t b,uint16_t h);
void show_picture2c(uint16_t x,uint16_t y,uint16_t b,uint16_t h);
void show_picture3(void);

void show_picture4(void);


//unsigned int pic[];
//unsigned char String6_12[];

#define LCD_W 240
#define LCD_H 240

#define GC9A01_TFTWIDTH 240
#define GC9A01_TFTHEIGHT 240

#define GC9A01_RST_DELAY 120    ///< delay ms wait for reset finish
#define GC9A01_SLPIN_DELAY 120  ///< delay ms wait for sleep in finish
#define GC9A01_SLPOUT_DELAY 120 ///< delay ms wait for sleep out finish

#define GC9A01_NOP 0x00
#define GC9A01_SWRESET 0x01
#define GC9A01_RDDID 0x04
#define GC9A01_RDDST 0x09

#define GC9A01_SLPIN 0x10
#define GC9A01_SLPOUT 0x11
#define GC9A01_PTLON 0x12
#define GC9A01_NORON 0x13

#define GC9A01_INVOFF 0x20
#define GC9A01_INVON 0x21
#define GC9A01_DISPOFF 0x28
#define GC9A01_DISPON 0x29

#define GC9A01_CASET 0x2A
#define GC9A01_RASET 0x2B
#define GC9A01_RAMWR 0x2C
#define GC9A01_RAMRD 0x2E

#define GC9A01_PTLAR 0x30
#define GC9A01_COLMOD 0x3A
#define GC9A01_MADCTL 0x36

#define GC9A01_MADCTL_MY 0x80
#define GC9A01_MADCTL_MX 0x40
#define GC9A01_MADCTL_MV 0x20
#define GC9A01_MADCTL_ML 0x10
#define GC9A01_MADCTL_RGB 0x00

#define GC9A01_RDID1 0xDA
#define GC9A01_RDID2 0xDB
#define GC9A01_RDID3 0xDC
#define GC9A01_RDID4 0xDD



//predefine colors
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000
#define BLUE           	 0x001F
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 //brown
#define BRRED 			 0XFC07 //brown red
#define GRAY  			 0X8430 //gray
#define DARKBLUE      	 0X01CF	//dark blue
#define LIGHTBLUE      	 0X7D7C	//light blue
#define GRAYBLUE       	 0X5458 //gray blue
#define LIGHTGREEN     	 0X841F //light green
#define LGRAY 			 0XC618 //light gray
#define LGRAYBLUE        0XA651 //light gray blue
#define LBBLUE           0X2B12 //light brown blue

#define CBLUE 			 0x96be
#define CYELLOW  		 0xedc9
#define CORANGE  		 0xed23
#define CRED			 0xe2c3
#define CBLUE2 			 0x4bb1
#define CPINK 			 0xf69f
#endif /* INC_CG9A01_H_ */

/*
 * lcd.c
 *
 *  Created on: 29 SEP, 2022
 *  Author: wenqb
 */

#include "cg9a01.h"
#include "main.h"
//#include "spi.h"
#include "font.h"

extern SPI_HandleTypeDef hspi1;

#define LCD_RST_1 HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin,GPIO_PIN_SET)			// LCD_RST = 1 , LCD RESET pin
#define LCD_RST_0 HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin,GPIO_PIN_RESET)		// LCD_RST = 0 , LCD RESET pin

#define LCD_CS_1 HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin,GPIO_PIN_SET)			// LCD_CS = 1, LCD select pin
#define LCD_CS_0 HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin,GPIO_PIN_RESET)			// LCD_CS = 0, LCD select pin

#define LCD_DC_1 HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin,GPIO_PIN_SET)			// LCD_DC = 1, LCD Data/Command pin
#define LCD_DC_0 HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin,GPIO_PIN_RESET)			// LCD_DC = 0，LCD Data/Command pin

HAL_StatusTypeDef SPI_Write(uint8_t* pbuff, uint16_t size);
HAL_StatusTypeDef SPI_Read(uint8_t* pbuff, uint16_t size);

unsigned char String6_12[];
unsigned short pic1[];
unsigned short pic2[];



//===============================================================
//write parameter

void  Write_Cmd_Data (unsigned char CMDP)
{
    LCD_CS_0;
   	LCD_DC_1;

   	SPI_Write(&CMDP, 1);

   	LCD_CS_1;
}

//=============================================================
//write command

void Write_Cmd(unsigned char CMD)
{
    LCD_CS_0;
   	LCD_DC_0;

   	SPI_Write(&CMD, 1);

   	LCD_CS_1;
}

//==============================================================
//write  data word

void  Write_Data_U16(unsigned int y)
{
	unsigned char m,n;
	m=y>>8;
	n=y;
	Write_Data(m,n);
}


//===================================================================
//write data byte

void Write_Data(unsigned char DH,unsigned char DL)
{
    LCD_CS_0;
   	LCD_DC_1;

   	SPI_Write(&DH, 1);
   	SPI_Write(&DL, 1);

   	LCD_CS_1;
}


void Write_Bytes(unsigned char * pbuff, unsigned short size)
{
    LCD_CS_0;
   	LCD_DC_1;

   	SPI_Write(pbuff, size);

   	LCD_CS_1;
}


HAL_StatusTypeDef SPI_Write(uint8_t* pbuff, uint16_t size)
{
	//DMA, use HAL_SPI_Transmit_DMA() function
    HAL_StatusTypeDef status =  HAL_SPI_Transmit_DMA(&hspi1, pbuff, size);
    while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){;}
    return status;

    //no DMA, use HAL_SPI_Transmit() function
    //return HAL_SPI_Transmit(&hspi1, pbuff, size, 100);
}

HAL_StatusTypeDef SPI_Read(uint8_t* pbuff, uint16_t size)
{
	//DMA, use HAL_SPI_Receive_DMA() function
   HAL_StatusTypeDef status = HAL_SPI_Receive_DMA(&hspi1, pbuff, size);
   while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){;}
   return status;

   //no DMA, use HAL_SPI_Receive() function
   //return HAL_SPI_Receive(&hspi1, pbuff, size, 100);
}


//=============================================================
//LCD Initial

void GC9A01_Initial(void)
{
  	LCD_CS_1;
	HAL_Delay(5);
	LCD_RST_0;
	HAL_Delay(10);
	LCD_RST_1;
	HAL_Delay(120);


 //************* Start Initial Sequence **********//
	Write_Cmd(0xEF);

	Write_Cmd(0xEB);
	Write_Cmd_Data(0x14);

    Write_Cmd(0xFE);
	Write_Cmd(0xEF);

	Write_Cmd(0xEB);
	Write_Cmd_Data(0x14);

	Write_Cmd(0x84);
	Write_Cmd_Data(0x40);

	Write_Cmd(0x85);
	Write_Cmd_Data(0xFF);

	Write_Cmd(0x86);
	Write_Cmd_Data(0xFF);

	Write_Cmd(0x87);
	Write_Cmd_Data(0xFF);

	Write_Cmd(0x88);
	Write_Cmd_Data(0x0A);

	Write_Cmd(0x89);
	Write_Cmd_Data(0x21);

	Write_Cmd(0x8A);
	Write_Cmd_Data(0x00);

	Write_Cmd(0x8B);
	Write_Cmd_Data(0x80);

	Write_Cmd(0x8C);
	Write_Cmd_Data(0x01);

	Write_Cmd(0x8D);
	Write_Cmd_Data(0x01);

	Write_Cmd(0x8E);
	Write_Cmd_Data(0xFF);

	Write_Cmd(0x8F);
	Write_Cmd_Data(0xFF);


	Write_Cmd(0xB6);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0x00);

	Write_Cmd(0x36);

	if(USE_HORIZONTAL==0)Write_Cmd_Data(0x18);
	else if(USE_HORIZONTAL==1)Write_Cmd_Data(0x28);
	else if(USE_HORIZONTAL==2)Write_Cmd_Data(0x48);
	else Write_Cmd_Data(0x88);



	Write_Cmd(0x3A);
	Write_Cmd_Data(0x05);


	Write_Cmd(0x90);
	Write_Cmd_Data(0x08);
	Write_Cmd_Data(0x08);
	Write_Cmd_Data(0x08);
	Write_Cmd_Data(0x08);

	Write_Cmd(0xBD);
	Write_Cmd_Data(0x06);

	Write_Cmd(0xBC);
	Write_Cmd_Data(0x00);

	Write_Cmd(0xFF);
	Write_Cmd_Data(0x60);
	Write_Cmd_Data(0x01);
	Write_Cmd_Data(0x04);

	Write_Cmd(0xC3);
	Write_Cmd_Data(0x13);
	Write_Cmd(0xC4);
	Write_Cmd_Data(0x13);

	Write_Cmd(0xC9);
	Write_Cmd_Data(0x22);

	Write_Cmd(0xBE);
	Write_Cmd_Data(0x11);

	Write_Cmd(0xE1);
	Write_Cmd_Data(0x10);
	Write_Cmd_Data(0x0E);

	Write_Cmd(0xDF);
	Write_Cmd_Data(0x21);
	Write_Cmd_Data(0x0c);
	Write_Cmd_Data(0x02);

	Write_Cmd(0xF0);
    Write_Cmd_Data(0x45);
    Write_Cmd_Data(0x09);
 	Write_Cmd_Data(0x08);
  	Write_Cmd_Data(0x08);
 	Write_Cmd_Data(0x26);
 	Write_Cmd_Data(0x2A);

 	Write_Cmd(0xF1);
 	Write_Cmd_Data(0x43);
 	Write_Cmd_Data(0x70);
 	Write_Cmd_Data(0x72);
 	Write_Cmd_Data(0x36);
 	Write_Cmd_Data(0x37);
 	Write_Cmd_Data(0x6F);


 	Write_Cmd(0xF2);
 	Write_Cmd_Data(0x45);
 	Write_Cmd_Data(0x09);
 	Write_Cmd_Data(0x08);
 	Write_Cmd_Data(0x08);
 	Write_Cmd_Data(0x26);
 	Write_Cmd_Data(0x2A);

 	Write_Cmd(0xF3);
 	Write_Cmd_Data(0x43);
 	Write_Cmd_Data(0x70);
 	Write_Cmd_Data(0x72);
 	Write_Cmd_Data(0x36);
 	Write_Cmd_Data(0x37);
 	Write_Cmd_Data(0x6F);

	Write_Cmd(0xED);
	Write_Cmd_Data(0x1B);
	Write_Cmd_Data(0x0B);

	Write_Cmd(0xAE);
	Write_Cmd_Data(0x77);

	Write_Cmd(0xCD);
	Write_Cmd_Data(0x63);


	Write_Cmd(0x70);
	Write_Cmd_Data(0x07);
	Write_Cmd_Data(0x07);
	Write_Cmd_Data(0x04);
	Write_Cmd_Data(0x0E);
	Write_Cmd_Data(0x0F);
	Write_Cmd_Data(0x09);
	Write_Cmd_Data(0x07);
	Write_Cmd_Data(0x08);
	Write_Cmd_Data(0x03);

	Write_Cmd(0xE8);
	Write_Cmd_Data(0x34);

	Write_Cmd(0x62);
	Write_Cmd_Data(0x18);
	Write_Cmd_Data(0x0D);
	Write_Cmd_Data(0x71);
	Write_Cmd_Data(0xED);
	Write_Cmd_Data(0x70);
	Write_Cmd_Data(0x70);
	Write_Cmd_Data(0x18);
	Write_Cmd_Data(0x0F);
	Write_Cmd_Data(0x71);
	Write_Cmd_Data(0xEF);
	Write_Cmd_Data(0x70);
	Write_Cmd_Data(0x70);

	Write_Cmd(0x63);
	Write_Cmd_Data(0x18);
	Write_Cmd_Data(0x11);
	Write_Cmd_Data(0x71);
	Write_Cmd_Data(0xF1);
	Write_Cmd_Data(0x70);
	Write_Cmd_Data(0x70);
	Write_Cmd_Data(0x18);
	Write_Cmd_Data(0x13);
	Write_Cmd_Data(0x71);
	Write_Cmd_Data(0xF3);
	Write_Cmd_Data(0x70);
	Write_Cmd_Data(0x70);

	Write_Cmd(0x64);
	Write_Cmd_Data(0x28);
	Write_Cmd_Data(0x29);
	Write_Cmd_Data(0xF1);
	Write_Cmd_Data(0x01);
	Write_Cmd_Data(0xF1);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0x07);

	Write_Cmd(0x66);
	Write_Cmd_Data(0x3C);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0xCD);
	Write_Cmd_Data(0x67);
	Write_Cmd_Data(0x45);
	Write_Cmd_Data(0x45);
	Write_Cmd_Data(0x10);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0x00);

	Write_Cmd(0x67);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0x3C);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0x01);
	Write_Cmd_Data(0x54);
	Write_Cmd_Data(0x10);
	Write_Cmd_Data(0x32);
	Write_Cmd_Data(0x98);

	Write_Cmd(0x74);
	Write_Cmd_Data(0x10);
	Write_Cmd_Data(0x85);
	Write_Cmd_Data(0x80);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0x4E);
	Write_Cmd_Data(0x00);

    Write_Cmd(0x98);
	Write_Cmd_Data(0x3e);
	Write_Cmd_Data(0x07);

	Write_Cmd(0x35);
	Write_Cmd(0x21);

	Write_Cmd(0x11);
	HAL_Delay(120);
	Write_Cmd(0x29);
	HAL_Delay(20);
}


//===============================================================
//clear screen
//point to point clear , slowly
void ClearScreen(unsigned int bColor)
{
 unsigned int i,j;

 LCD_SetPos(0,0,GC9A01_TFTWIDTH-1,GC9A01_TFTHEIGHT-1);//240x240

 for (i=0;i<GC9A01_TFTWIDTH;i++)
 {
	 for (j=0;j<GC9A01_TFTHEIGHT;j++)
		 Write_Data_U16(bColor);
 }

}

//===============================================================
//clear screen2
//Use DMA，faster
void ClearScreen2(unsigned int bColor)
{
	ClearWindow(0,0,GC9A01_TFTWIDTH,GC9A01_TFTHEIGHT,bColor);
}

void ClearWindow(unsigned int startX, unsigned int startY, unsigned int endX, unsigned int endY, unsigned int bColor)
{
 unsigned int i;

 //Exchange high 8bit and low 8bit of bColor for DMA batch transmit
 unsigned char hb = (bColor&0xFFFF) >> 8;
 unsigned char lb = bColor & 0xFF;
 unsigned short tempColor = lb * 256 + hb;

 unsigned int totalSize = (endX-startX) * (endY - startY) * 2; // total clear window data size
 unsigned int bufSize = 512;  // define bufSize, need less than DMA transmit size

 unsigned int loopNum = (totalSize - (totalSize % bufSize)) / bufSize; // transmit loop times
 unsigned int modNum = totalSize % bufSize;  // remainder data bytes


 //use a tempBuf to initial bColor data, bufSize < DMA transmit size
 unsigned short tempBuf[bufSize];
 unsigned char * ptempBuf;

 //init tempBuf data to tempColor( Exchange high 8bit and low 8bit of bColor )
 for(i=0; i<bufSize; i++){
	 tempBuf[i] = tempColor;
 }

 // Clear window size: from (startX, startY) to (endX, endY)
 LCD_SetPos(startX,startY,endX-1,endY-1);// (endX-startX) * (endY - startY)

 // transmit bufSize byte one time, loopNum loops
 ptempBuf = (unsigned char *)tempBuf;
 for(i=0; i<loopNum; i++){
	 Write_Bytes(ptempBuf, bufSize);
 }

 // transmit remainder data, modNum bytes
 Write_Bytes(ptempBuf, modNum);

}


//===============================================================
 void LCD_SetPos(unsigned int Xstart,unsigned int Ystart,unsigned int Xend,unsigned int Yend)
{
	Write_Cmd(0x2a);
	Write_Cmd_Data(Xstart>>8);
	Write_Cmd_Data(Xstart);
 	Write_Cmd_Data(Xend>>8);
	Write_Cmd_Data(Xend);

	Write_Cmd(0x2b);
	Write_Cmd_Data(Ystart>>8);
	Write_Cmd_Data(Ystart);
	Write_Cmd_Data(Yend>>8);
	Write_Cmd_Data(Yend);

  	Write_Cmd(0x2c);//LCD_WriteCMD(GRAMWR);
}


 //show one Character
 void showzifu(unsigned int x,unsigned int y,unsigned char value,unsigned int dcolor,unsigned int bgcolor)
 {
 	unsigned char i,j;
 	unsigned char *temp=String6_12;

    LCD_SetPos(x,y,x+7,y+11);
 	temp+=(value-32)*12;
 	for(j=0;j<12;j++)
 	{
 		for(i=0;i<8;i++)
 		{
 		 	if((*temp&(1<<(7-i)))!=0)
 			{
 				Write_Data(dcolor>>8,dcolor);
 			}
 			else
 			{
 				Write_Data(bgcolor>>8,bgcolor);
 			}
 		}
 		temp++;
 	 }
 }

 //show String
 void showzifustr(unsigned int x,unsigned int y,unsigned char *str,unsigned int dcolor,unsigned int bgcolor)
 {
 	unsigned int x1,y1;
 	x1=x;
 	y1=y;
 	while(*str!='\0')
 	{
 		showzifu(x1,y1,*str,dcolor,bgcolor);
 		x1+=7;
 		str++;
 	}
 }


 void LCD_DrawPoint(unsigned int x,unsigned int y,unsigned int color)
 {
 	LCD_SetPos(x,y,x,y);
 	Write_Data_U16(color);
 }


 void LCD_DrawLine(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2,unsigned int color)
 {
 	unsigned int t;
 	int xerr=0,yerr=0,delta_x,delta_y,distance;
 	int incx,incy,uRow,uCol;
 	delta_x=x2-x1;
 	delta_y=y2-y1;
 	uRow=x1;
 	uCol=y1;
 	if(delta_x>0)incx=1;
 	else if (delta_x==0)incx=0;
 	else {incx=-1;delta_x=-delta_x;}
 	if(delta_y>0)incy=1;
 	else if (delta_y==0)incy=0;
 	else {incy=-1;delta_y=-delta_x;}
 	if(delta_x>delta_y)distance=delta_x;
 	else distance=delta_y;
 	for(t=0;t<distance+1;t++)
 	{
 		LCD_DrawPoint(uRow,uCol,color);
 		xerr+=delta_x;
 		yerr+=delta_y;
 		if(xerr>distance)
 		{
 			xerr-=distance;
 			uRow+=incx;
 		}
 		if(yerr>distance)
 		{
 			yerr-=distance;
 			uCol+=incy;
 		}
 	}
 }


 void LCD_DrawRectangle(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2,unsigned int color)
 {
 	LCD_DrawLine(x1,y1,x2,y1,color);
 	LCD_DrawLine(x1,y1,x1,y2,color);
 	LCD_DrawLine(x1,y2,x2,y2,color);
 	LCD_DrawLine(x2,y1,x2,y2,color);
 }


 void Draw_Circle(unsigned int x0,unsigned int y0,unsigned char r,unsigned int color)
 {
 	int a,b;
 	a=0;b=r;
 	while(a<=b)
 	{
 		LCD_DrawPoint(x0-b,y0-a,color);             //3
 		LCD_DrawPoint(x0+b,y0-a,color);             //0
 		LCD_DrawPoint(x0-a,y0+b,color);             //1
 		LCD_DrawPoint(x0-a,y0-b,color);             //2
 		LCD_DrawPoint(x0+b,y0+a,color);             //4
 		LCD_DrawPoint(x0+a,y0-b,color);             //5
 		LCD_DrawPoint(x0+a,y0+b,color);             //6
 		LCD_DrawPoint(x0-b,y0+a,color);             //7
 		a++;
 		if((a*a+b*b)>(r*r))
 		{
 			b--;
 		}
 	}
 }


 //============================================================
 //show gray

 void DispRGBGray(void)
 {

 	unsigned int  A,B,C;
     unsigned int  i,j,k,DH;

     LCD_SetPos(0,0,GC9A01_TFTWIDTH-1,GC9A01_TFTHEIGHT-1);//240x240

 	for(k=0;k<60;k++)
 	{
     	A=0;
     	B=0;
     	C=0;
 		for(i=0;i<16;i++)
 		{
     		for(j=0;j<15;j++)
     		{
      			DH=(A<<11)+(B<<5)+C;
      			Write_Data_U16(DH);
     		}
     		A=A+2;
    			B=B+4;
     		C=C+2;
   		}
 	}

 	for(k=0;k<60;k++)
 	{
     	A=0;
     	B=0;
     	C=0;
 		for(i=0;i<16;i++)
 		{
     		for(j=0;j<15;j++)
     		{
      			DH=(A<<11)+B+C;
      			Write_Data_U16(DH);
     		}
     		A=A+2;
   		}
 	}

 	for(k=0;k<60;k++)
 	{
     	A=0;
     	B=0;
     	C=0;
 		for(i=0;i<16;i++)
 		{
     		for(j=0;j<15;j++)
     		{
      			DH=A+(B<<5)+C;
      			Write_Data_U16(DH);
     		}
     		B=B+4;
   		}
 	}

 	for(k=0;k<60;k++)
 	{
     	A=0;
     	B=0;
     	C=0;
 		for(i=0;i<16;i++)
 		{
     		for(j=0;j<15;j++)
     		{
      			DH=A+B+C;
      			Write_Data_U16(DH);
     		}
     		C=C+2;
   		}
 	}
 }




 //============================================================
 //show picture
 //point to point show picture data, slowly
 void show_picture(void)
 {
 	unsigned char i,j;
 	unsigned int n=0;


    LCD_SetPos(60,40,179,199);

 	for(j=0;j<160;j++)
 	{
 		for(i=0;i<120;i++)
 		{
             Write_Data_U16(pic[n++]);
 	    }
 	}

     return;
 }


 //use DMA batch transmit, faster
 void show_picture2a(uint16_t x,uint16_t y,uint16_t b,uint16_t h)
 {
	 int i;
	 unsigned char * pPic;
	 unsigned int totalSize = 208*60*2;
	 unsigned int bufSize = 512; //bufSize, need less than DMA transmit data size

 	 unsigned int loopNum = (totalSize - (totalSize % bufSize)) / bufSize;
 	 unsigned int modNum = totalSize % bufSize;


    LCD_SetPos(x,y,(x+b)-1,(y+h)-1);

    //Exchange high 8bit and low 8bit of every point, for picture DMA transmit

    pPic = (unsigned char *)pic1;

    //transmit bufSize bytes of picture data , loop loopNum times
    for(i=0; i<loopNum; i++){
    	Write_Bytes(pPic+i*bufSize, bufSize);
    }
    //transmit remainder data of picture
    Write_Bytes(pPic+i*bufSize, modNum);

    //Exchange high 8bit and low 8bit of every point, recover picture data
    inversPicData1a();

    return;
 }
 void show_picture2b(uint16_t x,uint16_t y,uint16_t b,uint16_t h)
 {
	 int i;
	 unsigned char * pPic;
	 unsigned int totalSize = 208*60*2;
	 unsigned int bufSize = 512; //bufSize, need less than DMA transmit data size

 	 unsigned int loopNum = (totalSize - (totalSize % bufSize)) / bufSize;
 	 unsigned int modNum = totalSize % bufSize;


    LCD_SetPos(x,y,(x+b)-1,(y+h)-1);

    //Exchange high 8bit and low 8bit of every point, for picture DMA transmit

    pPic = (unsigned char *)pic2;

    //transmit bufSize bytes of picture data , loop loopNum times
    for(i=0; i<loopNum; i++){
    	Write_Bytes(pPic+i*bufSize, bufSize);
    }
    //transmit remainder data of picture
    Write_Bytes(pPic+i*bufSize, modNum);

    //Exchange high 8bit and low 8bit of every point, recover picture data
    inversPicData1b();

    return;
 }
 void show_picture2c(uint16_t x,uint16_t y,uint16_t b,uint16_t h)
 {
	 int i;
	 unsigned char * pPic;
	 unsigned int totalSize = 40*40*2;
	 unsigned int bufSize = 512; //bufSize, need less than DMA transmit data size

 	 unsigned int loopNum = (totalSize - (totalSize % bufSize)) / bufSize;
 	 unsigned int modNum = totalSize % bufSize;


    LCD_SetPos(x,y,(x+b)-1,(y+h)-1);

    //Exchange high 8bit and low 8bit of every point, for picture DMA transmit

    pPic = (unsigned char *)pic3;

    //transmit bufSize bytes of picture data , loop loopNum times
    for(i=0; i<loopNum; i++){
    	Write_Bytes(pPic+i*bufSize, bufSize);
    }
    //transmit remainder data of picture
    Write_Bytes(pPic+i*bufSize, modNum);

    //Exchange high 8bit and low 8bit of every point, recover picture data
    inversPicData1c();

    return;
 }
 void show_picture3(void)
  {
 	 int i;
 	 unsigned char * pPic;
 	 unsigned int totalSize = 240*240*2;
 	 unsigned int bufSize = 512; //bufSize, need less than DMA transmit data size

  	 unsigned int loopNum = (totalSize - (totalSize % bufSize)) / bufSize;
  	 unsigned int modNum = totalSize % bufSize;


     LCD_SetPos(0,0,239,239);

     //Exchange high 8bit and low 8bit of every point, for picture DMA transmit
     inversPicData1();

     pPic = (unsigned char *)pic;

     //transmit bufSize bytes of picture data , loop loopNum times
     for(i=0; i<loopNum; i++){
     	Write_Bytes(pPic+i*bufSize, bufSize);
     }
     //transmit remainder data of picture
     Write_Bytes(pPic+i*bufSize, modNum);

     //Exchange high 8bit and low 8bit of every point, recover picture data
     inversPicData1();

     return;
  }

// void show_picture4(void)
//   {
//  	 int i;
//  	 unsigned char * pPic;
//  	 unsigned int totalSize = 240*240*2;
//  	 unsigned int bufSize = 512; //bufSize, need less than DMA transmit data size
//
//   	 unsigned int loopNum = (totalSize - (totalSize % bufSize)) / bufSize;
//   	 unsigned int modNum = totalSize % bufSize;
//
//
//      LCD_SetPos(0,0,239,239);
//
//      //Exchange high 8bit and low 8bit of every point, for picture DMA transmit
//      inversPicData2();
//
//      pPic = (unsigned char *)pic2;
//
//      //transmit bufSize bytes of picture data , loop loopNum times
//      for(i=0; i<loopNum; i++){
//      	Write_Bytes(pPic+i*bufSize, bufSize);
//      }
//      //transmit remainder data of picture
//      Write_Bytes(pPic+i*bufSize, modNum);
//
//      //Exchange high 8bit and low 8bit of every point, recover picture data
//      inversPicData2();
//
//      return;
//   }


// Exchange high 8bit and low 8bit of picture data, for DMA transmit
 void inversPicData1a(void){
 	int i;
 	unsigned char hb, lb;

 	for(i=0; i<208*60; i++){  			//160*120 is the size of picture
 			hb = pic1[i] >> 8;  			//high 8 bit
 			lb = pic1[i] & 0xFF; 		//low 8 bit
 			pic1[i] = lb * 256 + hb; 	//exchange high 8 bit and low 8 bit
 	}
 }
 void inversPicData1b(void){
 	int i;
 	unsigned char hb, lb;

 	for(i=0; i<208*60; i++){  			//160*120 is the size of picture
 			hb = pic2[i] >> 8;  			//high 8 bit
 			lb = pic2[i] & 0xFF; 		//low 8 bit
 			pic2[i] = lb * 256 + hb; 	//exchange high 8 bit and low 8 bit
 	}
 }
 void inversPicData1c(void){
 	int i;
 	unsigned char hb, lb;

 	for(i=0; i<40*40; i++){  			//160*120 is the size of picture
 			hb = pic3[i] >> 8;  			//high 8 bit
 			lb = pic3[i] & 0xFF; 		//low 8 bit
 			pic3[i] = lb * 256 + hb; 	//exchange high 8 bit and low 8 bit
 	}
 }
 void inversPicData1(void){
 	int i;
 	unsigned char hb, lb;

 	for(i=0; i<240*240; i++){  			//160*120 is the size of picture
 			hb = pic[i] >> 8;  			//high 8 bit
 			lb = pic[i] & 0xFF; 		//low 8 bit
 			pic[i] = lb * 256 + hb; 	//exchange high 8 bit and low 8 bit
 	}
 }

 void inversPicData2(void){
 	int i;
 	unsigned char hb, lb;

 	for(i=0; i<240*240; i++){  			//160*120 is the size of picture
 			hb = pic[i] >> 8;  			//high 8 bit
 			lb = pic[i] & 0xFF; 		//low 8 bit
 			pic[i] = lb * 256 + hb; 	//exchange high 8 bit and low 8 bit
 	}
 }

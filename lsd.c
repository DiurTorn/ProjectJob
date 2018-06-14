#include "lsd.h"
//---------------------------------------------------
void delay(void)
{
	uint16_t i;
  for(i=0;i<1000;i++)
	{
		
	}
}
//---------------------------------------------------
void LCD_WriteData(uint8_t dt)
{
	if(((dt>>3)&0x01)==1) {d7_set();}else {d7_reset();}
	if(((dt>>2)&0x01)==1) {d6_set();}else {d6_reset();}
	if(((dt>>1)&0x01)==1) {d5_set();}else {d5_reset();}
	if((dt&0x01)==1) {d4_set();}else {d4_reset();}
}
//---------------------------------------------------
void LCD_Command(uint8_t dt){
	rs0;
	LCD_WriteData(dt>>4);
	e1;
	delay();
	e0;
	LCD_WriteData(dt);
	e1;
	delay();
	e0;
	
}
//---------------------------------------------------
void LCD_Data(uint8_t dt){
	rs1;
	LCD_WriteData(dt>>4);
	e1;
	delay();
	e0;
	LCD_WriteData(dt);
	e1;
	delay();
	e0;
	
}
//---------------------------------------------------
void LCD_ini(void)
{
	HAL_Delay(40);
	rs0;
	LCD_WriteData(3);
	e1;
	delay();
	e0;
	HAL_Delay(1);
	LCD_WriteData(3);
	e1;
	delay();
	e0;
	HAL_Delay(1);
	LCD_WriteData(3);
	e1;
	delay();
	e0;
	HAL_Delay(1);
	LCD_Command(0x28);
	HAL_Delay(1);
	LCD_Command(0x28);
	HAL_Delay(1);
	LCD_Command(0x0F);
	HAL_Delay(1);
	LCD_Command(0x01);
	HAL_Delay(2);
	LCD_Command(0x06);
	HAL_Delay(1);
	LCD_Command(0x02);
	HAL_Delay(2);
}
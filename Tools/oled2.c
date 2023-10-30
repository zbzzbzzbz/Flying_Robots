#include "includes.h"
#include "OLED_Font2.h"

//硬件i2c
//写数据
void OLED_Writedata2(unsigned char I2C_Data)
{
	I2C_Sand_Byte(0x78,0x40,I2C_Data);
}
//写命令
void OLED_WriteCommand2(unsigned char I2C_Command)
{
	I2C_Sand_Byte(0x78,0x00,I2C_Command);
}

//设置光标位置，以左上角为原点
void OLED_SetCursor2(uint8_t Y, uint8_t X)
{
	OLED_WriteCommand2(0xB0 | Y);					//设置Y位置
	OLED_WriteCommand2(0x10 | ((X & 0xF0) >> 4));	//设置X位置低4位
	OLED_WriteCommand2(0x00 | (X & 0x0F));			//设置X位置高4位
}


void OLED_Clear2(void)
{  
	uint8_t i, j;
	for (j = 0; j < 8; j++)
	{
		OLED_SetCursor2(j, 0);
		for(i = 0; i < 128; i++)
		{
			OLED_Writedata2(0x00);
		}
	}
}


/**
  * @brief  OLED显示一个字符
  * @param  Line 行位置，范围：1~4
  * @param  Column 列位置，范围：1~16
  * @param  Char 要显示的一个字符，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowChar2(uint8_t Line, uint8_t Column, char Char)
{      	
	
	uint8_t i;
	OLED_SetCursor2((Line - 1) * 2, (Column - 1) * 8);		//设置光标位置在上半部分
	for (i = 0; i < 8; i++)
	{
		OLED_Writedata2(OLED_F8x162[Char - ' '][i]);			//显示上半部分内容
	}
	OLED_SetCursor2((Line - 1) * 2 + 1, (Column - 1) * 8);	//设置光标位置在下半部分
	for (i = 0; i < 8; i++)
	{
		OLED_Writedata2(OLED_F8x162[Char - ' '][i + 8]);		//显示下半部分内容
	}
}

/**
  * @brief  OLED次方函数
  * @retval 返回值等于X的Y次方
  */
uint32_t OLED_Pow2(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

void OLED_ShowNum2(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar2(Line, Column + i, Number / OLED_Pow2(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * @brief  OLED显示数字（十六进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~0xFFFFFFFF
  * @param  Length 要显示数字的长度，范围：1~8
  * @retval 无
  */
void OLED_ShowHexNum2(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)							
	{
		SingleNumber = Number / OLED_Pow2(16, Length - i - 1) % 16;
		if (SingleNumber < 10)
		{
			OLED_ShowChar2(Line, Column + i, SingleNumber + '0');
		}
		else
		{
			OLED_ShowChar2(Line, Column + i, SingleNumber - 10 + 'A');
		}
	}
}

/**
  * @brief  OLED显示数字（十进制，带符号数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：-2147483648~2147483647
  * @param  Length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void OLED_ShowSignedNum2(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		OLED_ShowChar2(Line, Column, '+');
		Number1 = Number;
	}
	else
	{
		OLED_ShowChar2(Line, Column, '-');
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar2(Line, Column + i + 1, Number1 / OLED_Pow2(10, Length - i - 1) % 10 + '0');
	}
}

//z_len为整数显示位数，f_len为小数显示位数，size2为字体大小
void OLED_Showdecimal2(uint8_t x,uint8_t y,float num,uint8_t z_len,uint8_t f_len,uint8_t size2)
{         	
	uint8_t t,temp;
	uint8_t enshow;
	int z_temp,f_temp;      
	z_temp=(int)num;
	//整数部分
	for(t=0;t<z_len;t++)
	{
		temp=(z_temp/OLED_Pow2(10,z_len-t-1))%10;
		if(enshow==0 && t<(z_len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar2(x+(size2/2)*t,y,' ');
				continue;
			}
			else
			enshow=1;
		}
		OLED_ShowChar2(x+(size2/2)*t,y,temp+'0'); 
	}
	//小数点
	OLED_ShowChar2(x+(size2/2)*(z_len),y,'.'); 
	
	f_temp=(int)((num-z_temp)*(OLED_Pow2(10,f_len)));
  //小数部分
	for(t=0;t<f_len;t++)
	{
		temp=(f_temp/OLED_Pow2(10,f_len-t-1))%10;
		OLED_ShowChar2(x+(size2/2)*(t+z_len)+5,y,temp+'0'); 
	}
}


void OLED_Init2(){
	uint32_t i, j;
	
	for (i = 0; i < 100; i++)			//上电延时
	{
		for (j = 0; j < 100; j++);
	}
	
	
	OLED_WriteCommand2(0xAE);	//关闭显示
	
	OLED_WriteCommand2(0xD5);	//设置显示时钟分频比/振荡器频率
	OLED_WriteCommand2(0x80);
	OLED_WriteCommand2(0xA8);	//设置多路复用率
	OLED_WriteCommand2(0x3F);
	
	OLED_WriteCommand2(0xD3);	//设置显示偏移
	OLED_WriteCommand2(0x00);
	
	OLED_WriteCommand2(0x40);	//设置显示开始行
	
	OLED_WriteCommand2(0xA1);	//设置左右方向，0xA1正常 0xA0左右反置
	
	OLED_WriteCommand2(0xC8);	//设置上下方向，0xC8正常 0xC0上下反置

	OLED_WriteCommand2(0xDA);	//设置COM引脚硬件配置
	OLED_WriteCommand2(0x12);
	
	OLED_WriteCommand2(0x81);	//设置对比度控制
	OLED_WriteCommand2(0xCF);

	OLED_WriteCommand2(0xD9);	//设置预充电周期
	OLED_WriteCommand2(0xF1);

	OLED_WriteCommand2(0xDB);	//设置VCOMH取消选择级别
	OLED_WriteCommand2(0x30);

	OLED_WriteCommand2(0xA4);	//设置整个显示打开/关闭

	OLED_WriteCommand2(0xA6);	//设置正常/倒转显示

	OLED_WriteCommand2(0x8D);	//设置充电泵
	OLED_WriteCommand2(0x14);

	OLED_WriteCommand2(0xAF);	//开启显示
	OLED_Clear2();				//OLED清屏
}



#include "config.h"

#ifdef LONGAN_NANO

#include "lcd/lcd.h"
#include "lcd/oledfont.h"
#include "lcd/bmp.h"
u16 BACK_COLOR;   // Background colour

/******************************************************************************
      Function description: send 8 bit byte to the LCD using the SPI bus
	  Parameters:    
	    dat: 8 bit data
	  Return value: none
******************************************************************************/
void LCD_Writ_Bus(u8 dat) 
{
#if SPI0_CFG == 1
	OLED_CS_Clr();

	while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE));
        spi_i2s_data_transmit(SPI0, dat);
	while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE));
        spi_i2s_data_receive(SPI0);

	OLED_CS_Set();
#elif SPI0_CFG == 2
	spi_dma_enable(SPI0, SPI_DMA_TRANSMIT);
#else
	u8 i;
	OLED_CS_Clr();
	for(i=0;i<8;i++)
	{			  
		OLED_SCLK_Clr();
		if(dat&0x80)
		   OLED_SDIN_Set();
		else
		   OLED_SDIN_Clr();
		OLED_SCLK_Set();
		dat<<=1;
	}	
  OLED_CS_Set();	
#endif
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
/*****************************************************************************
      Function description: display 8 bit byte on the LCD
      Parameter: 
	    dat: byte to write
      Return value: none
******************************************************************************/
void LCD_WR_DATA8(u8 dat)
{
	OLED_DC_Set();//写数据  // set display in data mode
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
/******************************************************************************
      Function description: display 16 bit word to the LCD
	  Parameter: 
        dat: word to write
      Return value: None
******************************************************************************/
void LCD_WR_DATA(u16 dat)
{
	OLED_DC_Set();//写数据 // set display in data mode
	LCD_Writ_Bus(dat>>8);
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入命令
      入口数据：dat 写入的命令
      返回值：  无
******************************************************************************/
/*****************************************************************************
      Function description: write 8 bit command to LCD write command
      Parameter: 
	    dat: 8 bit command 
      Return value: None
******************************************************************************/
void LCD_WR_REG(u8 dat)
{
	OLED_DC_Clr();//写命令 // set display in command mode
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
******************************************************************************/
/*****************************************************************************
      Function description: Set start and end addresses
      Parameters: 
	    x1, x2: column start and end addresses
        y1, y2: line start and end addresses
      Return value: None
******************************************************************************/
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2)
{
	if(USE_HORIZONTAL==0)
	{
		LCD_WR_REG(0x2a);//列地址设置  // set column address
		LCD_WR_DATA(x1+26);
		LCD_WR_DATA(x2+26);
		LCD_WR_REG(0x2b);//行地址设置  // set row address
		LCD_WR_DATA(y1+1);
		LCD_WR_DATA(y2+1);
		LCD_WR_REG(0x2c);//储存器写  // memory write
	}
	else if(USE_HORIZONTAL==1)
	{
		LCD_WR_REG(0x2a);//列地址设置  // set column address
		LCD_WR_DATA(x1+26);
		LCD_WR_DATA(x2+26);
		LCD_WR_REG(0x2b);//行地址设置  // set row address
		LCD_WR_DATA(y1+1);
		LCD_WR_DATA(y2+1);
		LCD_WR_REG(0x2c);//储存器写  // memory write
	}
	else if(USE_HORIZONTAL==2)
	{
		LCD_WR_REG(0x2a);//列地址设置  // set column address
		LCD_WR_DATA(x1+1);
		LCD_WR_DATA(x2+1);
		LCD_WR_REG(0x2b);//行地址设置  // set row address
		LCD_WR_DATA(y1+26);
		LCD_WR_DATA(y2+26);
		LCD_WR_REG(0x2c);//储存器写  // memory write
	}
	else
	{
		LCD_WR_REG(0x2a);//列地址设置  // set column address
		LCD_WR_DATA(x1+1);
		LCD_WR_DATA(x2+1);
		LCD_WR_REG(0x2b);//行地址设置  // set row address
		LCD_WR_DATA(y1+26);
		LCD_WR_DATA(y2+26);
		LCD_WR_REG(0x2c);//储存器写  // memory write
	}
}

#if SPI0_CFG == 2
/*!
    \brief      configure the DMA peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void lcd_dma_config(void)
{
	dma_parameter_struct dma_init_struct;

    /* SPI0 transmit dma config:DMA0,DMA_CH2 */
    dma_deinit(DMA0, DMA_CH2);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.periph_addr  = (uint32_t)&SPI_DATA(SPI0);
    dma_init_struct.memory_addr  = (uint32_t)image;
    dma_init_struct.direction    = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority     = DMA_PRIORITY_LOW;
    dma_init_struct.number       = FRAME_SIZE;
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init(DMA0, DMA_CH2, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH2);
    dma_memory_to_memory_disable(DMA0, DMA_CH2);
}
#endif

#if SPI0_CFG == 1
/*!
    \brief      configure the SPI peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void spi_config(void)
{
    spi_parameter_struct spi_init_struct;
    /* deinitilize SPI and the parameters */
    OLED_CS_Set();
    spi_struct_para_init(&spi_init_struct);

    /* SPI0 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    // spi_init_struct.prescale             = SPI_PSC_8;		// 108/8 = 13.5 MHz
    spi_init_struct.prescale             = SPI_PSC_2;	// 54 MHz (!)
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI0, &spi_init_struct);

	spi_crc_polynomial_set(SPI0,7);
	spi_enable(SPI0);
}
#endif

/******************************************************************************
      函数说明：LCD初始化函数
      入口数据：无
      返回值：  无
******************************************************************************/
/*****************************************************************************
      Function description: initialize display
      Parameter: None
      Return value: None
******************************************************************************/
void Lcd_Init(void)
{
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOB);

#if SPI0_CFG == 1
 	rcu_periph_clock_enable(RCU_AF);
	rcu_periph_clock_enable(RCU_SPI0);
	/* SPI0 GPIO config: NSS/PA4, SCK/PA5, MOSI/PA7 */	// XXX doesn't match below
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5 |GPIO_PIN_6| GPIO_PIN_7);
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);

	spi_config();

#elif SPI0_CFG == 2
    rcu_periph_clock_enable(RCU_DMA0);
    rcu_periph_clock_enable(RCU_SPI0);

	/* SPI0 GPIO config: NSS/PA4, SCK/PA5, MOSI/PA7 */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7);
    /* SPI0 GPIO config: MISO/PA6 */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

	lcd_dma_config();

	dma_channel_enable(DMA0,DMA_CH2);
#elif SPI0_CFG == 3
	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5 | GPIO_PIN_7);
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);

	gpio_bit_reset(GPIOA, GPIO_PIN_5 | GPIO_PIN_7);
	gpio_bit_reset(GPIOB, GPIO_PIN_2);
#endif

	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1);
	gpio_bit_reset(GPIOB, GPIO_PIN_0 | GPIO_PIN_1);

	OLED_RST_Clr();
	delay(200);
	OLED_RST_Set();
	delay(20);
	OLED_BLK_Set();

	LCD_WR_REG(0x11);	// turn off sleep mode
	delay(100);

	LCD_WR_REG(0x21);	// display inversion mode

	LCD_WR_REG(0xB1);	// Set the frame frequency of the full colors normal mode
						// Frame rate=fosc/((RTNA x 2 + 40) x (LINE + FPA + BPA +2))
						// fosc = 850kHz
	LCD_WR_DATA8(0x05);	// RTNA
	LCD_WR_DATA8(0x3A);	// FPA
	LCD_WR_DATA8(0x3A);	// BPA

	LCD_WR_REG(0xB2);	// Set the frame frequency of the Idle mode
						// Frame rate=fosc/((RTNB x 2 + 40) x (LINE + FPB + BPB +2))
						// fosc = 850kHz
	LCD_WR_DATA8(0x05);	// RTNB
	LCD_WR_DATA8(0x3A);	// FPB
	LCD_WR_DATA8(0x3A);	// BPB

	LCD_WR_REG(0xB3);	// Set the frame frequency of the Partial mode/ full colors
	LCD_WR_DATA8(0x05);  
	LCD_WR_DATA8(0x3A);
	LCD_WR_DATA8(0x3A);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3A);
	LCD_WR_DATA8(0x3A);

	LCD_WR_REG(0xB4);
	LCD_WR_DATA8(0x03);

	LCD_WR_REG(0xC0);
	LCD_WR_DATA8(0x62);
	LCD_WR_DATA8(0x02);
	LCD_WR_DATA8(0x04);

	LCD_WR_REG(0xC1);
	LCD_WR_DATA8(0xC0);

	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x00);

	LCD_WR_REG(0xC3);
	LCD_WR_DATA8(0x8D);
	LCD_WR_DATA8(0x6A);   

	LCD_WR_REG(0xC4);
	LCD_WR_DATA8(0x8D); 
	LCD_WR_DATA8(0xEE); 

	LCD_WR_REG(0xC5);  /*VCOM*/
	LCD_WR_DATA8(0x0E);    

	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0x10);
	LCD_WR_DATA8(0x0E);
	LCD_WR_DATA8(0x02);
	LCD_WR_DATA8(0x03);
	LCD_WR_DATA8(0x0E);
	LCD_WR_DATA8(0x07);
	LCD_WR_DATA8(0x02);
	LCD_WR_DATA8(0x07);
	LCD_WR_DATA8(0x0A);
	LCD_WR_DATA8(0x12);
	LCD_WR_DATA8(0x27);
	LCD_WR_DATA8(0x37);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x0E);
	LCD_WR_DATA8(0x10);

	LCD_WR_REG(0xE1);
	LCD_WR_DATA8(0x10);
	LCD_WR_DATA8(0x0E);
	LCD_WR_DATA8(0x03);
	LCD_WR_DATA8(0x03);
	LCD_WR_DATA8(0x0F);
	LCD_WR_DATA8(0x06);
	LCD_WR_DATA8(0x02);
	LCD_WR_DATA8(0x08);
	LCD_WR_DATA8(0x0A);
	LCD_WR_DATA8(0x13);
	LCD_WR_DATA8(0x26);
	LCD_WR_DATA8(0x36);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x0E);
	LCD_WR_DATA8(0x10);

	LCD_WR_REG(0x3A);	// define the format of RGB picture data
	LCD_WR_DATA8(0x05);	// 16-bit/pixel

	LCD_WR_REG(0x36);
	if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x08);
	else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC8);
	else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x78);
	else LCD_WR_DATA8(0xA8);

	LCD_WR_REG(0x29);	// Display On
}

/******************************************************************************
      函数说明：LCD清屏函数
      入口数据：无
      返回值：  无
******************************************************************************/
/*****************************************************************************
      Function description: clear display
      Parameter: 
	    Color: background colour
      Return value: None
******************************************************************************/
void LCD_Clear(u16 Color)
{
	u16 i,j;  	
	LCD_Address_Set(0,0,LCD_W-1,LCD_H-1);
    for(i=0;i<LCD_W;i++)
	  {
			for (j=0;j<LCD_H;j++)
				{
					LCD_WR_DATA(Color);
				}
	  }
}


/******************************************************************************
      函数说明：LCD显示汉字
      入口数据：x,y   起始坐标
                index 汉字的序号
                size  字号
      返回值：  无
******************************************************************************/
/*****************************************************************************
      Function description: Display Chinese characters
	  Parameters:
         x: start column,
		 y: start row
         index: Chinese character number
         size: font size (16 or 32)
		 color: 16 bit colour
      Return value: None
******************************************************************************/
void LCD_ShowChinese(u16 x,u16 y,u8 index,u8 size,u16 color)	
{  
	u8 i,j;
	u8 *temp,size1;
	if(size==16){temp=Hzk16;}//选择字号  // choose a font based on the size
	if(size==32){temp=Hzk32;}
  LCD_Address_Set(x,y,x+size-1,y+size-1); //设置一个汉字的区域  // area of the Chinese character
  size1=size*size/8;//一个汉字所占的字节  // size of the Chinese character in bytes
	temp+=index*size1;//写入的起始位置 // starting position
	for(j=0;j<size1;j++)
	{
		for(i=0;i<8;i++)
		{
		 	if((*temp&(1<<i))!=0)//从数据的低位开始读 // read from the bottom bit of the data
			{
				LCD_WR_DATA(color);//点亮  // turn pixel on
			}
			else
			{
				LCD_WR_DATA(BACK_COLOR);//不点亮 // turn pixel off
			}
		}
		temp++;
	 }
}


/******************************************************************************
      函数说明：LCD显示汉字
      入口数据：x,y   起始坐标
      返回值：  无
******************************************************************************/
/*****************************************************************************
      Function description: draw a pixel on the display
      Parameters: 
	    x: column position
		y: row position
		color: 16 bit colour of pixel
      Return value: None
******************************************************************************/
void LCD_DrawPoint(u16 x,u16 y,u16 color)
{
	LCD_Address_Set(x,y,x,y);//设置光标位置  // set the cursor position
	LCD_WR_DATA(color);
} 


/******************************************************************************
      函数说明：LCD画一个大的点
      入口数据：x,y   起始坐标
      返回值：  无
******************************************************************************/
/*****************************************************************************
      Function description: draw a large, 3x3 pixel, dot on the display
      Parameters:
	    x: column position of the center of the dot
		y: row position of the center of the dot
      Return value: None
******************************************************************************/
void LCD_DrawPoint_big(u16 x,u16 y,u16 color)
{
	LCD_Fill(x-1,y-1,x+1,y+1,color);
} 


/******************************************************************************
      函数说明：在指定区域填充颜色
      入口数据：xsta,ysta   起始坐标
                xend,yend   终止坐标
      返回值：  无
******************************************************************************/
/*****************************************************************************
      Function description: colour fill a rectangle
      Parameters:
	    xsta, ysta: top left starting coordinates (column, row)
        xend, yend: bottom right ending coordinates (column, row xend>= xsta, yend>=ysta)
		color: 16 bit colour
      Return value: None
******************************************************************************/
void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color)
{          
	u16 i,j; 
	LCD_Address_Set(xsta,ysta,xend,yend);      //设置光标位置 // set cursor position
	for(i=ysta;i<=yend;i++)
	{													   	 	
		for(j=xsta;j<=xend;j++)LCD_WR_DATA(color);//设置光标位置  // set pixel colour  	    
	} 					  	    
}


/******************************************************************************
      函数说明：画线
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
      返回值：  无
******************************************************************************/
/*****************************************************************************
      Function description: draw a line
	  Parameters: 
        x1, y1 starting coordinates (column, row)
        x2, y2 ending coordinates (column, row)
		color: 16 bit colour
      Return value: None
******************************************************************************/
void LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //计算坐标增量  // calculate increments of coordinates
	delta_y=y2-y1;
	uRow=x1;//画线起点坐标  // coordindates of starting position
	uCol=y1;
	if(delta_x>0)incx=1; //设置单步方向  // set single step direction
	else if (delta_x==0)incx=0;//垂直线  // vertical line
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;//水平线 // horizontal line
	else {incy=-1;delta_y=-delta_x;}
	if(delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴  // select the basic axis for incrementation
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		LCD_DrawPoint(uRow,uCol,color);//画点  // draw a pixel
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


/******************************************************************************
      函数说明：画矩形
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
      返回值：  无
******************************************************************************/
/*****************************************************************************
      Function description: draw a rectangle
	  Parameters: 
        x1, y1 starting coordinates (column, row)
        x2, y2 ending coordinates (column, row)
		color: 16 bit colour
      Return value: None
******************************************************************************/
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color)
{
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}


/******************************************************************************
      函数说明：画圆
      入口数据：x0,y0   圆心坐标
                r       半径
      返回值：  无
******************************************************************************/
/*****************************************************************************
      Function description: draw circle
      Parameters:
	    x0, y0: coordinates of the center (column, row)
        r: radius
		color: 16 bit colour
      Return value: None
******************************************************************************/
void Draw_Circle(u16 x0,u16 y0,u8 r,u16 color)
{
	int a,b;
	// int di;
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
		if((a*a+b*b)>(r*r))//判断要画的点是否过远
		{
			b--;
		}
	}
}


/******************************************************************************
      函数说明：显示字符
      入口数据：x,y    起点坐标
                num    要显示的字符
                mode   1叠加方式  0非叠加方式
      返回值：  无
******************************************************************************/
/*****************************************************************************
      Function description: Display a character
	  Parameters:
         x: start column,
		 y: start row
         num: character number
         mode: 0 opaque, 1 transparent
		 color: 16 bit colour
      Return value: None
******************************************************************************/
void LCD_ShowChar(u16 x,u16 y,u8 num,u8 mode,u16 color)
{
    u8 temp;
    u8 pos,t;
	u16 x0=x;    
    if(x>LCD_W-16||y>LCD_H-16)return;	    //设置窗口		 // set window
	num=num-' ';//得到偏移后的值 // get the character offset
	LCD_Address_Set(x,y,x+8-1,y+16-1);      //设置光标位置   // set the cursor position
	if(!mode) //非叠加方式 // opaque mode
	{
		for(pos=0;pos<16;pos++)
		{ 
			temp=asc2_1608[(u16)num*16+pos];		 //调用1608字体  // use 1608 font
			for(t=0;t<8;t++)
		    {                 
		        if(temp&0x01)LCD_WR_DATA(color);
				else LCD_WR_DATA(BACK_COLOR);
				temp>>=1;
				x++;
		    }
			x=x0;
			y++;
		}	
	}else//叠加方式  // transparent mode
	{
		for(pos=0;pos<16;pos++)
		{
		    temp=asc2_1608[(u16)num*16+pos];		 //调用1608字体  // use 1608 font
			for(t=0;t<8;t++)
		    {                 
		        if(temp&0x01)LCD_DrawPoint(x+t,y+pos,color);//画一个点   // set pixel 
		        temp>>=1; 
		    }
		}
	}
}


/******************************************************************************
      函数说明：显示字符串
      入口数据：x,y    起点坐标
                *p     字符串起始地址
      返回值：  无
******************************************************************************/
/*****************************************************************************
      Function description: display a string (opaque mode) 
	  Parameters:
	     x, y starting coordinates (column, row)
        *p: address of the string
		color: 16 bit colour value
      Return value: None
******************************************************************************/
void LCD_ShowString(u16 x,u16 y,const u8 *p,u16 color)
{         
    while(*p!='\0')
    {       
        if(x>LCD_W-16){x=0;y+=16;}
        if(y>LCD_H-16){y=x=0;LCD_Clear(RED);}
        LCD_ShowChar(x,y,*p,0,color);
        x+=8;
        p++;
    }  
}


/******************************************************************************
      函数说明：显示数字
      入口数据：m底数，n指数
      返回值：  无
******************************************************************************/
/*****************************************************************************
      Function description: power of a number
      Paramters: 
	    m: base (8 bit)
		n: exponent (8 bit)
      Return value: m to the power of n (m^n) (32 bit)
******************************************************************************/
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}


/******************************************************************************
      函数说明：显示数字
      入口数据：x,y    起点坐标
                num    要显示的数字
                len    要显示的数字个数
      返回值：  无
******************************************************************************/
/*****************************************************************************
      Function description: display integer
	  Parameter:
        x, y: starting point coordinates (column, row)
        num: number to display (16 bit)
        len: number of digits to display
		colour: 16 bit colour
      Return value: None
******************************************************************************/
void LCD_ShowNum(u16 x,u16 y,u16 num,u8 len,u16 color)
{         	
	u8 t,temp;
	u8 enshow=0;
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+8*t,y,' ',0,color);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+8*t,y,temp+48,0,color); 
	}
} 


/******************************************************************************
      函数说明：显示小数
      入口数据：x,y    起点坐标
                num    要显示的小数
                len    要显示的数字个数
      返回值：  无
******************************************************************************/
/*****************************************************************************
      Function description: display decimal number
	  Parameter:
        x, y: starting point coordinates (column, row)
        num: decimal to display (float)
        len: number of digits to display
		colour: 16 bit colour
      Return value: None
******************************************************************************/
void LCD_ShowNum1(u16 x,u16 y,float num,u8 len,u16 color)
{         	
	u8 t,temp;
	// u8 enshow=0;
	u16 num1;
	num1=num*100;
	for(t=0;t<len;t++)
	{
		temp=(num1/mypow(10,len-t-1))%10;
		if(t==(len-2))
		{
			LCD_ShowChar(x+8*(len-2),y,'.',0,color);
			t++;
			len+=1;
		}
	 	LCD_ShowChar(x+8*t,y,temp+48,0,color);
	}
}


/******************************************************************************
      函数说明：显示40x40图片
      入口数据：x,y    起点坐标
      返回值：  无
******************************************************************************/
/*****************************************************************************
      Function description: display 40x40 picture
	  Parameters: 
        x1, y1 starting coordinates (column, row)
        x2, y2 ending coordinates (column, row)
		color: 16 bit colour
      Return value: None
******************************************************************************/
void LCD_ShowPicture(u16 x1,u16 y1,u16 x2,u16 y2)
{
	int i;
	LCD_Address_Set(x1,y1,x2,y2);
	for(i=0;i<12800;i++)
	{ 	
		// LCD_WR_DATA8(image[i*2+1]);
		LCD_WR_DATA8(image[i]);
	}			
}

void LCD_ShowLogo(void)
{
	int i;
	LCD_Address_Set(0,0,159,75);
	for(i=0;i<25600;i++)
	{
		LCD_WR_DATA8(logo_bmp[i]);
	}			
}

#endif // LONGAN_NANO
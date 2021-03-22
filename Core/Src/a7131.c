#include"a7131.h"
Uint8			RfBuf[64];
Uint8			RfBuf2[64];
Uint8			TmpUint8;


const Uint8 PageTab[8]={0x00,0x10,0x20,0x30,0x40,0x50,0x60,0x70};//page select

//-----------------------------------------------------------------------------
// 64 bytes PN9 pseudo random code
//-----------------------------------------------------------------------------
Uint8  PN9_Tab[]=
{   0xFF,0x83,0xDF,0x17,0x32,0x09,0x4E,0xD1,
    0xE7,0xCD,0x8A,0x91,0xC6,0xD5,0xC4,0xC4,
    0x40,0x21,0x18,0x4E,0x55,0x86,0xF4,0xDC,
    0x8A,0x15,0xA7,0xEC,0x92,0xDF,0x93,0x53,
    0x30,0x18,0xCA,0x34,0xBF,0xA2,0xC7,0x59,
    0x67,0x8F,0xBA,0x0D,0x6D,0xD8,0x2D,0x7D,
    0x54,0x0A,0x57,0x97,0x70,0x39,0xD2,0x7A,
    0xEA,0x24,0x33,0x85,0xED,0x9A,0x1D,0xE0
};
Uint8  dada[]=
{   0xFF,0x83,0xDF,0x17,0x32,0x09,0x4E,0xD1,
    0xE7,0xCD,0x8A,0x91,0xC6,0xD5,0xC4,0xC4,
    0x40,0x21,0x18,0x4E,0x55,0x86,0xF4,0xDC,
    0x8A,0x15,0xA7,0xEC,0x92,0xDF,0x93,0x53,
    0x30,0x18,0xCA,0x34,0xBF,0xA2,0xC7,0x59,
    0x67,0x8F,0xBA,0x0D,0x6D,0xD8,0x2D,0x7D,
    0x54,0x0A,0x57,0x97,0x70,0x39,0xD2,0x7A,
    0xEA,0x24,0x33,0x85,0xED,0x9A,0x1D,0xE0
};
//-----------------------------------------------------------------------------
// RF ID code
//-----------------------------------------------------------------------------
const Uint8 ID_Tab[] =
{
    0x54, 0x75, 0xC5, 0x2A
};

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//                                   NOTE                                    !!
// !!         THIS CONFIG TABLE ONLY USE ON RF CRYSTAL = 16MHz               !!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
const Uint16  A7131Config[] =
{
    //      address   name              Descript
    //      -------   ----              ---------
    0x00,   //0x00  ; MODE_REG          only reset, not use on config
    0x62,   //0x01  ; MODE_CONTROL_REG  FIFO mode, Enable ARSSI, Enable AIF
    0x00,   //0x02  ; CALIBRATION_REG
    0x3F,   //0x03  ; FIFO1_REG         
    0x00,   //0x04  ; FIFO2_REG
    0x00,   //0x05  ; FIFO_REG
    0x00,   //0x06  ; IDCODE_REG
    0x00,   //0x07  ; RCOSC1_REG
    0x00,   //0x08  ; RCOSC2_REG
    0x00,   //0x09  ; RCOSC3_REG
    0x00,   //0x0A  ; CKO_REG
    0x19,   //0x0B  ; GIO1 register,    SDO
    0x01,   //0x0C  ; GIO2 register,    WTR 
    0x9F,   //0x0D  ; DATARATE_REG      
    0x00,   //0x0E  ; PLL1_REG
    0x0E,   //0x0F  ; PLL2_REG          RFbase 2400.001MHz
    0x96,   //0x10  ; PLL3_REG
    0x00,   //0x11  ; PLL4_REG
    0x04,   //0x12  ; PLL5_REG
    0x3C,   //0x13  ; ChannelGroup1 register
    0x78,   //0x14  ; ChannelGroup2 register
    0xAF,   //0x15  ; TX1_REG           
    0x40,   //0x16  ; TX2_REG
    0x10,   //0x17  ; DELAY1_REG        
    0x60,   //0x18  ; DELAY2_REG        
    0x72,   //0x19  ; RX_REG            
    0x6F,   //0x1A  ; RXGAIN1
    0xC2,   //0x1B  ; RXGAIN2
    0x3D,   //0x1C  ; RXGAIN3
    0xE7,   //0x1D  ; RXGAIN4
    0x00,   //0x1E  ; RSSI_REG
    0xF1,   //0x1F  ; ADC_REG
    0x07,   //0x20  ; CODE1_REG         
    0x06,   //0x21  ; CODE2_REG
    0x2A,   //0x22  ; CODE3_REG
    0x60,   //0x23  ; IFCAL1_REG
    0xFF,   //0x24  ; IFCAL2_REG
    0x80,   //0x25  ; VCOCCAL_REG       
    0xD0,   //0x26  ; VCOCAL1_REG
    0x00,   //0x27  ; VCOCAL2_REG
    0x70,   //0x28  ; VCO deviation 1 register
    0x00,   //0x29  ; VCO deviation 2 register
    0x00,   //0x2A  ; DSA register
    0xDC,   //0x2B  ; VCO Modulation delay register
    0xF0,   //0x2C  ; BATTERY_REG       
    0x3F,   //0x2D  ; TXTEST_REG        
    0x47,   //0x2E  ; RXDEM1_REG
    0xF7,   //0x2F  ; RXDEM2_REG
    0xF0,   //0x30  ; CPC1_REG           
    0x37,   //0x31  ; CPC2_REG           
    0x55,   //0x32  ; CRYSTALTEST_REG
    0x15,   //0x33  ; PLLTEST_REG
    0x15,   //0x34  ; VCOTEST1_REG
    0x00,   //0x35  ; RF Analog register
    0x00,   //0x36  ; Key data register
    0x77,   //0x37  ; Channel select register
    0x00,   //0x38  ; ROM register
    0x00,   //0x39  ; DataRate register
    0x00,   //0x3A  ; FCR register
    0x00,   //0x3B  ; ARD register
    0x00,   //0x3C  ; AFEP register
    0x00,   //0x3D  ; FCB register
    0x00,   //0x3E  ; KEYC register
    0x00,   //0x3F  ; USID register
};

const Uint8 A7131_Addr2A_Config[]=
{
	0x34, //page0,
	0x49, //page1,
	0x00, //Page2,
	0x80, //page3,
	0x80, //page4,
	0x00, //page5,
	0x5A, //page6,
	0x00, //page7,
};

const Uint8  A7131_Addr38_Config[]=
{
	0x00, //page0,
	0x10, //page1,
	0x20, //page2,
	0x24, //page3,
	0x20, //page4,
};
Uint8 SPI1_ReadWriteByte(Uint8 x){
	unsigned char y;
	HAL_SPI_TransmitReceive(&hspi1,&x,&y,1,100);  
	return y;
}

void Err_State(int k)
{
    //ERR display
    //Error Proc...
    while(1)
    {
       printf("error   %d\n",k);
			HAL_Delay(1000);
    }
}

/************************************************************************
**  A7131_WriteReg
************************************************************************/
void A7131_WriteReg(Uint8 addr, Uint8 dataByte)
{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
    addr |= 0x00; //bit cmd=0,r/w=0
    SPI1_ReadWriteByte(addr);
    //send data byte
    SPI1_ReadWriteByte(dataByte);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
}



void A7131_WriteReg_Page(Uint8 addr, Uint8 wbyte, Uint8 page)
{
	A7131_WriteReg(RFANALOG_REG, A7131Config[RFANALOG_REG] | PageTab[page]);//page select
	A7131_WriteReg(addr, wbyte);
}

Uint8 A7131_ReadReg(Uint8 addr)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
    addr |= 0x40; //bit cmd=0,r/w=1
		SPI1_ReadWriteByte(addr);
    unsigned char x=SPI1_ReadWriteByte(0x00);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
	return x;
}


void StrobeCmd(Uint8 src)
{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
   SPI1_ReadWriteByte(src);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
}

void ByteSend(Uint8 src)
{
	SPI1_ReadWriteByte(src);
}

void A7131_WriteID(void)
{
    Uint8 i;
    Uint8 addr;
    addr = IDCODE_REG; //send address 0x06, bit cmd=0, r/w=0
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
    ByteSend(addr);
    for (i=0; i < 4; i++)
        ByteSend(ID_Tab[i]);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
}

void A7131_Reset(void)
{
    A7131_WriteReg(MODE_REG, 0x00); //reset RF chip
}

void CHGroupCal(Uint8 ch)
{
    Uint8 tmp;
    Uint8 vb,vbcf;
    Uint8 deva,adev;
    
    A7131_WriteReg(PLL1_REG, ch);
    
    A7131_WriteReg(CALIBRATION_REG, 0x0C);
    do
    {
        tmp = A7131_ReadReg(CALIBRATION_REG);
        tmp &= 0x0C;
    }while (tmp);
    
    //for check
    tmp = A7131_ReadReg(VCOCAL1_REG);
    vb = tmp & 0x07;
    vbcf = (tmp >>3) & 0x01;
    
    tmp = A7131_ReadReg(VCODEVCAL1_REG);
    deva = tmp;
    
    tmp = A7131_ReadReg(VCODEVCAL2_REG);
    adev = tmp;

    if (adev<45 || adev >68)
        Err_State(1);//error
    
    if (vbcf)
        Err_State(2);//error
}

/*********************************************************************
** calibration
*********************************************************************/
void A7131_Cal(void)
{
    Uint8 tmp, loopCnt;
    Uint8 fb, fbcf, fcd, vcb, vccf, rhc, rlc;

    StrobeCmd(CMD_PLL); //calibration @PLL state
    
    //calibration IF procedure
    loopCnt = 0;
    A7131_WriteReg(RXGAIN3_REG, 0x1D);
    while(1)
    {
        A7131_WriteReg(CALIBRATION_REG, 0x02);
        do
        {
            tmp = A7131_ReadReg(CALIBRATION_REG);
            tmp &= 0x02;
        }while(tmp);
        
        tmp = A7131_ReadReg(IFCAL1_REG);
        fb = tmp & 0x0F;
        if (fb>3 && fb<10)
            break;
        loopCnt++;
        
        if (loopCnt>=10)
            break;
    }
    
    A7131_WriteReg(RXGAIN3_REG, 0x3D);
    
    if (loopCnt >=10)
        Err_State(3);
    
    //calibration RSSI, VCC procedure
    A7131_WriteReg(CALIBRATION_REG, 0x11);
    do
    {
        tmp = A7131_ReadReg(CALIBRATION_REG);
        tmp &= 0x11;
    }while(tmp);
	
    //calibration VBC,VDC procedure
    CHGroupCal(30); //calibrate channel group Bank I
    CHGroupCal(90); //calibrate channel group Bank II
    CHGroupCal(150); //calibrate channel group Bank III
    StrobeCmd(CMD_STBY); //return to STBY state
    
    //for check
    tmp = A7131_ReadReg(IFCAL1_REG);
    fb = tmp & 0x0F;
    fbcf = (tmp >>4) & 0x01;
    
    tmp = A7131_ReadReg(IFCAL2_REG);
    fcd = tmp & 0x1F;
    
    tmp = A7131_ReadReg(VCOCCAL_REG) & 0x1F;
    vcb = tmp & 0x0F;
    vccf = (tmp >>4) & 0x01;
    
    rhc = A7131_ReadReg(RXGAIN2_REG);
    rlc = A7131_ReadReg(RXGAIN3_REG);
    
    if (fbcf || vccf)
        Err_State(4);
}



/*********************************************************************
** A7131_Config
*********************************************************************/
void A7131_Config(void)
{
    Uint8 i;

    //0x00 mode register, for reset
    //0x05 fifo data register
    //0x06 id code register
    //0x3F USID register, read only
    //0x36 key data, 16 bytes
    //0x3D FCB register,4 bytes

    for (i=0x01; i<=0x04; i++)
        A7131_WriteReg(i, A7131Config[i]);
    
    for (i=0x07; i<=0x29; i++)
        A7131_WriteReg(i, A7131Config[i]);
    
    for (i=0; i<=7; i++)//0x2A DAS
        A7131_WriteReg_Page(0x2A, A7131_Addr2A_Config[i], i);
    for (i=0x2B; i<=0x35; i++)
        A7131_WriteReg(i, A7131Config[i]);
    
    A7131_WriteReg(0x37, A7131Config[0x37]);
    
    for (i=0; i<=4; i++)//0x38 ROM
        A7131_WriteReg_Page(0x38, A7131_Addr38_Config[i], i);
    
    for (i=0x39; i<=0x3C; i++)
        A7131_WriteReg(i, A7131Config[i]);
    
    A7131_WriteReg(0x3E, A7131Config[0x3E]);
}
void initRF(void)
{
    A7131_Reset(); //reset A7105 RF chip
    A7131_WriteID(); //write ID code
    A7131_Config(); //config A7105 chip
    A7131_Cal(); //calibration IF,VCO,VCOC
}


Uint8 ByteRead(void)
{
    return SPI1_ReadWriteByte(0xff);
}
void ReadFIFO(Uint8 length)
{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
    ByteSend(FIFO_REG | 0x40);
		HAL_SPI_Receive(&hspi1,RfBuf,64,20);  
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
}


void InitA7131(void){
	
	initRF();
	StrobeCmd(CMD_STBY);
	A7131_WriteReg( IFCAL2_REG, 0xFF );     //  
	A7131_WriteReg( TXTEST_REG, 0x3F );     //  TX power = 10dBm
	A7131_WriteReg( MODECTRL_REG, 0x62 );   //  FIFO mode
	A7131_WriteReg( PLL1_REG, 101 );        // set radio channel
	A7131_WriteReg( CODE1_REG, 0x17);       //  enable CRC check function
  A7131_WriteReg( FIFO1_REG, 64-1 ); // set FIFO length 
}


void WriteFIFO(Uint8 length)
{
    Uint8 i;
    A7131_WriteReg( FIFO1_REG, length-1);   // set FIFO length
    StrobeCmd(CMD_TFR);                     // Reset TX FIFO point
    
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
    ByteSend(FIFO_REG);
    for(i=0; i <length; i++)
        ByteSend(PN9_Tab[i]);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
}

void WriteThing(Uint8 * xx) 
{
    StrobeCmd(CMD_TFR);                     // Reset TX FIFO point
	
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
    ByteSend(FIFO_REG);
		HAL_SPI_Transmit(&hspi1,xx,64,100);  
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
}





















Uint8 SPI1_ReadWriteByte2(Uint8 x){
	unsigned char y;
	HAL_SPI_TransmitReceive(&hspi3,&x,&y,1,100);  
	return y;
}

void Err_State2(int k)
{
    //ERR display
    //Error Proc...
    while(1)
    {
       printf("error   %d\n",k);
			HAL_Delay(1000);
    }
}

/************************************************************************
**  A7131_WriteReg
************************************************************************/
void A7131_WriteReg2(Uint8 addr, Uint8 dataByte)
{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
    addr |= 0x00; //bit cmd=0,r/w=0
    SPI1_ReadWriteByte2(addr);
    //send data byte
    SPI1_ReadWriteByte2(dataByte);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
}



void A7131_WriteReg_Page2(Uint8 addr, Uint8 wbyte, Uint8 page)
{
	A7131_WriteReg2(RFANALOG_REG, A7131Config[RFANALOG_REG] | PageTab[page]);//page select
	A7131_WriteReg2(addr, wbyte);
}

Uint8 A7131_ReadReg2(Uint8 addr)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
    addr |= 0x40; //bit cmd=0,r/w=1
		SPI1_ReadWriteByte2(addr);
    unsigned char x= SPI1_ReadWriteByte2(0x00);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
	return x;
}


void StrobeCmd2(Uint8 src)
{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
   SPI1_ReadWriteByte2(src);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
}

void ByteSend2(Uint8 src)
{
	SPI1_ReadWriteByte2(src);
}

void A7131_WriteID2(void)
{
    Uint8 i;
    Uint8 addr;
    addr = IDCODE_REG; //send address 0x06, bit cmd=0, r/w=0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
    ByteSend2(addr);
    for (i=0; i < 4; i++)
        ByteSend2(ID_Tab[i]);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
}

void A7131_Reset2(void)
{
    A7131_WriteReg2(MODE_REG, 0x00); //reset RF chip
}

void CHGroupCal2(Uint8 ch)
{
    Uint8 tmp;
    Uint8 vb,vbcf;
    Uint8 deva,adev;
    
    A7131_WriteReg2(PLL1_REG, ch);
    
    A7131_WriteReg2(CALIBRATION_REG, 0x0C);
    do
    {
        tmp = A7131_ReadReg2(CALIBRATION_REG);
        tmp &= 0x0C;
    }while (tmp);
    
    //for check
    tmp = A7131_ReadReg2(VCOCAL1_REG);
    vb = tmp & 0x07;
    vbcf = (tmp >>3) & 0x01;
    
    tmp = A7131_ReadReg2(VCODEVCAL1_REG);
    deva = tmp;
    
    tmp = A7131_ReadReg2(VCODEVCAL2_REG);
    adev = tmp;

    if (adev<45 || adev >68)
        Err_State2(1);//error
    
    if (vbcf)
        Err_State2(2);//error
}

/*********************************************************************
** calibration
*********************************************************************/
void A7131_Cal2(void)
{
    Uint8 tmp, loopCnt;
    Uint8 fb, fbcf, fcd, vcb, vccf, rhc, rlc;

    StrobeCmd2(CMD_PLL); //calibration @PLL state
    
    //calibration IF procedure
    loopCnt = 0;
    A7131_WriteReg2(RXGAIN3_REG, 0x1D);
    while(1)
    {
        A7131_WriteReg2(CALIBRATION_REG, 0x02);
        do
        {
            tmp = A7131_ReadReg2(CALIBRATION_REG);
            tmp &= 0x02;
        }while(tmp);
        
        tmp = A7131_ReadReg2(IFCAL1_REG);
        fb = tmp & 0x0F;
        if (fb>3 && fb<10)
            break;
        loopCnt++;
        
        if (loopCnt>=10)
            break;
    }
    
    A7131_WriteReg2(RXGAIN3_REG, 0x3D);
    
    if (loopCnt >=10)
        Err_State2(3);
    
    //calibration RSSI, VCC procedure
    A7131_WriteReg2(CALIBRATION_REG, 0x11);
    do
    {
        tmp = A7131_ReadReg2(CALIBRATION_REG);
        tmp &= 0x11;
    }while(tmp);
	
    //calibration VBC,VDC procedure
    CHGroupCal2(30); //calibrate channel group Bank I
    CHGroupCal2(90); //calibrate channel group Bank II
    CHGroupCal2(150); //calibrate channel group Bank III
    StrobeCmd2(CMD_STBY); //return to STBY state
    
    //for check
    tmp = A7131_ReadReg2(IFCAL1_REG);
    fb = tmp & 0x0F;
    fbcf = (tmp >>4) & 0x01;
    
    tmp = A7131_ReadReg2(IFCAL2_REG);
    fcd = tmp & 0x1F;
    
    tmp = A7131_ReadReg2(VCOCCAL_REG) & 0x1F;
    vcb = tmp & 0x0F;
    vccf = (tmp >>4) & 0x01;
    
    rhc = A7131_ReadReg2(RXGAIN2_REG);
    rlc = A7131_ReadReg2(RXGAIN3_REG);
    
    if (fbcf || vccf)
        Err_State2(4);
}



/*********************************************************************
** A7131_Config
*********************************************************************/
void A7131_Config2(void)
{
    Uint8 i;

    //0x00 mode register, for reset
    //0x05 fifo data register
    //0x06 id code register
    //0x3F USID register, read only
    //0x36 key data, 16 bytes
    //0x3D FCB register,4 bytes

    for (i=0x01; i<=0x04; i++)
        A7131_WriteReg2(i, A7131Config[i]);
    
    for (i=0x07; i<=0x29; i++)
        A7131_WriteReg2(i, A7131Config[i]);
    
    for (i=0; i<=7; i++)//0x2A DAS
        A7131_WriteReg_Page2(0x2A, A7131_Addr2A_Config[i], i);
    for (i=0x2B; i<=0x35; i++)
        A7131_WriteReg2(i, A7131Config[i]);
    
    A7131_WriteReg2(0x37, A7131Config[0x37]);
    
    for (i=0; i<=4; i++)//0x38 ROM
        A7131_WriteReg_Page2(0x38, A7131_Addr38_Config[i], i);
    
    for (i=0x39; i<=0x3C; i++)
        A7131_WriteReg2(i, A7131Config[i]);
    
    A7131_WriteReg2(0x3E, A7131Config[0x3E]);
}
void initRF2(void)
{
    A7131_Reset2(); //reset A7105 RF chip
    A7131_WriteID2(); //write ID code
    A7131_Config2(); //config A7105 chip
    A7131_Cal2(); //calibration IF,VCO,VCOC
}


Uint8 ByteRead2(void)
{
    return SPI1_ReadWriteByte2(0xff);
}
void ReadFIFO2(Uint8 length)
{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
    ByteSend2(FIFO_REG|0x40);
		HAL_SPI_Receive(&hspi3,RfBuf2,64,1010);  
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
}


void InitA71312(void){
	
	initRF2();
	StrobeCmd2(CMD_STBY);
	A7131_WriteReg2( IFCAL2_REG, 0xFF );     //  
	A7131_WriteReg2( TXTEST_REG, 0x3F );     //  TX power = 10dBm
	A7131_WriteReg2( MODECTRL_REG, 0x62 );   //  FIFO mode
	A7131_WriteReg2( PLL1_REG, 100 );        // set radio channel
	A7131_WriteReg2( CODE1_REG, 0x17);       //  enable CRC check function
  A7131_WriteReg2( FIFO1_REG, 64-1 ); // set FIFO length 
}


void WriteFIFO2(Uint8 length)
{
    Uint8 i;
    A7131_WriteReg2( FIFO1_REG, length-1);   // set FIFO length
    StrobeCmd2(CMD_TFR);                     // Reset TX FIFO point
    
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
    ByteSend2(FIFO_REG);
    for(i=0; i <length; i++)
        ByteSend2(PN9_Tab[i]);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
}

void WriteThing2(Uint8 * xx) 
{
    StrobeCmd2(CMD_TFR);                     // Reset TX FIFO point
	
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
    ByteSend2(FIFO_REG);
		HAL_SPI_Transmit(&hspi3,xx,64,20);  
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
}


















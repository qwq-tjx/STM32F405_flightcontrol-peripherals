#include "wit_c_sdk.h"
#include "Delay.h"
#include "Serial.h"

static SerialWrite p_WitSerialWriteFunc = NULL;
static WitI2cWrite p_WitI2cWriteFunc = NULL;
static WitI2cRead p_WitI2cReadFunc = NULL;
static CanWrite p_WitCanWriteFunc = NULL;
static RegUpdateCb p_WitRegUpdateCbFunc = NULL;
static DelaymsCb p_WitDelaymsFunc = NULL;

static uint8_t s_ucAddr = 0xff;
static uint8_t s_ucWitDataBuff[WIT_DATA_BUFF_SIZE];
static uint32_t s_uiWitDataCnt = 0, s_uiProtoclo = 0, s_uiReadRegIndex = 0;
int16_t sReg[REGSIZE];


#define FuncW 0x06
#define FuncR 0x03

static const uint8_t __auchCRCHi[256] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40
};
static const uint8_t __auchCRCLo[256] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40
};


static uint16_t __CRC16(uint8_t *puchMsg, uint16_t usDataLen)
{
    uint8_t uchCRCHi = 0xFF;
    uint8_t uchCRCLo = 0xFF;
    uint8_t uIndex;
    int i = 0;
    uchCRCHi = 0xFF;
    uchCRCLo = 0xFF;
    for (; i<usDataLen; i++)
    {
        uIndex = uchCRCHi ^ puchMsg[i];
        uchCRCHi = uchCRCLo ^ __auchCRCHi[uIndex];
        uchCRCLo = __auchCRCLo[uIndex] ;
    }
    return (uint16_t)(((uint16_t)uchCRCHi << 8) | (uint16_t)uchCRCLo) ;
}
static uint8_t __CaliSum(uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint8_t ucCheck = 0;
    for(i=0; i<len; i++) ucCheck += *(data + i);
    return ucCheck;
}
int32_t WitSerialWriteRegister(SerialWrite Write_func)
{
    if(!Write_func)return WIT_HAL_INVAL;
    p_WitSerialWriteFunc = Write_func;
    return WIT_HAL_OK;
}
static void CopeWitData(uint8_t ucIndex, uint16_t *p_data, uint32_t uiLen)
{
    uint32_t uiReg1 = 0, uiReg2 = 0, uiReg1Len = 0, uiReg2Len = 0;
    uint16_t *p_usReg1Val = p_data;
    uint16_t *p_usReg2Val = p_data+3;
    
    uiReg1Len = 4;
    switch(ucIndex)
    {
        case WIT_ACC:   uiReg1 = AX;    uiReg1Len = 3;  uiReg2 = TEMP;  uiReg2Len = 1;  break;
        case WIT_ANGLE: uiReg1 = Roll;  uiReg1Len = 3;  uiReg2 = VERSION;  uiReg2Len = 1;  break;
        case WIT_TIME:  uiReg1 = YYMM;	break;
        case WIT_GYRO:  uiReg1 = GX;  uiReg1Len = 3; break;
        case WIT_MAGNETIC: uiReg1 = HX; uiReg1Len = 3; break;
        case WIT_DPORT: uiReg1 = D0Status;  break;
        case WIT_PRESS: uiReg1 = PressureL;  break;
        case WIT_GPS:   uiReg1 = LonL;  break;
        case WIT_VELOCITY: uiReg1 = GPSHeight;  break;
        case WIT_QUATER:    uiReg1 = q0;  break;
        case WIT_GSA:   uiReg1 = SVNUM;  break;
        case WIT_REGVALUE:  uiReg1 = s_uiReadRegIndex;  break;
		default:
			return ;

    }
    if(uiLen == 3)
    {
        uiReg1Len = 3;
        uiReg2Len = 0;
    }
    if(uiReg1Len)
	{
		memcpy(&sReg[uiReg1], p_usReg1Val, uiReg1Len<<1);
		p_WitRegUpdateCbFunc(uiReg1, uiReg1Len);
	}
    if(uiReg2Len)
	{
		memcpy(&sReg[uiReg2], p_usReg2Val, uiReg2Len<<1);
		p_WitRegUpdateCbFunc(uiReg2, uiReg2Len);
	}
}

void WitSerialDataIn(uint8_t ucData)
{
    uint16_t usCRC16, usTemp, i, usData[4];
    uint8_t ucSum;

    if(p_WitRegUpdateCbFunc == NULL)return ;
    s_ucWitDataBuff[s_uiWitDataCnt++] = ucData;
    switch(s_uiProtoclo)
    {
        case WIT_PROTOCOL_NORMAL:
            if(s_ucWitDataBuff[0] != 0x55)
            {
                s_uiWitDataCnt--;
                memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                return ;
            }
            if(s_uiWitDataCnt >= 11)
            {
                ucSum = __CaliSum(s_ucWitDataBuff, 10);
                if(ucSum != s_ucWitDataBuff[10])
                {
                    s_uiWitDataCnt--;
                    memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                    return ;
                }
                usData[0] = ((uint16_t)s_ucWitDataBuff[3] << 8) | s_ucWitDataBuff[2];
                usData[1] = ((uint16_t)s_ucWitDataBuff[5] << 8) | s_ucWitDataBuff[4];
                usData[2] = ((uint16_t)s_ucWitDataBuff[7] << 8) | s_ucWitDataBuff[6];
                usData[3] = ((uint16_t)s_ucWitDataBuff[9] << 8) | s_ucWitDataBuff[8];
                CopeWitData(s_ucWitDataBuff[1], usData, 4);
                s_uiWitDataCnt = 0;
            }
        break;
        case WIT_PROTOCOL_MODBUS:
            if(s_uiWitDataCnt > 2)
            {
                if(s_ucWitDataBuff[1] != FuncR)
                {
                    s_uiWitDataCnt--;
                    memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                    return ;
                }
                if(s_uiWitDataCnt < (s_ucWitDataBuff[2] + 5))return ;
                usTemp = ((uint16_t)s_ucWitDataBuff[s_uiWitDataCnt-2] << 8) | s_ucWitDataBuff[s_uiWitDataCnt-1];
                usCRC16 = __CRC16(s_ucWitDataBuff, s_uiWitDataCnt-2);
                if(usTemp != usCRC16)
                {
                    s_uiWitDataCnt--;
                    memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                    return ;
                }
                usTemp = s_ucWitDataBuff[2] >> 1;
                for(i = 0; i < usTemp; i++)
                {
                    sReg[i+s_uiReadRegIndex] = ((uint16_t)s_ucWitDataBuff[(i<<1)+3] << 8) | s_ucWitDataBuff[(i<<1)+4];
                }
                p_WitRegUpdateCbFunc(s_uiReadRegIndex, usTemp);
                s_uiWitDataCnt = 0;
            }
        break;
        case WIT_PROTOCOL_CAN:
        case WIT_PROTOCOL_I2C:
        s_uiWitDataCnt = 0;
        break;
    }
    if(s_uiWitDataCnt == WIT_DATA_BUFF_SIZE)s_uiWitDataCnt = 0;
}
int32_t WitI2cFuncRegister(WitI2cWrite write_func, WitI2cRead read_func)
{
    if(!write_func)return WIT_HAL_INVAL;
    if(!read_func)return WIT_HAL_INVAL;
    p_WitI2cWriteFunc = write_func;
    p_WitI2cReadFunc = read_func;
    return WIT_HAL_OK;
}
int32_t WitCanWriteRegister(CanWrite Write_func)
{
    if(!Write_func)return WIT_HAL_INVAL;
    p_WitCanWriteFunc = Write_func;
    return WIT_HAL_OK;
}
void WitCanDataIn(uint8_t ucData[8], uint8_t ucLen)
{
	uint16_t usData[3];
    if(p_WitRegUpdateCbFunc == NULL)return ;
    if(ucLen < 8)return ;
    switch(s_uiProtoclo)
    {
        case WIT_PROTOCOL_CAN:
            if(ucData[0] != 0x55)return ;
            usData[0] = ((uint16_t)ucData[3] << 8) | ucData[2];
            usData[1] = ((uint16_t)ucData[5] << 8) | ucData[4];
            usData[2] = ((uint16_t)ucData[7] << 8) | ucData[6];
            CopeWitData(ucData[1], usData, 3);
            break;
        case WIT_PROTOCOL_NORMAL:
        case WIT_PROTOCOL_MODBUS:
        case WIT_PROTOCOL_I2C:
            break;
    }
}
int32_t WitRegisterCallBack(RegUpdateCb update_func)
{
    if(!update_func)return WIT_HAL_INVAL;
    p_WitRegUpdateCbFunc = update_func;
    return WIT_HAL_OK;
}
int32_t WitWriteReg(uint32_t uiReg, uint16_t usData)
{
    uint16_t usCRC;
    uint8_t ucBuff[8];
    if(uiReg >= REGSIZE)return WIT_HAL_INVAL;
    switch(s_uiProtoclo)
    {
        case WIT_PROTOCOL_NORMAL:
            if(p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = uiReg & 0xFF;
            ucBuff[3] = usData & 0xff;
            ucBuff[4] = usData >> 8;
            p_WitSerialWriteFunc(ucBuff, 5);
            break;
        case WIT_PROTOCOL_MODBUS:
            if(p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = s_ucAddr;
            ucBuff[1] = FuncW;
            ucBuff[2] = uiReg >> 8;
            ucBuff[3] = uiReg & 0xFF;
            ucBuff[4] = usData >> 8;
            ucBuff[5] = usData & 0xff;
            usCRC = __CRC16(ucBuff, 6);
            ucBuff[6] = usCRC >> 8;
            ucBuff[7] = usCRC & 0xff;
            p_WitSerialWriteFunc(ucBuff, 8);
            break;
        case WIT_PROTOCOL_CAN:
            if(p_WitCanWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = uiReg & 0xFF;
            ucBuff[3] = usData & 0xff;
            ucBuff[4] = usData >> 8;
            p_WitCanWriteFunc(s_ucAddr, ucBuff, 5);
            break;
        case WIT_PROTOCOL_I2C:
            if(p_WitI2cWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = usData & 0xff;
            ucBuff[1] = usData >> 8;
			if(p_WitI2cWriteFunc(s_ucAddr << 1, uiReg, ucBuff, 2) != 1)
			{
				//printf("i2c write fail\r\n");
			}
        break;
	default: 
            return WIT_HAL_INVAL;        
    }
    return WIT_HAL_OK;
}
int32_t WitReadReg(uint32_t uiReg, uint32_t uiReadNum)
{
    uint16_t usTemp, i;
    uint8_t ucBuff[8];
    if((uiReg + uiReadNum) >= REGSIZE)return WIT_HAL_INVAL;
    switch(s_uiProtoclo)
    {
        case WIT_PROTOCOL_NORMAL:
            if(uiReadNum > 4)return WIT_HAL_INVAL;
            if(p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = 0x27;
            ucBuff[3] = uiReg & 0xff;
            ucBuff[4] = uiReg >> 8;
            p_WitSerialWriteFunc(ucBuff, 5);
            break;
        case WIT_PROTOCOL_MODBUS:
            if(p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            usTemp = uiReadNum << 1;
            if((usTemp + 5) > WIT_DATA_BUFF_SIZE)return WIT_HAL_NOMEM;
            ucBuff[0] = s_ucAddr;
            ucBuff[1] = FuncR;
            ucBuff[2] = uiReg >> 8;
            ucBuff[3] = uiReg & 0xFF;
            ucBuff[4] = uiReadNum >> 8;
            ucBuff[5] = uiReadNum & 0xff;
            usTemp = __CRC16(ucBuff, 6);
            ucBuff[6] = usTemp >> 8;
            ucBuff[7] = usTemp & 0xff;
            p_WitSerialWriteFunc(ucBuff, 8);
            break;
        case WIT_PROTOCOL_CAN:
            if(uiReadNum > 3)return WIT_HAL_INVAL;
            if(p_WitCanWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = 0x27;
            ucBuff[3] = uiReg & 0xff;
            ucBuff[4] = uiReg >> 8;
            p_WitCanWriteFunc(s_ucAddr, ucBuff, 5);
            break;
        case WIT_PROTOCOL_I2C:
            if(p_WitI2cReadFunc == NULL)return WIT_HAL_EMPTY;
            usTemp = uiReadNum << 1;
            if(WIT_DATA_BUFF_SIZE < usTemp)return WIT_HAL_NOMEM;
            if(p_WitI2cReadFunc(s_ucAddr << 1, uiReg, s_ucWitDataBuff, usTemp) == 1)
            {
                if(p_WitRegUpdateCbFunc == NULL)return WIT_HAL_EMPTY;
                for(i = 0; i < uiReadNum; i++)
                {
                    sReg[i+uiReg] = ((uint16_t)s_ucWitDataBuff[(i<<1)+1] << 8) | s_ucWitDataBuff[i<<1];
                }
                p_WitRegUpdateCbFunc(uiReg, uiReadNum);
            }
			
            break;
		default: 
            return WIT_HAL_INVAL;
    }
    s_uiReadRegIndex = uiReg;

    return WIT_HAL_OK;
}
int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr)
{
	if(uiProtocol > WIT_PROTOCOL_I2C)return WIT_HAL_INVAL;
    s_uiProtoclo = uiProtocol;
    s_ucAddr = ucAddr;
    s_uiWitDataCnt = 0;
    return WIT_HAL_OK;
}
void WitDeInit(void)
{
    p_WitSerialWriteFunc = NULL;
    p_WitI2cWriteFunc = NULL;
    p_WitI2cReadFunc = NULL;
    p_WitCanWriteFunc = NULL;
    p_WitRegUpdateCbFunc = NULL;
    s_ucAddr = 0xff;
    s_uiWitDataCnt = 0;
    s_uiProtoclo = 0;
}

int32_t WitDelayMsRegister(DelaymsCb delayms_func)
{
    if(!delayms_func)return WIT_HAL_INVAL;
    p_WitDelaymsFunc = delayms_func;
    return WIT_HAL_OK;
}

char CheckRange(short sTemp,short sMin,short sMax)
{
    if ((sTemp>=sMin)&&(sTemp<=sMax)) return 1;
    else return 0;
}
/*Acceleration calibration demo*/
int32_t WitStartAccCali(void)
{
/*
	First place the equipment horizontally, and then perform the following operations
*/
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	    return  WIT_HAL_ERROR;// unlock reg
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(CALSW, CALGYROACC) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
int32_t WitStopAccCali(void)
{
	if(WitWriteReg(CALSW, NORMAL) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
/*Magnetic field calibration*/
int32_t WitStartMagCali(void)
{
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(CALSW, CALMAGMM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
int32_t WitStopMagCali(void)
{
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(CALSW, NORMAL) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
/*change Band*/
int32_t WitSetUartBaud(int32_t uiBaudIndex)
{
	if(!CheckRange(uiBaudIndex,WIT_BAUD_4800,WIT_BAUD_230400))
	{
		return WIT_HAL_INVAL;
	}
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(BAUD, uiBaudIndex) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
/*change Can Band*/
int32_t WitSetCanBaud(int32_t uiBaudIndex)
{
	if(!CheckRange(uiBaudIndex,CAN_BAUD_1000000,CAN_BAUD_3000))
	{
		return WIT_HAL_INVAL;
	}
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(BAUD, uiBaudIndex) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
/*change Bandwidth*/
int32_t WitSetBandwidth(int32_t uiBaudWidth)
{	
	if(!CheckRange(uiBaudWidth,BANDWIDTH_256HZ,BANDWIDTH_5HZ))
	{
		return WIT_HAL_INVAL;
	}
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(BANDWIDTH, uiBaudWidth) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}

/*change output rate */
int32_t WitSetOutputRate(int32_t uiRate)
{	
	if(!CheckRange(uiRate,RRATE_02HZ,RRATE_NONE))
	{
		return WIT_HAL_INVAL;
	}
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(RRATE, uiRate) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}

/*change WitSetContent */
int32_t WitSetContent(int32_t uiRsw)
{	
	if(!CheckRange(uiRsw,RSW_TIME,RSW_MASK))
	{
		return WIT_HAL_INVAL;
	}
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(RSW, uiRsw) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}

/* ============================================================
 * IMU 高级封装实现
 * ============================================================ */

/**
 * @brief  IMU 数据输入 (由 Serial.c 中断调用)
 */
void WitImu_DataIn(uint8_t ucData)
{
    WitSerialDataIn(ucData);
}

static volatile uint32_t s_uiDataUpdate = 0;
static volatile char s_cCmd = 0xff;
static WitImuPrintCb s_pfPrint = NULL;

#define FLAG_ACC          0x01
#define FLAG_GYRO         0x02
#define FLAG_ANGLE        0x04
#define FLAG_MAG          0x08
#define FLAG_QUATERNION   0x10
#define FLAG_PRESS        0x20

static WitImuData_t s_tImuData = {0};

/**
 * @brief  传感器数据更新回调
 */
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
    if(uiReg >= AX && uiReg <= AZ) {
        s_uiDataUpdate |= FLAG_ACC;
    }
    else if(uiReg >= GX && uiReg <= GZ) {
        s_uiDataUpdate |= FLAG_GYRO;
    }
    else if(uiReg >= Roll && uiReg <= Yaw) {
        s_uiDataUpdate |= FLAG_ANGLE;
    }
    else if(uiReg >= HX && uiReg <= HZ) {
        s_uiDataUpdate |= FLAG_MAG;
    }
    else if(uiReg >= q0 && uiReg <= q3) {
        s_uiDataUpdate |= FLAG_QUATERNION;
    }
    else if(uiReg == PressureL) {
        s_uiDataUpdate |= FLAG_PRESS;
    }
}

/**
 * @brief  打印函数
 */
static void Imu_Printf(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if(s_pfPrint) {
        s_pfPrint(buf);
    }
}

/**
 * @brief  注册打印回调
 */
void WitImu_RegisterPrintCb(WitImuPrintCb printCb)
{
    s_pfPrint = printCb;
}

/**
 * @brief  显示帮助信息
 */
void WitImu_ShowHelp(void)
{
    Imu_Printf("\r\n************************     WIT_SDK_DEMO      ************************\r\n");
    Imu_Printf("\r\n************************        HELP           ************************\r\n");
    Imu_Printf("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
    Imu_Printf("UART SEND:m\\r\\n   Magnetic field calibration, After calibration send: e\\r\\n to indicate the end.\r\n");
    Imu_Printf("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
    Imu_Printf("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
    Imu_Printf("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
    Imu_Printf("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
    Imu_Printf("UART SEND:R\\r\\n   The return rate increases to 10Hz.\r\n");
    Imu_Printf("UART SEND:r\\r\\n   The return rate reduction to 1Hz.\r\n");
    Imu_Printf("UART SEND:C\\r\\n   Basic return content: acceleration, angular velocity, angle, magnetic field.\r\n");
    Imu_Printf("UART SEND:c\\r\\n   Return content: acceleration.\r\n");
    Imu_Printf("UART SEND:h\\r\\n   help.\r\n");
    Imu_Printf("******************************************************************************\r\n");
}

/**
 * @brief  处理串口命令
 */
static void CmdProcess(void)
{
    int32_t i32Ret;
    
    switch(s_cCmd)
    {
        case 'a':   // 加速度校准
            i32Ret = WitStartAccCali();
            if(i32Ret != WIT_HAL_OK) Imu_Printf("\r\nSet AccCali Error\r\n");
            break;
            
        case 'm':   // 磁场校准开始
            i32Ret = WitStartMagCali();
            if(i32Ret != WIT_HAL_OK) Imu_Printf("\r\nSet MagCali Error\r\n");
            break;
            
        case 'e':   // 磁场校准结束
            i32Ret = WitStopMagCali();
            if(i32Ret != WIT_HAL_OK) Imu_Printf("\r\nSet MagCali Error\r\n");
            break;
            
        case 'u':   // 降低带宽
            i32Ret = WitSetBandwidth(BANDWIDTH_5HZ);
            if(i32Ret != WIT_HAL_OK) Imu_Printf("\r\nSet Bandwidth Error\r\n");
            break;
            
        case 'U':   // 提高带宽
            i32Ret = WitSetBandwidth(BANDWIDTH_256HZ);
            if(i32Ret != WIT_HAL_OK) Imu_Printf("\r\nSet Bandwidth Error\r\n");
            break;

        case 'R':   // 输出速率提高到 10Hz
            i32Ret = WitSetOutputRate(RRATE_10HZ);
            if(i32Ret != WIT_HAL_OK) Imu_Printf("\r\nSet Rate Error\r\n");
            break;
            
        case 'r':   // 输出速率降低到 1Hz
            i32Ret = WitSetOutputRate(RRATE_1HZ);
            if(i32Ret != WIT_HAL_OK) Imu_Printf("\r\nSet Rate Error\r\n");
            break;
            
        case 'C':   // 输出内容：加速度+角速度+角度+磁场
            i32Ret = WitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE | RSW_MAG);
            if(i32Ret != WIT_HAL_OK) Imu_Printf("\r\nSet RSW Error\r\n");
            break;
            
        case 'c':   // 输出内容：仅加速度
            i32Ret = WitSetContent(RSW_ACC);
            if(i32Ret != WIT_HAL_OK) Imu_Printf("\r\nSet RSW Error\r\n");
            break;
            
        case 'h':   // 帮助
            WitImu_ShowHelp();
            break;
            
        default:
            break;
    }
    s_cCmd = 0xff;
}

/**
 * @brief  处理串口命令数据 (由 WitImu_CmdProcess 调用)
 */
void WitImu_CopeCmdData(unsigned char ucData)
{
    static unsigned char s_ucData[50], s_ucRxCnt = 0;
    
    s_ucData[s_ucRxCnt++] = ucData;
    if(s_ucRxCnt < 3) return;                                      // 少于3个字节，继续等待
    if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
    
    if(s_ucRxCnt >= 3)
    {
        // 检测命令结束标志 "\r\n"
        if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
        {
            s_cCmd = s_ucData[0];
            memset(s_ucData, 0, 50);
            s_ucRxCnt = 0;
        }
        else
        {
            // 滑动窗口，继续查找
            s_ucData[0] = s_ucData[1];
            s_ucData[1] = s_ucData[2];
            s_ucRxCnt = 2;
        }
    }
}

/**
 * @brief  命令处理入口
 */
void WitImu_CmdProcess(uint8_t ucData)
{
    WitImu_CopeCmdData(ucData);
    CmdProcess();
}

/**
 * @brief  IMU 初始化
 * @note  必须先注册串口发送函数，然后才能调用 WitSetContent 等需要发送数据的函数
 */
void WitImu_Init(void)
{
    // ========== 1. 首先注册串口发送函数（必须在其他发送操作之前）==========
    WitSerialWriteRegister(SensorUartSend);
    WitDelayMsRegister(DelayMs_Callback);

    // 2. 注册数据更新回调
    WitRegisterCallBack(SensorDataUpdata);

    // 3. 初始化协议和地址
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);

    // 4. 设置输出频率：100Hz（所有数据）
    WitSetOutputRate(RRATE_100HZ);

    // 5. 设置输出内容：加速度 + 角速度 + 角度 + 磁场 + 气压
    //    此时 p_WitSerialWriteFunc 已注册，WitSetContent 可以正常工作
    WitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE | RSW_MAG | RSW_PRESS);

    // 6. 保存配置到传感器 Flash（断电不丢失）
    WitWriteReg(SAVE, SAVE_PARAM);
    p_WitDelaymsFunc(100);

    // 等待 IMU 响应并开始输出数据 (200ms)
    if(p_WitDelaymsFunc) {
        p_WitDelaymsFunc(200);
    }

    Imu_Printf("\r\n********************** WIT-Motion Normal Example (F405) ************************\r\n");
    Imu_Printf("Output rate: 100Hz, Content: ACC+GYRO+ANGLE+MAG+PRESS\r\n");
    WitImu_ShowHelp();

    WitImu_RegisterPrintCb(WitImu_Print);
}

/**
 * @brief  获取 IMU 数据 (在主循环中调用)
 * @note  返回后 pData 包含最新的传感器数据
 */
void WitImu_GetData(WitImuData_t *pData)
{
    int i;
    uint32_t uiDataUpdate;
    
    if(pData == NULL) return;
    
    // 原子性读取更新标志，避免数据竞争
    uiDataUpdate = s_uiDataUpdate;
    
    // 处理传感器数据更新
    if(uiDataUpdate)
    {
        // 加速度数据
        if(uiDataUpdate & FLAG_ACC)
        {
            // 一次性读取所有加速度值，确保一致性
            short accX = sReg[AX];
            short accY = sReg[AY];
            short accZ = sReg[AZ];
            
            s_tImuData.acc[0] = accX / 32768.0f * 16.0f;
            s_tImuData.acc[1] = accY / 32768.0f * 16.0f;
            s_tImuData.acc[2] = accZ / 32768.0f * 16.0f;
            s_tImuData.acc_updated = 1;
        }
        
        // 角速度数据
        if(uiDataUpdate & FLAG_GYRO)
        {
            // 一次性读取所有角速度值，确保一致性
            short gyroX = sReg[GX];
            short gyroY = sReg[GY];
            short gyroZ = sReg[GZ];
            
            s_tImuData.gyro[0] = gyroX / 32768.0f * 2000.0f;
            s_tImuData.gyro[1] = gyroY / 32768.0f * 2000.0f;
            s_tImuData.gyro[2] = gyroZ / 32768.0f * 2000.0f;
            s_tImuData.gyro_updated = 1;
        }
        
        // 角度数据
        if(uiDataUpdate & FLAG_ANGLE)
        {
            // 一次性读取所有角度值，确保一致性
            short rawRoll = sReg[Roll];
            short rawPitch = sReg[Pitch];
            short rawYaw = sReg[Yaw];
            
            s_tImuData.angle[0] = rawRoll / 32768.0f * 180.0f;
            s_tImuData.angle[1] = rawPitch / 32768.0f * 180.0f;
            s_tImuData.angle[2] = rawYaw / 32768.0f * 180.0f;
            s_tImuData.angle_updated = 1;
        }
        
        // 磁场数据
        if(uiDataUpdate & FLAG_MAG)
        {
            // 一次性读取所有磁场值，确保一致性
            short magX = sReg[HX];
            short magY = sReg[HY];
            short magZ = sReg[HZ];
            
            s_tImuData.mag[0] = magX / 10.0f;  // 原始数据单位 0.1mT，转换为 mT
            s_tImuData.mag[1] = magY / 10.0f;
            s_tImuData.mag[2] = magZ / 10.0f;
            s_tImuData.mag_updated = 1;
        }
        
        // 四元数数据
        if(uiDataUpdate & FLAG_QUATERNION)
        {
            short raw_q0 = sReg[q0];
            short raw_q1 = sReg[q1];
            short raw_q2 = sReg[q2];
            short raw_q3 = sReg[q3];
            
            s_tImuData.quat[0] = raw_q0 / 32768.0f;  // w
            s_tImuData.quat[1] = raw_q1 / 32768.0f;  // x
            s_tImuData.quat[2] = raw_q2 / 32768.0f;  // y
            s_tImuData.quat[3] = raw_q3 / 32768.0f;  // z
            s_tImuData.quat_updated = 1;
        }
        
        // 气压数据 (含高度) — 32-bit 值，高低 16 位分存
        if(uiDataUpdate & FLAG_PRESS)
        {
            // PressureL(0x45)=低16位, PressureH(0x46)=高16位 → 32-bit 气压 (Pa 或 0.01hPa)
            int32_t rawPressure = ((int32_t)(uint16_t)sReg[PressureH] << 16) | (uint16_t)sReg[PressureL];
            // HeightL(0x47)=低16位, HeightH(0x48)=高16位 → 32-bit 高度 (cm)
            int32_t rawAltitude = ((int32_t)(uint16_t)sReg[HeightH]  << 16) | (uint16_t)sReg[HeightL];
            
            s_tImuData.pressure = rawPressure / 100.0f;   // → hPa
            s_tImuData.altitude = rawAltitude / 100.0f;   // → m
            s_tImuData.pressure_updated = 1;
        }
        
        // 清除更新标志
        s_uiDataUpdate = 0;
    }
    
    // 复制数据到用户缓冲区
    for(i = 0; i < 3; i++) {
        pData->acc[i] = s_tImuData.acc[i];
        pData->gyro[i] = s_tImuData.gyro[i];
        pData->angle[i] = s_tImuData.angle[i];
        pData->mag[i] = s_tImuData.mag[i];
    }
    for(i = 0; i < 4; i++) {
        pData->quat[i] = s_tImuData.quat[i];
    }
    pData->pressure = s_tImuData.pressure;
    pData->altitude = s_tImuData.altitude;
    pData->acc_updated = s_tImuData.acc_updated;
    pData->gyro_updated = s_tImuData.gyro_updated;
    pData->angle_updated = s_tImuData.angle_updated;
    pData->mag_updated = s_tImuData.mag_updated;
    pData->quat_updated = s_tImuData.quat_updated;
    pData->pressure_updated = s_tImuData.pressure_updated;
}

static void WitImu_Print(const char *str)
{
    Serial_Printf("%s", str);
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
    UART5_SendData(p_data, uiSize);
}

static void DelayMs_Callback(uint16_t ms)
{
    Delay_ms(ms);
}


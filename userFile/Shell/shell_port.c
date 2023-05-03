#include "main.h"
#include "shell_port.h"
#include "ring_buffer.h"
#include "computerUart.h"
//

#include "shell.h"
#define SHELL_LOG
static uint8_t isShellOpen;
static uint8_t DataBuff[256];  //shell 用缓冲区
static uint8_t ringBuff[256];
char tempData;
computerUart_t computerUart;
void commandProcess(void);
void switchToShell();
void getEeprom(uint8_t page);
void laser_cur_eeprom(void);
 
void osSaveParmEeprom(void const * argument );
osThreadId osSaveParmEepromHandle;
Shell shell;
#ifdef SHELL_LOG
#include "log.h"
Log uartLog = {
    .write = uartLogWrite,
    .active = 1,
    .level = LOG_DEBUG
};
#endif
void userShellInit(void)
{
	isShellOpen=USE_SHELL;
  shell.write = shellWrite;
  shell.read = shellRead;
  shellInit(&shell,(char *)DataBuff,256); //shell初始化
	Ring_Buffer_Init(&RB,ringBuff,256);
#ifdef SHELL_LOG
    logRegister(&uartLog, &shell); //shell伴生对象初始化
#endif
    HAL_UART_Receive_IT(&huart3,(uint8_t*)(&tempData),1);
}
static short shellWrite(char* data,unsigned short len)
{
    if(osSemaphoreWait(uartSendBinarySemHandle,osWaitForever)==osOK)
    {
        HAL_UART_Transmit_DMA(&huart3, (uint8_t *)data, len);
        return len;
    }
    else
    {
        return 0;
    }
}
static short shellRead(char* data,unsigned short len)
{
    unsigned short curDataLength=Ring_Buffer_Get_Lenght(&RB);
    if(curDataLength>0 && Ring_Buffer_Get_Lenght(&RB)>=len)
    {
        Ring_Buffer_Read_String(&RB,(uint8_t*)data,len);
        return len;
    }
    else if(curDataLength>0 && Ring_Buffer_Get_Lenght(&RB)<len)
    {
        Ring_Buffer_Read_String(&RB,(uint8_t*)data,curDataLength);
        return curDataLength;
    }
    else
    {
        return 0;
    }
}
#ifdef SHELL_LOG
void uartLogWrite(char *buffer, short len)
{
	while(osSemaphoreWait(uartSendBinarySemHandle,osWaitForever)!=osOK)
	{}
  HAL_UART_Transmit_DMA(&huart3, (uint8_t *)buffer, len);
        
}
#endif
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart3)
    {
        osSemaphoreRelease(uartSendBinarySemHandle);
    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        Ring_Buffer_Write_Byte(&RB, tempData);

        HAL_UART_Receive_IT(&huart3, (uint8_t *)(&tempData), 1);
    }
}

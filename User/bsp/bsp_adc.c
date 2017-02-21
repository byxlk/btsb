
#include "bsp.h"



/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((u32)0x4001204C)
vu16 ADC_ConvertedValue[4];

uint16_t ExtTempValue[152] = {
    42559, 39921, 37464, 35173, 33036, 31043, 29182, 27444, 25821, 24303, 22884, 21557, 20315, 
    19151, 18062, 17041, 16084, 15187, 14345, 13554, 12812, 12116, 11461, 10845, 10266,
    
    9722, 9209, 8727, 8273, 7845, 7441, 7061, 6703, 6364, 6045, 5744, 5459, 5190, 4936, 4696, 4469, 4254, 
    4051, 3858, 3676, 3503, 3340, 3185, 3038, 2899, 2767, 2641, 2522, 2409, 2302, 2200, 2103, 2011, 1924, 
    1840, 1761, 1686, 1614, 1546, 1481, 1419, 1360, 1304, 1250, 1199, 1150, 1104, 1059, 1017,

    976, 938, 901, 866, 832, 800, 769, 740, 711, 684, 659, 634, 610, 588, 566, 545, 525, 506, 488, 470, 454, 
    438, 422, 407, 393, 379, 366, 354, 341, 330, 319, 308, 298, 288, 278, 269, 260, 251, 243, 235, 228, 220,
    213, 207, 200, 194, 188, 182, 176, 171, 166, 160, 156, 151, 146, 142, 138, 134, 130, 126, 122, 119, 115, 
    112, 109, 106, 103, 100,

    97, 94, 92, 89, 86, 84, 82, 80, 77, 68
};

void bsp_InitADC(void)
{
      ADC_InitTypeDef       ADC_InitStructure;
      ADC_CommonInitTypeDef ADC_CommonInitStructure;
      DMA_InitTypeDef       DMA_InitStructure;
      GPIO_InitTypeDef      GPIO_InitStructure;

      /* 使能 ADC1， DMA 和 GPIOA 时钟 */
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA, ENABLE);
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

      /* Configure ADC1 Channel6 pin as analog input ******************************/
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

    
      //DMA_DeInit(DMA2_Stream0);
      /* DMA2 Stream0 channe0 configuration **************************************/
      //DMA_DeInit(DMA2_Stream0);
      DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
      DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;//外设地址
      DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_ConvertedValue;//内存地址
      DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//DMA传输方向
      DMA_InitStructure.DMA_BufferSize = 4;//设置DMA在传输时缓冲区的长度 word,根据通道数来定
      DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//设置DMA的外设递增模式
      DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//设置DMA的内存递增模式
      DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//外设数据字长
      DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//内存数据字长
      DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//设置DMA的传输模式：连续不断的循环模式
      DMA_InitStructure.DMA_Priority = DMA_Priority_High;//设置DMA的优先级别
      DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  //       
      DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
      DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
      DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
      DMA_Init(DMA2_Stream0, &DMA_InitStructure);
      DMA_Cmd(DMA2_Stream0, ENABLE);
    
      /****************************************************************************   
        PCLK2 = HCLK / 2 
        下面选择的是2分频
        ADCCLK = PCLK2 /8 = HCLK / 8 = 168 / 8 = 21M
        ADC采样频率： Sampling Time + Conversion Time = 480 + 12 cycles = 492cyc
                      Conversion Time = 21MHz / 492cyc = 42.6ksps. 
      *****************************************************************************/

    
      /* ADC Common Init **********************************************************/
      ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
      ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;//预分频
      ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
      ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
      ADC_CommonInit(&ADC_CommonInitStructure);
    
      /* ADC1 Init ****************************************************************/
      ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; //设置12bit的分辨率
      ADC_InitStructure.ADC_ScanConvMode = ENABLE;//扫描模式，以单通道和多通道来区分
      ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//连续模式
      ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//外部触发禁止
      ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//数据对齐
      ADC_InitStructure.ADC_NbrOfConversion = 4;//设置ADC转换通道数
      ADC_Init(ADC1, &ADC_InitStructure);

     /* Enable ADC3 DMA */
      ADC_DMACmd(ADC1, ENABLE);
     
      /* ADC1 regular channe6 configuration *************************************/
      ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_480Cycles);
      ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_480Cycles);
      ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 3, ADC_SampleTime_3Cycles);
      ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_480Cycles);

      ADC_TempSensorVrefintCmd(ENABLE);

     /* Enable DMA request after last transfer (Single-ADC mode) */
      ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

      /* Enable ADC3 */
      ADC_Cmd(ADC1, ENABLE);

      ADC_SoftwareStartConv(ADC1);

}

/*
*************************************************************************************************
  VSENSE：温度传感器的当前输出电压，与ADC_DR 寄存器中的结果ADC_ConvertedValue之间的转换关系为： 
            ADC_ConvertedValue * Vdd
  VSENSE = --------------------------
            Vdd_convert_value(0xFFF)  
    ADC转换结束以后，读取ADC_DR寄存器中的结果，转换温度值计算公式如下：
          V25 - VSENSE
  T(℃) = ------------  + 25
           Avg_Slope
  V25：  温度传感器在25℃时 的输出电压，典型值0.76 V。
  Avg_Slope：温度传感器输出电压和温度的关联参数，典型值2.5 mV/℃。
************************************************************************************************
*/
int8_t getCurent_IntTempValue(void)
{
#if 0

    float Vtemp_IntSensor;
    float Current_IntTemp = 0.0;
    
    Vtemp_IntSensor = ADC_ConvertedValue[0] * 3.3f / 4095;  				           
    Current_IntTemp = (Vtemp_IntSensor - 0.76f) /0.0025f + 25;  
 
#else

    uint32_t Vtemp_IntSensor;
    int8_t Current_IntTemp = 0.0;
    
    Vtemp_IntSensor = ADC_ConvertedValue[0] * 33000 / 4095;                             
    Current_IntTemp = (Vtemp_IntSensor - 7600) / 25 + 25; 

#endif
    return Current_IntTemp;
}

#define UP_R_VALUE 20 //20K
int8_t getCurent_ExtTempValue(void)
{
    uint32_t Vtemp_ExtSeneor;
    uint32_t Rtemp_Value;
    int8_t Current_ExtTemp = 125;
    uint8_t i = 0;

    /* 得到ADC采集的电压 */
    Vtemp_ExtSeneor = ADC_ConvertedValue[1] * 3300 / 4095;

    /* 计算当前温度传感器的阻值 */
    Rtemp_Value = Vtemp_ExtSeneor * UP_R_VALUE * 100 / (3300 - Vtemp_ExtSeneor);

    /* 根据温度传感器的阻值查表获得当前温度 */
    if(Rtemp_Value <= ExtTempValue[0] && Rtemp_Value > ExtTempValue[151])
    {
        for(i = 0; i < 151 ; i++ )
        {
            if( Rtemp_Value <= ExtTempValue[i] && Rtemp_Value > ExtTempValue[i+1] )
            {
                Current_ExtTemp = i - 30;
                break;
            }
        }
    }
    else if(Rtemp_Value > ExtTempValue[0])
        Current_ExtTemp = -30;
    else 
        Current_ExtTemp = 125;

    return Current_ExtTemp;
}

#define R19_VALUE 2 // 2K
uint8_t getLightVLuxValue(void)
{
    uint32_t VLight_Sensor; // mA
    uint16_t ILight_Sensor; // uA
    uint8_t LightToLuxConvValue = 0; // Lux
    
    VLight_Sensor = ADC_ConvertedValue[2] * 3300 / 4095;
    ILight_Sensor = (3300 - VLight_Sensor)  /R19_VALUE;

    if(ILight_Sensor < 10 ) LightToLuxConvValue = 0;
    else if(ILight_Sensor < 10 ) LightToLuxConvValue = 0;
    else if(ILight_Sensor < 20 ) LightToLuxConvValue = 1;
    else if(ILight_Sensor < 30 ) LightToLuxConvValue = 2;
    else if(ILight_Sensor < 40 ) LightToLuxConvValue = 3;
    else if(ILight_Sensor < 48 ) LightToLuxConvValue = 4;
    else if(ILight_Sensor < 55 ) LightToLuxConvValue = 5;
    else if(ILight_Sensor < 58 ) LightToLuxConvValue = 6;
    else if(ILight_Sensor < 63 ) LightToLuxConvValue = 7;
    else if(ILight_Sensor < 69 ) LightToLuxConvValue = 8;
    else if(ILight_Sensor < 74 ) LightToLuxConvValue = 9;
    else if(ILight_Sensor < 80 ) LightToLuxConvValue = 10;
    else if(ILight_Sensor < 90 ) LightToLuxConvValue = 12;
    else if(ILight_Sensor < 100 ) LightToLuxConvValue = 15;
    else if(ILight_Sensor < 200 ) LightToLuxConvValue = 30;
    else if(ILight_Sensor < 300 ) LightToLuxConvValue = 48;
    else if(ILight_Sensor < 400 ) LightToLuxConvValue = 68;
    else if(ILight_Sensor < 500 ) LightToLuxConvValue = 88;
    else if(ILight_Sensor < 600 ) LightToLuxConvValue = 95;
    else  LightToLuxConvValue = 100;
    
    return (LightToLuxConvValue);
}

uint16_t getMicAmp_dBValue(void)
{
    return (ADC_ConvertedValue[3] * 3300 / 4095);
}


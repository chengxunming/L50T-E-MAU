/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include "global.h"

/* Private variables ---------------------------------------------------------*/

/*时钟配置*/
void RCC_Configuration(void)
{
    ErrorStatus HSEStartUpStatus;

   
	RCC_DeInit(); 
	//使能外部晶振
    RCC_HSEConfig(RCC_HSE_ON);
    //等待外部晶振稳定
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    //如果外部晶振启动成功，则进行下一步操作
    if(HSEStartUpStatus==SUCCESS)
    {
        //设置HCLK（AHB时钟）=SYSCLK
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        //PCLK1(APB1) = HCLK/2
        RCC_PCLK1Config(RCC_HCLK_Div2);
        //PCLK2(APB2) = HCLK
        RCC_PCLK2Config(RCC_HCLK_Div1);
        //设置ADC时钟频率
        RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
        //FLASH时序控制
        //推荐值：SYSCLK = 0~24MHz   Latency=0
        //        SYSCLK = 24~48MHz  Latency=1
        //        SYSCLK = 48~72MHz  Latency=2
        //PLL设置 SYSCLK/1 * 9 = 8*1*9 = 72MHz
        FLASH_SetLatency(FLASH_Latency_2);  //flash操作的延时   //Flash 2 wait state
				FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);   //flash读取缓冲，加速 //Enable Prefetch Buffer
			
				RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
        //启动PLL
        RCC_PLLCmd(ENABLE);
        //等待PLL稳定
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
				
        //系统时钟SYSCLK来自PLL输出
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        //切换时钟后等待系统时钟稳定
        while(RCC_GetSYSCLKSource()!=0x08);    
    }

		//启动GPIO
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
		//启动AFIO
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		//启动 CAN1
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
		//启动 timer1
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		//启动 timer2
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		//启动 timer3
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		//启动 timer4
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

}



/**********************************************************************
* 名    称：IWDG_INIT()
* 功    能：延时
* 入口参数：cnt
* 出口参数：
-----------------------------------------------------------------------
* 说明：
***********************************************************************/
void IWDG_INIT(void)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_32);
	IWDG_SetReload(0x4e2);
	IWDG_ReloadCounter();
	IWDG_Enable();
}




/**************************************************************************************************************
函数描述 : GPIO初始化
***************************************************************************************************************/
void GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//上电设置所有GPIO为输入上拉
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|
								   GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|
								   GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|
								   GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|
								   GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	

	/***************************************************************************
	                      KeyBoard 矩阵键盘初始化
	****************************************************************************/
	//设为上拉输入
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_SetBits(GPIOB,GPIO_Pin_1|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);	//置1,默认上拉输入
	GPIO_SetBits(GPIOC,GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);
	
	/***************************************************************************
	                      KeyBoard 背光LED初始化
	****************************************************************************/
	//设置 KeyBoard 背光LED 默认推挽输出，默认置1
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_SetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);	//置1关闭LED
	GPIO_SetBits(GPIOC,GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
	
	/***************************************************************************
	                      状态指示灯LED初始化
	****************************************************************************/
	//设置状态指示灯LED推挽输出，默认置1
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD,&GPIO_InitStructure);

	GPIO_SetBits(GPIOB,GPIO_Pin_6);	//置1关闭LED
	GPIO_SetBits(GPIOC,GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12);
	GPIO_SetBits(GPIOD,GPIO_Pin_2);
	
	/***************************************************************************
	                      拨动开关指示灯LED初始化
	****************************************************************************/
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_SetBits(GPIOA,GPIO_Pin_8);	//置1关闭LED
	GPIO_SetBits(GPIOC,GPIO_Pin_14|GPIO_Pin_15);

	/***************************************************************************
	                      拨动开关状态检测初始化
	****************************************************************************/
	//拨动开关状态检测，上拉输入默认置1
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_SetBits(GPIOB,GPIO_Pin_0); //置1，默认上拉输入
	GPIO_SetBits(GPIOC,GPIO_Pin_4|GPIO_Pin_5); //置1，默认上拉输入

	/***************************************************************************
	                      蜂鸣器Beep输出控制初始化
	****************************************************************************/
	//蜂鸣器驱动Beep推挽输出，默认置0
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_ResetBits(GPIOC,GPIO_Pin_0);	//置0，关闭蜂鸣器输出

	/***************************************************************************
	                      CAN通信引脚初始化
	****************************************************************************/
	//CAN GPIO CONFIG
	// Configure CAN pin: RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	//Configure CAN pin: TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
}

/**************************************************************************************************************
函数描述 : CAN初始化
***************************************************************************************************************/
void CAN_Configuration(void)
{
	CAN_InitTypeDef  CAN_InitStructure;

	CAN_DeInit(CAN1);/* CAN register init */
	CAN_StructInit(&CAN_InitStructure);
	
	CAN_InitStructure.CAN_TTCM = DISABLE; /*禁止时间触发通讯模式*/
	CAN_InitStructure.CAN_ABOM = ENABLE; /*自动退出离线状态方式，0-有条件手动离线，1-自动退出离线状态*/
	//CAN_InitStructure.CAN_ABOM = DISABLE; /*自动退出离线状态方式，0-有条件手动离线，1-自动退出离线状态*/
	CAN_InitStructure.CAN_AWUM = DISABLE; /*0-由软件通过清0唤醒，1-检测到报文时，自动唤醒*/
	CAN_InitStructure.CAN_NART = DISABLE; /*0-一直重复发送直到成功，1-不论成功以否只发送一次*///自动重传											
	CAN_InitStructure.CAN_RFLM = DISABLE; /*0-溢出时FIFO未锁定，新报文盖掉旧报文，1-锁定，溢出后新报文直接丢失*/
	CAN_InitStructure.CAN_TXFP = DISABLE; /*0-报文发送优先级由标志符决定，1-报文发送优先级由请求先后顺序决定*/
//	CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;/*模式-测试模式-正常模式*/
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
/*-----------------------------------------------
//BaudRate = APB1 / ((BS1 + BS2 + 1) * Prescaler)
//--------------------------------------------------------
//   速率   |  SJW  |  BS1  |  BS2  | prescaler | Sample |
//--------------------------------------------------------
// 1000kbps |  1tq  | 2tq   |  1tq  |     9     |  75.0% |
//--------------------------------------------------------
//  800kbps |  1tq  | 3tq   |  1tq  |     9     |  80.0% |
//--------------------------------------------------------
//  500kbps |  1tq  | 6tq   |  1tq  |     9     |  87.5% |
//--------------------------------------------------------
//  250kbps |  1tq  | 6tq   |  1tq  |     18    |  87.5% |
//--------------------------------------------------------
//  125kbps |  1tq  | 13tq  |  2tq  |     18    |  87.5% |
//--------------------------------------------------------
//  100kbps |  1tq  | 6tq   |  1tq  |     45    |  87.5% |  
//--------------------------------------------------------*/
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;//重同步时间宽度
	CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;//CAN_BS1_5tq;//时间1宽度
	CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;//时间2宽度
	CAN_InitStructure.CAN_Prescaler = 9;//分频值(时间单元长度)
	CAN_Init(CAN1, &CAN_InitStructure);
	
	//CAN_Filter_Reconfig(BOARD);
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

/*
 * 函数名：CAN_Filter_Config
 * 描述  ：CAN的过滤器 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_Filter_Config(void)
{
   CAN_FilterInitTypeDef  CAN_FilterInitStructure;
   
   
   /*CAN过滤器初始化*/
	CAN_FilterInitStructure.CAN_FilterNumber=0;						//过滤器组0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//工作在标识列表模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//过滤器位宽为单个32位。
	/* 使能报文标示符过滤器按照标示符的内容进行比对过滤，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */

    CAN_FilterInitStructure.CAN_FilterIdHigh= ((uint32_t)(Heartbeat_ID<<21)&0xffff0000)>>16;				//要过滤的ID高位 
    CAN_FilterInitStructure.CAN_FilterIdLow= (CAN_ID_STD|CAN_RTR_DATA)&0xffff; //要过滤的ID低位 
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x6FFF;//0x7FFF;			//过滤器高16位除心跳帧之外必须匹配
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFFFF;//0xFFFF;			//过滤器低16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;				//过滤器被关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//使能过滤器
	CAN_FilterInit(&CAN_FilterInitStructure);

	/*CAN过滤器初始化*/
	CAN_FilterInitStructure.CAN_FilterNumber=1;						//过滤器组0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//工作在标识列表模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//过滤器位宽为单个32位。
	/* 使能报文标示符过滤器按照标示符的内容进行比对过滤，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */

    CAN_FilterInitStructure.CAN_FilterIdHigh= ((uint32_t)(MAU_CAN_ID<<21)&0xffff0000)>>16;				//要过滤的ID高位 
    CAN_FilterInitStructure.CAN_FilterIdLow= (CAN_ID_STD|CAN_RTR_DATA)&0xffff; //要过滤的ID低位 
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x6FFF;//0x7FFF;			//过滤器高16位除心跳帧之外必须匹配
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFFFF;//0xFFFF;			//过滤器低16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;				//过滤器被关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//使能过滤器
	CAN_FilterInit(&CAN_FilterInitStructure);
	/*CAN通信中断使能*/
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}



/**************************************************************************************************************
函数描述 : 中断优先级设置
***************************************************************************************************************/	

/* ============================================================================================================================
    NVIC_PriorityGroup   | NVIC_IRQChannelPreemptionPriority | NVIC_IRQChannelSubPriority  | Description
  ============================================================================================================================
   NVIC_PriorityGroup_0  |                0                  |            0-15             |   0 bits for pre-emption priority
                         |                                   |                             |   4 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------
   NVIC_PriorityGroup_1  |                0-1                |            0-7              |   1 bits for pre-emption priority
                         |                                   |                             |   3 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------    
   NVIC_PriorityGroup_2  |                0-3                |            0-3              |   2 bits for pre-emption priority
                         |                                   |                             |   2 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------    
   NVIC_PriorityGroup_3  |                0-7                |            0-1              |   3 bits for pre-emption priority
                         |                                   |                             |   1 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------    
   NVIC_PriorityGroup_4  |                0-15               |            0                |   4 bits for pre-emption priority
                         |                                   |                             |   0 bits for subpriority                       
  ============================================================================================================================*/


void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	/* Enable the CAN Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	

	
}


/**************************************************************************************************************
函数描述 : 定时器初始化
***************************************************************************************************************/	
void TIM_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	//-----------------------------------------------------------------TIM2	  100ms    主程序推动定时器(心跳+执行)
	//进入中断服务程序间隔时间为 TIM2 = ((1+TIM_Prescaler)/72M)*(1+TIM_Period)=100ms
	TIM_DeInit(TIM2);
	TIM_TimeBaseStructure.TIM_Period=(200-1);		 						/* 自动重装载寄存器周期的值(计数值) */
	/* 累计 TIM_Period个频率后产生一个更新或者中断 */					//定时公式：(TIM_Period + 1) * (TIM_Prescaler + 1) / TIMx Clock(72M)
	TIM_TimeBaseStructure.TIM_Prescaler= (36000 - 1);				    	/* 时钟预分频数 72M/36000 = 2k*/
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 				/* 采样分频 */
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;			/* 向上计数模式 */
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);							    /* 清除溢出中断标志 */
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM2, ENABLE);												/* 开启时钟 */

	//-----------------------------------------------------------------TIM3		100us     蜂鸣器专用
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructure.TIM_Period=(200-1);		 						/* 自动重装载寄存器周期的值(计数值) */
	/* 累计 TIM_Period个频率后产生一个更新或者中断 */
	TIM_TimeBaseStructure.TIM_Prescaler= (36 - 1);				    	/* 时钟预分频数 72M/36 = 2M*/
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 				/* 采样分频 */
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;			/* 向上计数模式 */
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);							    /* 清除溢出中断标志 */
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
//	TIM_Cmd(TIM3, ENABLE);												/* 开启时钟 */

	//-----------------------------------------------------------------TIM4		10ms    数据采集定时器
	//进入中断服务程序间隔时间为TIM4 = ((1+TIM_Prescaler)/72M)*(1+TIM_Period)=10ms
	TIM_DeInit(TIM4);
	TIM_TimeBaseStructure.TIM_Period=(20-1);		 						/* 自动重装载寄存器周期的值(计数值) */
	/* 累计 TIM_Period个频率后产生一个更新或者中断 */
	TIM_TimeBaseStructure.TIM_Prescaler= (36000 - 1);				    	/* 时钟预分频数 72M/36000 = 2k,1/72MHz=13.89ns*/
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 				/* 采样分频 时间分割值*/
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;			/* 向上计数模式 */
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);           /* 初始化定时器TIM4 */
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);							    /* 清除溢出中断标志 */
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);           /* 打开中断 溢出中断 */
	TIM_Cmd(TIM4, ENABLE);												/* 开启时钟 */


}

void Structure_Init(void)
{
	gFlag.ActionOnOrOffLine=0;
	
	gFlag.KeyFNFun=false;
	gFlag.KeyScan=false;
	gFlag.LedAllOn=false;
	gFlag.LedLockBlink=false;
	gFlag.PasswordChange=false;
	gFlag.Ring=false;
	gFlag.SeriousErr=false;
	gFlag.WarningErr=false;
	gFlag.CanDataRx=false;
	
	gFlag.CPUOffLine=true;
	gFlag.LedRefresh=true;
	gFlag.PasswordEnable=true;
	gFlag.PasswordLock=true;
	gFlag.KeyNone=true;
	gFlag.SendHeartbeat=true;
//	gFlag.Wait=true;
	gFlag.Wait=false;
}

int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */    


	SystemInit();			//系统初始化
   
	RCC_Configuration();	//系统RCC时钟初始化
	
	GPIOInit();				//IO初始化
	
	NVIC_Configuration();	//中断优先级设置
	
	delay_init(72);			//延迟初始化，使用systick做延迟
	
	delay_ms(500);			//上电延迟500ms，待单片机稳定
	
	CAN_Configuration();	//CAN配置，速率125K
	CAN_Filter_Config();    //CAN过滤器设置，只有心跳包，命令数据可以传送进来
	
	Structure_Init();
	
	TIM_Configuration();	//定时器初始化
	
//	IWDG_INIT();				//看门狗初始化				   
	
	while (1)
	{	
		if(gFlag.SendHeartbeat)   //心跳发送
		{
			gFlag.SendHeartbeat=false;
			Send_CAN_HeartbeatFrame();
//			IWDG_ReloadCounter();
		}
		
		if(gFlag.CanDataRx)
		{
			Can_Receive_CMD();
			gFlag.CanDataRx=false;
		}
		
//		if(gFlag.KeyScan && gFlag.CPUOffLine!=true)
		if(gFlag.KeyScan)
		{
			gFlag.KeyScan=false;
			KEY_Scan();
			SWITCH_Scan();
			KEY_Process();
		}
		
		if (gFlag.PasswordChange)//密码修改标志
		{
			gFlag.PasswordChange=false;
			FLASH_PasswordSave(password);//密码存入Flash
		}

		if(gFlag.LedRefresh)
		{
			gFlag.LedRefresh=false;
			LED_REFRESH();
		}
		if(gFlag.LedAllOn)
		{
			LEDALL(1);
		}
		RUNLED_FLASH();	//运行灯闪烁
		
		Device_StateChange();
	}
}

/******************* (C) COPYRIGHT 2015 Lopu *****END OF FILE****/

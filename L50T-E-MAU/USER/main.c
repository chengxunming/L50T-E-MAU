/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include "global.h"

/* Private variables ---------------------------------------------------------*/

/*ʱ������*/
void RCC_Configuration(void)
{
    ErrorStatus HSEStartUpStatus;

   
	RCC_DeInit(); 
	//ʹ���ⲿ����
    RCC_HSEConfig(RCC_HSE_ON);
    //�ȴ��ⲿ�����ȶ�
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    //����ⲿ���������ɹ����������һ������
    if(HSEStartUpStatus==SUCCESS)
    {
        //����HCLK��AHBʱ�ӣ�=SYSCLK
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        //PCLK1(APB1) = HCLK/2
        RCC_PCLK1Config(RCC_HCLK_Div2);
        //PCLK2(APB2) = HCLK
        RCC_PCLK2Config(RCC_HCLK_Div1);
        //����ADCʱ��Ƶ��
        RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
        //FLASHʱ�����
        //�Ƽ�ֵ��SYSCLK = 0~24MHz   Latency=0
        //        SYSCLK = 24~48MHz  Latency=1
        //        SYSCLK = 48~72MHz  Latency=2
        //PLL���� SYSCLK/1 * 9 = 8*1*9 = 72MHz
        FLASH_SetLatency(FLASH_Latency_2);  //flash��������ʱ   //Flash 2 wait state
				FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);   //flash��ȡ���壬���� //Enable Prefetch Buffer
			
				RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
        //����PLL
        RCC_PLLCmd(ENABLE);
        //�ȴ�PLL�ȶ�
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
				
        //ϵͳʱ��SYSCLK����PLL���
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        //�л�ʱ�Ӻ�ȴ�ϵͳʱ���ȶ�
        while(RCC_GetSYSCLKSource()!=0x08);    
    }

		//����GPIO
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
		//����AFIO
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		//���� CAN1
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
		//���� timer1
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		//���� timer2
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		//���� timer3
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		//���� timer4
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

}



/**********************************************************************
* ��    �ƣ�IWDG_INIT()
* ��    �ܣ���ʱ
* ��ڲ�����cnt
* ���ڲ�����
-----------------------------------------------------------------------
* ˵����
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
�������� : GPIO��ʼ��
***************************************************************************************************************/
void GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//�ϵ���������GPIOΪ��������
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
	                      KeyBoard ������̳�ʼ��
	****************************************************************************/
	//��Ϊ��������
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_SetBits(GPIOB,GPIO_Pin_1|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);	//��1,Ĭ����������
	GPIO_SetBits(GPIOC,GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);
	
	/***************************************************************************
	                      KeyBoard ����LED��ʼ��
	****************************************************************************/
	//���� KeyBoard ����LED Ĭ�����������Ĭ����1
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_SetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);	//��1�ر�LED
	GPIO_SetBits(GPIOC,GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
	
	/***************************************************************************
	                      ״ָ̬ʾ��LED��ʼ��
	****************************************************************************/
	//����״ָ̬ʾ��LED���������Ĭ����1
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

	GPIO_SetBits(GPIOB,GPIO_Pin_6);	//��1�ر�LED
	GPIO_SetBits(GPIOC,GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12);
	GPIO_SetBits(GPIOD,GPIO_Pin_2);
	
	/***************************************************************************
	                      ��������ָʾ��LED��ʼ��
	****************************************************************************/
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_SetBits(GPIOA,GPIO_Pin_8);	//��1�ر�LED
	GPIO_SetBits(GPIOC,GPIO_Pin_14|GPIO_Pin_15);

	/***************************************************************************
	                      ��������״̬����ʼ��
	****************************************************************************/
	//��������״̬��⣬��������Ĭ����1
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_SetBits(GPIOB,GPIO_Pin_0); //��1��Ĭ����������
	GPIO_SetBits(GPIOC,GPIO_Pin_4|GPIO_Pin_5); //��1��Ĭ����������

	/***************************************************************************
	                      ������Beep������Ƴ�ʼ��
	****************************************************************************/
	//����������Beep���������Ĭ����0
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_ResetBits(GPIOC,GPIO_Pin_0);	//��0���رշ��������

	/***************************************************************************
	                      CANͨ�����ų�ʼ��
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
�������� : CAN��ʼ��
***************************************************************************************************************/
void CAN_Configuration(void)
{
	CAN_InitTypeDef  CAN_InitStructure;

	CAN_DeInit(CAN1);/* CAN register init */
	CAN_StructInit(&CAN_InitStructure);
	
	CAN_InitStructure.CAN_TTCM = DISABLE; /*��ֹʱ�䴥��ͨѶģʽ*/
	CAN_InitStructure.CAN_ABOM = ENABLE; /*�Զ��˳�����״̬��ʽ��0-�������ֶ����ߣ�1-�Զ��˳�����״̬*/
	//CAN_InitStructure.CAN_ABOM = DISABLE; /*�Զ��˳�����״̬��ʽ��0-�������ֶ����ߣ�1-�Զ��˳�����״̬*/
	CAN_InitStructure.CAN_AWUM = DISABLE; /*0-�����ͨ����0���ѣ�1-��⵽����ʱ���Զ�����*/
	CAN_InitStructure.CAN_NART = DISABLE; /*0-һֱ�ظ�����ֱ���ɹ���1-���۳ɹ��Է�ֻ����һ��*///�Զ��ش�											
	CAN_InitStructure.CAN_RFLM = DISABLE; /*0-���ʱFIFOδ�������±��ĸǵ��ɱ��ģ�1-������������±���ֱ�Ӷ�ʧ*/
	CAN_InitStructure.CAN_TXFP = DISABLE; /*0-���ķ������ȼ��ɱ�־��������1-���ķ������ȼ��������Ⱥ�˳�����*/
//	CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;/*ģʽ-����ģʽ-����ģʽ*/
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
/*-----------------------------------------------
//BaudRate = APB1 / ((BS1 + BS2 + 1) * Prescaler)
//--------------------------------------------------------
//   ����   |  SJW  |  BS1  |  BS2  | prescaler | Sample |
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
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;//��ͬ��ʱ����
	CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;//CAN_BS1_5tq;//ʱ��1���
	CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;//ʱ��2���
	CAN_InitStructure.CAN_Prescaler = 9;//��Ƶֵ(ʱ�䵥Ԫ����)
	CAN_Init(CAN1, &CAN_InitStructure);
	
	//CAN_Filter_Reconfig(BOARD);
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

/*
 * ��������CAN_Filter_Config
 * ����  ��CAN�Ĺ����� ����
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_Filter_Config(void)
{
   CAN_FilterInitTypeDef  CAN_FilterInitStructure;
   
   
   /*CAN��������ʼ��*/
	CAN_FilterInitStructure.CAN_FilterNumber=0;						//��������0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//�����ڱ�ʶ�б�ģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//������λ��Ϊ����32λ��
	/* ʹ�ܱ��ı�ʾ�����������ձ�ʾ�������ݽ��бȶԹ��ˣ���չID�������µľ����������ǵĻ��������FIFO0�� */

    CAN_FilterInitStructure.CAN_FilterIdHigh= ((uint32_t)(Heartbeat_ID<<21)&0xffff0000)>>16;				//Ҫ���˵�ID��λ 
    CAN_FilterInitStructure.CAN_FilterIdLow= (CAN_ID_STD|CAN_RTR_DATA)&0xffff; //Ҫ���˵�ID��λ 
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x6FFF;//0x7FFF;			//��������16λ������֮֡�����ƥ��
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFFFF;//0xFFFF;			//��������16λÿλ����ƥ��
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;				//��������������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//ʹ�ܹ�����
	CAN_FilterInit(&CAN_FilterInitStructure);

	/*CAN��������ʼ��*/
	CAN_FilterInitStructure.CAN_FilterNumber=1;						//��������0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//�����ڱ�ʶ�б�ģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//������λ��Ϊ����32λ��
	/* ʹ�ܱ��ı�ʾ�����������ձ�ʾ�������ݽ��бȶԹ��ˣ���չID�������µľ����������ǵĻ��������FIFO0�� */

    CAN_FilterInitStructure.CAN_FilterIdHigh= ((uint32_t)(MAU_CAN_ID<<21)&0xffff0000)>>16;				//Ҫ���˵�ID��λ 
    CAN_FilterInitStructure.CAN_FilterIdLow= (CAN_ID_STD|CAN_RTR_DATA)&0xffff; //Ҫ���˵�ID��λ 
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x6FFF;//0x7FFF;			//��������16λ������֮֡�����ƥ��
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFFFF;//0xFFFF;			//��������16λÿλ����ƥ��
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;				//��������������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//ʹ�ܹ�����
	CAN_FilterInit(&CAN_FilterInitStructure);
	/*CANͨ���ж�ʹ��*/
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}



/**************************************************************************************************************
�������� : �ж����ȼ�����
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
�������� : ��ʱ����ʼ��
***************************************************************************************************************/	
void TIM_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	//-----------------------------------------------------------------TIM2	  100ms    �������ƶ���ʱ��(����+ִ��)
	//�����жϷ��������ʱ��Ϊ TIM2 = ((1+TIM_Prescaler)/72M)*(1+TIM_Period)=100ms
	TIM_DeInit(TIM2);
	TIM_TimeBaseStructure.TIM_Period=(200-1);		 						/* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) */
	/* �ۼ� TIM_Period��Ƶ�ʺ����һ�����»����ж� */					//��ʱ��ʽ��(TIM_Period + 1) * (TIM_Prescaler + 1) / TIMx Clock(72M)
	TIM_TimeBaseStructure.TIM_Prescaler= (36000 - 1);				    	/* ʱ��Ԥ��Ƶ�� 72M/36000 = 2k*/
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 				/* ������Ƶ */
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;			/* ���ϼ���ģʽ */
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);							    /* �������жϱ�־ */
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM2, ENABLE);												/* ����ʱ�� */

	//-----------------------------------------------------------------TIM3		100us     ������ר��
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructure.TIM_Period=(200-1);		 						/* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) */
	/* �ۼ� TIM_Period��Ƶ�ʺ����һ�����»����ж� */
	TIM_TimeBaseStructure.TIM_Prescaler= (36 - 1);				    	/* ʱ��Ԥ��Ƶ�� 72M/36 = 2M*/
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 				/* ������Ƶ */
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;			/* ���ϼ���ģʽ */
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);							    /* �������жϱ�־ */
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
//	TIM_Cmd(TIM3, ENABLE);												/* ����ʱ�� */

	//-----------------------------------------------------------------TIM4		10ms    ���ݲɼ���ʱ��
	//�����жϷ��������ʱ��ΪTIM4 = ((1+TIM_Prescaler)/72M)*(1+TIM_Period)=10ms
	TIM_DeInit(TIM4);
	TIM_TimeBaseStructure.TIM_Period=(20-1);		 						/* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) */
	/* �ۼ� TIM_Period��Ƶ�ʺ����һ�����»����ж� */
	TIM_TimeBaseStructure.TIM_Prescaler= (36000 - 1);				    	/* ʱ��Ԥ��Ƶ�� 72M/36000 = 2k,1/72MHz=13.89ns*/
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 				/* ������Ƶ ʱ��ָ�ֵ*/
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;			/* ���ϼ���ģʽ */
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);           /* ��ʼ����ʱ��TIM4 */
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);							    /* �������жϱ�־ */
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);           /* ���ж� ����ж� */
	TIM_Cmd(TIM4, ENABLE);												/* ����ʱ�� */


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


	SystemInit();			//ϵͳ��ʼ��
   
	RCC_Configuration();	//ϵͳRCCʱ�ӳ�ʼ��
	
	GPIOInit();				//IO��ʼ��
	
	NVIC_Configuration();	//�ж����ȼ�����
	
	delay_init(72);			//�ӳٳ�ʼ����ʹ��systick���ӳ�
	
	delay_ms(500);			//�ϵ��ӳ�500ms������Ƭ���ȶ�
	
	CAN_Configuration();	//CAN���ã�����125K
	CAN_Filter_Config();    //CAN���������ã�ֻ�����������������ݿ��Դ��ͽ���
	
	Structure_Init();
	
	TIM_Configuration();	//��ʱ����ʼ��
	
//	IWDG_INIT();				//���Ź���ʼ��				   
	
	while (1)
	{	
		if(gFlag.SendHeartbeat)   //��������
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
		
		if (gFlag.PasswordChange)//�����޸ı�־
		{
			gFlag.PasswordChange=false;
			FLASH_PasswordSave(password);//�������Flash
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
		RUNLED_FLASH();	//���е���˸
		
		Device_StateChange();
	}
}

/******************* (C) COPYRIGHT 2015 Lopu *****END OF FILE****/

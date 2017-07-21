#ifndef __global_H
#define __global_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "math.h"
#include "function.h"
#include "delay.h"
#include <string.h>
#include <stdbool.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define  ON       1
#define  OFF      0

#define HardVersions		20      //Ӳ���汾��
#define SoftVersions		20      //����汾��

#define	Heartbeat_ID	0x00
#define	MAU_CAN_ID		0x03



/* Private macro -------------------------------------------------------------*/
#define  FLASHPASSWORD_ADDR    0x0801F800						//����Ĵ��λ��127K����126ҳ

//����������
#define  BEEPON           GPIO_SetBits(GPIOC,GPIO_Pin_0)//��������
#define  BEEPOFF          GPIO_ResetBits(GPIOC,GPIO_Pin_0)//�������ر�

//LEDָʾ�ƿ���
#define RUNLEDON		   GPIO_ResetBits(GPIOD,GPIO_Pin_2)//����ָʾ�ƴ�
#define RUNLEDOFF		   GPIO_SetBits(GPIOD,GPIO_Pin_2)

#define ERRLEDON		   GPIO_ResetBits(GPIOC,GPIO_Pin_12)//����ָʾ�ƴ�
#define ERRLEDOFF		   GPIO_SetBits(GPIOC,GPIO_Pin_12)

#define KEYLOCKON		   GPIO_ResetBits(GPIOC,GPIO_Pin_10)//����ָʾ�ƴ�
#define KEYLOCKOFF		   GPIO_SetBits(GPIOC,GPIO_Pin_10)

#define WAITON		   GPIO_ResetBits(GPIOC,GPIO_Pin_11)//�ȴ�ָʾ�ƴ�
#define WAITOFF		   GPIO_SetBits(GPIOC,GPIO_Pin_11)

#define AUTOON		 GPIO_ResetBits(GPIOB,GPIO_Pin_6)//�Զ�ָʾ�ƴ�
#define AUTOOFF	 	 GPIO_SetBits(GPIOB,GPIO_Pin_6)
//����ָʾ�ƿ���
#define TUNONLEDON		 GPIO_ResetBits(GPIOA,GPIO_Pin_8)//��ƿ���ָʾ�ƴ�
#define TUNONLEDOFF		 GPIO_SetBits(GPIOA,GPIO_Pin_8)

#define MANON		   	 GPIO_ResetBits(GPIOC,GPIO_Pin_14)//�ֿ�ָʾ�ƴ�
#define MANOFF		     GPIO_SetBits(GPIOC,GPIO_Pin_14)


//ͨ��ָʾ�ƿ���
//Ϊ��ϱ����趨���������Ŷ����Ӧ�����
#define LEDFNON		  	  GPIO_ResetBits(GPIOA,GPIO_Pin_7)//FN
#define LEDFNOFF		  GPIO_SetBits(GPIOA,GPIO_Pin_7)

#define LEDYFLASHON		  GPIO_ResetBits(GPIOA,GPIO_Pin_4)//����
#define LEDYFLASHOFF	  GPIO_SetBits(GPIOA,GPIO_Pin_4)

#define LEDALLREDON		  GPIO_ResetBits(GPIOA,GPIO_Pin_1)//ȫ��,Step4
#define LEDALLREDOFF	  GPIO_SetBits(GPIOA,GPIO_Pin_1)

#define LEDSTEPON		  GPIO_ResetBits(GPIOC,GPIO_Pin_15)//����ָʾ��
#define LEDSTEPOFF		  GPIO_SetBits(GPIOC,GPIO_Pin_15)  //��������1���ر�ָʾ��

#define LEDONEON		  GPIO_ResetBits(GPIOA,GPIO_Pin_6)//������1,Step1
#define LEDONEOFF		  GPIO_SetBits(GPIOA,GPIO_Pin_6)

#define LEDTWOON		  GPIO_ResetBits(GPIOA,GPIO_Pin_3)//������2,Step2
#define LEDTWOOFF		  GPIO_SetBits(GPIOA,GPIO_Pin_3)

#define LEDTHREEON		  GPIO_ResetBits(GPIOA,GPIO_Pin_0)//������3,Step3
#define LEDTHREEOFF		  GPIO_SetBits(GPIOA,GPIO_Pin_0)

#define LEDFOURON		  GPIO_ResetBits(GPIOC,GPIO_Pin_2)//������4,Step4
#define LEDFOUROFF		  GPIO_SetBits(GPIOC,GPIO_Pin_2)

#define LEDFIVEON		  GPIO_ResetBits(GPIOA,GPIO_Pin_5)//������5,Step5
#define LEDFIVEOFF		  GPIO_SetBits(GPIOA,GPIO_Pin_5)

#define LEDSIXON		  GPIO_ResetBits(GPIOA,GPIO_Pin_2)//������6,Step6
#define LEDSIXOFF		  GPIO_SetBits(GPIOA,GPIO_Pin_2)

#define LEDSEVENON		  GPIO_ResetBits(GPIOC,GPIO_Pin_3)//������7,Step7
#define LEDSEVENOFF		  GPIO_SetBits(GPIOC,GPIO_Pin_3)

#define LEDEIGHTON		  GPIO_ResetBits(GPIOC,GPIO_Pin_1)//������8,Step8
#define LEDEIGHTOFF		  GPIO_SetBits(GPIOC,GPIO_Pin_1)


//��������״̬��ȡ���У��ϣ�
#define SIGNAL1	GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)//��ƿ���
#define SIGNAL2	GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)//�ֿؿ��أ�0Ϊ��


//����״̬��ȡ
#define KEY_NUMFN_IN		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)
#define KEY_NUMYF_IN		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10)
#define KEY_NUMAR_IN		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)
#define KEY_NUMSTEP_IN		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)//�������أ�1Ϊ��
#define KEY_NUM1_IN			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)
#define KEY_NUM2_IN			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)
#define KEY_NUM3_IN			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)
#define KEY_NUM4_IN			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)
#define KEY_NUM5_IN			GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)
#define KEY_NUM6_IN			GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)
#define KEY_NUM7_IN			GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8)
#define KEY_NUM8_IN			GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)

//�ṹ�嶨��
typedef struct  //LED״̬
{
	uint8_t Fault;
	_Bool Run;
	_Bool Lock;
	_Bool Wait;
	_Bool Auto;
	
	_Bool Open;
	_Bool Manual;
	_Bool Step;
	
	_Bool One;
	_Bool Two;
	_Bool Three;
	_Bool Four;
	_Bool Five;
	_Bool Six;
	_Bool Seven;
	_Bool Eight;
	_Bool Fn;
	_Bool Y_Flash;
	_Bool Allred;	
}LED;

typedef enum//����״̬
{
	KEY_PRESS=0,
	KEY_RELEASE,
	KEY_NONE
}KeyStateType;



typedef enum//��ֵ
{
	KEY_FN=0,
	KEY_YF=1,
	KEY_AR,
	KEY_STEP,
	KEY_NUM1,
	KEY_NUM2,
	KEY_NUM3,
	KEY_NUM4,
	KEY_NUM5,
	KEY_NUM6,
	KEY_NUM7,
	KEY_NUM8,
	KEY_TEST,
	KEY_YFFN,
	KEY_ARFN,
	KEY_STEPFN,
	KEY_NUM9,
	KEY_NUM10,
	KEY_NUM11,
	KEY_NUM12,
	KEY_NUM13,
	KEY_NUM14,
	KEY_NUM15,
	KEY_NUM16
}KeyValType;

typedef struct
{
	uint8_t KeyVal;
	KeyStateType KeyState; 
}KeyType;

typedef struct//���ض���
{
	uint8_t State;
	bool StateChange;
}SwitchModeType;

typedef struct//��־λ
{
	uint8_t ActionOnOrOffLine;//0:�豸״̬δ�����仯  1���豸���߲���  2���豸���߲���
	
	bool SendHeartbeat;//true������������
	bool CPUOffLine;//���߱�־λ  false������  true������
	bool KeyFNFun;//FN��ϼ����ܱ�־λ false:FN��Ч   true��FN��Ч 
	bool KeyScan;//������ѯ��־λ  true����ѯ����
	bool PasswordEnable;//������־λ  false����������   true��ʹ������
	bool PasswordChange;//���������־λ  false���������   true����Ҫ����
	bool PasswordLock;//����������־λ    false������       true������
	bool WarningErr;//������ϱ�־	     false���޹���     true���й���
	bool SeriousErr;//���ع��ϱ�־	     false���޹���     true���й���
	bool LedLockBlink;//����ָʾ����˸    false������˸     true����˸
	bool KeyNone;//15min���ް������±�־λ false���а������� true���ް�������
	bool LedRefresh;//ˢ��ָʾ�Ʊ�־λ     true��ˢ��
	bool LedAllOn;	//LEDȫ������־λ      true��ȫ������
	bool Wait;//�ȴ�״̬	  true������ȴ�״̬    false���˳��ȴ�״̬
    bool Ring;//�����־λ true������   
	bool CanDataRx;//�������ݱ�־λ true�����յ�����
}FlagType;   


extern volatile FlagType gFlag;
extern uint8_t password;


void FLASH_PasswordSave(uint8_t value);
uint8_t FLASH_PasswordGet(void);
uint8_t Send_CAN_HeartbeatFrame();
void Can_Receive_CMD(void);
void KEY_Scan(void);
void SWITCH_Scan(void);
void LEDALL(uint8_t Mark);
void KEY_Process(void);
void LED_REFRESH(void);
void RUNLED_FLASH();
void Device_StateChange(void);


#endif

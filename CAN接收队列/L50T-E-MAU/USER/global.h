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

#define HardVersions		20      //硬件版本号
#define SoftVersions		20      //软件版本号

#define	Heartbeat_ID	0x00
#define	MAU_CAN_ID		0x03



/* Private macro -------------------------------------------------------------*/
#define  FLASHPASSWORD_ADDR    0x0801F800						//密码的存放位置127K处即126页

//蜂鸣器控制
#define  BEEPON           GPIO_SetBits(GPIOC,GPIO_Pin_0)//蜂鸣器打开
#define  BEEPOFF          GPIO_ResetBits(GPIOC,GPIO_Pin_0)//蜂鸣器关闭

//LED指示灯控制
#define RUNLEDON		   GPIO_ResetBits(GPIOD,GPIO_Pin_2)//运行指示灯打开
#define RUNLEDOFF		   GPIO_SetBits(GPIOD,GPIO_Pin_2)

#define ERRLEDON		   GPIO_ResetBits(GPIOC,GPIO_Pin_12)//故障指示灯打开
#define ERRLEDOFF		   GPIO_SetBits(GPIOC,GPIO_Pin_12)

#define KEYLOCKON		   GPIO_ResetBits(GPIOC,GPIO_Pin_10)//健锁指示灯打开
#define KEYLOCKOFF		   GPIO_SetBits(GPIOC,GPIO_Pin_10)

#define WAITON		   GPIO_ResetBits(GPIOC,GPIO_Pin_11)//等待指示灯打开
#define WAITOFF		   GPIO_SetBits(GPIOC,GPIO_Pin_11)

#define AUTOON		 GPIO_ResetBits(GPIOB,GPIO_Pin_6)//自动指示灯打开
#define AUTOOFF	 	 GPIO_SetBits(GPIOB,GPIO_Pin_6)
//开关指示灯控制
#define TUNONLEDON		 GPIO_ResetBits(GPIOA,GPIO_Pin_8)//外灯开灯指示灯打开
#define TUNONLEDOFF		 GPIO_SetBits(GPIOA,GPIO_Pin_8)

#define MANON		   	 GPIO_ResetBits(GPIOC,GPIO_Pin_14)//手控指示灯打开
#define MANOFF		     GPIO_SetBits(GPIOC,GPIO_Pin_14)


//通道指示灯控制
//为配合背板设定，调整引脚定义对应输出口
#define LEDFNON		  	  GPIO_ResetBits(GPIOA,GPIO_Pin_7)//FN
#define LEDFNOFF		  GPIO_SetBits(GPIOA,GPIO_Pin_7)

#define LEDYFLASHON		  GPIO_ResetBits(GPIOA,GPIO_Pin_4)//黄闪
#define LEDYFLASHOFF	  GPIO_SetBits(GPIOA,GPIO_Pin_4)

#define LEDALLREDON		  GPIO_ResetBits(GPIOA,GPIO_Pin_1)//全红,Step4
#define LEDALLREDOFF	  GPIO_SetBits(GPIOA,GPIO_Pin_1)

#define LEDSTEPON		  GPIO_ResetBits(GPIOC,GPIO_Pin_15)//步进指示灯
#define LEDSTEPOFF		  GPIO_SetBits(GPIOC,GPIO_Pin_15)  //将引脚置1，关闭指示灯

#define LEDONEON		  GPIO_ResetBits(GPIOA,GPIO_Pin_6)//步进点1,Step1
#define LEDONEOFF		  GPIO_SetBits(GPIOA,GPIO_Pin_6)

#define LEDTWOON		  GPIO_ResetBits(GPIOA,GPIO_Pin_3)//步进点2,Step2
#define LEDTWOOFF		  GPIO_SetBits(GPIOA,GPIO_Pin_3)

#define LEDTHREEON		  GPIO_ResetBits(GPIOA,GPIO_Pin_0)//步进点3,Step3
#define LEDTHREEOFF		  GPIO_SetBits(GPIOA,GPIO_Pin_0)

#define LEDFOURON		  GPIO_ResetBits(GPIOC,GPIO_Pin_2)//步进点4,Step4
#define LEDFOUROFF		  GPIO_SetBits(GPIOC,GPIO_Pin_2)

#define LEDFIVEON		  GPIO_ResetBits(GPIOA,GPIO_Pin_5)//步进点5,Step5
#define LEDFIVEOFF		  GPIO_SetBits(GPIOA,GPIO_Pin_5)

#define LEDSIXON		  GPIO_ResetBits(GPIOA,GPIO_Pin_2)//步进点6,Step6
#define LEDSIXOFF		  GPIO_SetBits(GPIOA,GPIO_Pin_2)

#define LEDSEVENON		  GPIO_ResetBits(GPIOC,GPIO_Pin_3)//步进点7,Step7
#define LEDSEVENOFF		  GPIO_SetBits(GPIOC,GPIO_Pin_3)

#define LEDEIGHTON		  GPIO_ResetBits(GPIOC,GPIO_Pin_1)//步进点8,Step8
#define LEDEIGHTOFF		  GPIO_SetBits(GPIOC,GPIO_Pin_1)


//拨动开关状态读取（中，上）
#define SIGNAL1	GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)//外灯开关
#define SIGNAL2	GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)//手控开关，0为打开


//按键状态读取
#define KEY_NUMFN_IN		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)
#define KEY_NUMYF_IN		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10)
#define KEY_NUMAR_IN		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)
#define KEY_NUMSTEP_IN		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)//步进开关，1为打开
#define KEY_NUM1_IN			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)
#define KEY_NUM2_IN			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)
#define KEY_NUM3_IN			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)
#define KEY_NUM4_IN			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)
#define KEY_NUM5_IN			GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)
#define KEY_NUM6_IN			GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)
#define KEY_NUM7_IN			GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8)
#define KEY_NUM8_IN			GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)

//结构体定义
typedef struct  //LED状态
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

typedef enum//按键状态
{
	KEY_PRESS=0,
	KEY_RELEASE,
	KEY_NONE
}KeyStateType;



typedef enum//键值
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

typedef struct//开关定义
{
	uint8_t State;
	bool StateChange;
}SwitchModeType;

typedef struct//标志位
{
	uint8_t ActionOnOrOffLine;//0:设备状态未发生变化  1：设备离线操作  2：设备上线操作
	
	bool SendHeartbeat;//true：发送心跳包
	bool CPUOffLine;//离线标志位  false：在线  true：离线
	bool KeyFNFun;//FN组合键功能标志位 false:FN无效   true：FN有效 
	bool KeyScan;//按键轮询标志位  true：轮询按键
	bool PasswordEnable;//锁键标志位  false：禁用密码   true：使用密码
	bool PasswordChange;//更改密码标志位  false：不需更改   true：需要更改
	bool PasswordLock;//密码锁定标志位    false：解锁       true：锁定
	bool WarningErr;//警告故障标志	     false：无故障     true：有故障
	bool SeriousErr;//严重故障标志	     false：无故障     true：有故障
	bool LedLockBlink;//锁键指示灯闪烁    false：不闪烁     true：闪烁
	bool KeyNone;//15min内无按键按下标志位 false：有按键按下 true：无按键按下
	bool LedRefresh;//刷新指示灯标志位     true：刷新
	bool LedAllOn;	//LED全点亮标志位      true：全部点亮
	bool Wait;//等待状态	  true：进入等待状态    false：退出等待状态
    bool Ring;//响铃标志位 true：响铃   
	bool CanDataRx;//接收数据标志位 true：接收到数据
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

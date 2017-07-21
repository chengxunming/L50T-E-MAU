#include "global.h"


/* Private variables ---------------------------------------------------------*/
CanRxMsg RxMessage;					//CAN数据接收缓冲区
uint8_t Receive_Buf[10];
uint8_t TxBuff[10];

volatile uint8_t key_filter[12]={0,0,0,0,0,0,0,0,0,0,0,0};//按键延时消抖计数
volatile uint8_t switch_filter[2]={0,0};
volatile bool key_IsPress[12]={0,0,0,0,0,0,0,0,0,0,0,0};//按键状态

volatile _Bool Led_Status[12] = {0};

static uint8_t TimeCnt_GetCPUHeart=0;//获取CPU心跳包间隔超时计数
static uint8_t TimeCnt_KeyFN=0;//FN组合功能键超时计数
static uint8_t TimeCnt_LedRun=0;//运行指示灯闪烁频率
static uint16_t TimeCnt_KeyNone=0;//解锁后无按键触发计数

volatile uint8_t MAUFunFlag;//MAU功能码


uint8_t password;//锁键密码

volatile FlagType gFlag;
volatile LED LEDStatus;
volatile KeyType KeyHandle;
volatile SwitchModeType SwitchMode[2];


void GetLedStatus(void);

////////////////////////////////////////////////Flash存取功能函数///////////////////////////////////////////////////////////
//
void FLASH_PasswordSave(uint8_t value)
{
	FLASH_Unlock();//解锁

	FLASH_ErasePage(FLASHPASSWORD_ADDR);//删
	FLASH_ProgramHalfWord(FLASHPASSWORD_ADDR,value);

	FLASH_Lock();//锁
}
uint8_t FLASH_PasswordGet(void)
{
	uint8_t PWval;
	PWval= *(uint8_t*)FLASHPASSWORD_ADDR;
	return PWval;
}

////////////////////////////////////////////////CAN通信功能函数///////////////////////////////////////////////////////////
/************************************************
             CAN数据发送
***********************************************/
//发送CAN数据帧--单帧
uint8_t Send_CAN_DataFrame_Single(uint8_t *ptr, uint8_t len) //ptr: 数据指针. len: 数据长度
{
	uint16_t i=0;
	uint16_t FuncNumber;//功能码
	uint8_t mailbox_num, can_tx_fail=0,FuncH,FuncL;
	uint32_t SENDID;
	CanTxMsg TxMessage;

	SENDID = (0x01<<7) | MAU_CAN_ID; //将ID.25设置为1，1为发送地址，0为接收地址

	switch (MAUFunFlag)
	{
		case 21:
			FuncNumber = 21;//MLMS 外灯打开、关闭
			break;
		case 22:
			FuncNumber = 22;//MMAS 手动/自动
			break;
		case 25:
			FuncNumber = 25;//MKYS 按键命令
			break;
		case 26:
			FuncNumber = 26;//MAVS 版本信息
			break;
		default:
			break;
	}

	FuncH = (FuncNumber&0xFF00)>>8;
	FuncL = (uint8_t)(FuncNumber&0x00FF);

	TxMessage.StdId = SENDID;
	TxMessage.IDE = CAN_ID_STD;//CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;

	if ((len<=8)&&(len>0))
	{
		TxMessage.DLC = len+3;

		TxMessage.Data[0] = 0x00;   //开始帧,单帧传输
		TxMessage.Data[1] = FuncL;
		TxMessage.Data[2] = FuncH;

		for (i=0; i<len; i++)
		{
			TxMessage.Data[i+3] = *ptr++;
		}
	}
	else
	{
		TxMessage.DLC = 0;
	}

	mailbox_num = CAN_Transmit(CAN1, &TxMessage);

	while ((CAN_TransmitStatus(CAN1, mailbox_num) != CANTXOK)&&(i<2000)) //发送状态,等到发送完成
	{
		i++;
	}
	if (i==2000)
	{
		CAN_CancelTransmit(CAN1, mailbox_num);//超时关闭发送
		can_tx_fail = 1;
	}
	return can_tx_fail;
}
//发送CAN数据帧--多帧数据，10个信息字节，共2帧
uint8_t Send_CAN_DataFrame_MAU(uint8_t *ptr) //ptr: 数据指针
{
	uint16_t i=0;
	uint16_t FuncNumber;//功能码
	uint8_t mailbox_num, can_tx_fail=0,FuncH,FuncL;
	uint32_t SENDID;
//	uint16_t n,m;
	CanTxMsg TxMessage;
	if(gFlag.CPUOffLine!=true)
	{
		SENDID = (0x01<<7) | MAU_CAN_ID;//将ID.25设置1
		switch (MAUFunFlag)
		{
			case 21:
				FuncNumber = 21;//MLMS 外灯打开、关闭
				break;
			case 22:
				FuncNumber = 22;//MMAS 手动/自动
				break;
			case 25:
				FuncNumber = 25;//MKYS 按键命令
				break;
			case 26:
				FuncNumber = 26;//MAVS 版本信息
				break;
			default:
				break;
		}
	//	FuncNumber = 501+(MAU_CAN_ID-97);//设置功能编码501号????

		FuncH = (FuncNumber&0xFF00)>>8;
		FuncL = (uint8_t)(FuncNumber&0x00FF);

		TxMessage.StdId = SENDID;
		TxMessage.IDE = CAN_ID_STD;//CAN_ID_EXT;
		TxMessage.RTR = CAN_RTR_DATA;

		//frame 1
		TxMessage.DLC = 8;
		TxMessage.Data[0] = 0x01;   //开始帧
		TxMessage.Data[1] = FuncL;
		TxMessage.Data[2] = FuncH;
		TxMessage.Data[3] = *ptr++;
		TxMessage.Data[4] = *ptr++;
		TxMessage.Data[5] = *ptr++;
		TxMessage.Data[6] = *ptr++;
		TxMessage.Data[7] = *ptr++;

		mailbox_num = CAN_Transmit(CAN1, &TxMessage);
		while (mailbox_num==0x04)
		{
			mailbox_num = CAN_Transmit(CAN1, &TxMessage);
		}

		while ((CAN_TransmitStatus(CAN1, mailbox_num) != CANTXOK)&&(i<2000)) //发送状态,等到发送完成
		{
			i++;
		}
		if (i==2000)
		{
			CAN_CancelTransmit(CAN1, mailbox_num);//超时关闭发送
			can_tx_fail = 1;
		}

		//frame 2
		TxMessage.DLC = 8;
		TxMessage.Data[0] = 0x03;//结束帧
		TxMessage.Data[1] = FuncL;
		TxMessage.Data[2] = FuncH;
		TxMessage.Data[3] = *ptr++;
		TxMessage.Data[4] = *ptr++;
		TxMessage.Data[5] = *ptr++;
		TxMessage.Data[6] = *ptr++;
		TxMessage.Data[7] = *ptr++;

		mailbox_num = CAN_Transmit(CAN1, &TxMessage);
		while (mailbox_num==0x04)
		{
			mailbox_num = CAN_Transmit(CAN1, &TxMessage);
		}

		while ((CAN_TransmitStatus(CAN1, mailbox_num) != CANTXOK)&&(i<2000)) //发送状态,等到发送完成
		{
			i++;
		}
		if (i==2000)
		{
			CAN_CancelTransmit(CAN1, mailbox_num);//超时关闭发送
			can_tx_fail = 1;
		}
	}

	
	return can_tx_fail;
}
//发送CAN心跳桢
uint8_t Send_CAN_HeartbeatFrame(void)
{
	uint16_t i=0;
	uint8_t mailbox_num, can_tx_fail=0;
	uint32_t SENDID;
	CanTxMsg TxMessage;

	SENDID = (0x01<<10)|(0x01<<7) | MAU_CAN_ID;//将ID.28置1和ID.25置1,ID28数据帧类型设置，1为心跳帧，0为信息帧

	TxMessage.StdId = SENDID;
	TxMessage.IDE = CAN_ID_STD;   //标准ID
	TxMessage.RTR = CAN_RTR_DATA;

	TxMessage.DLC = 0;

	mailbox_num = CAN_Transmit(CAN1, &TxMessage);


	while ((CAN_TransmitStatus(CAN1, mailbox_num) != CANTXOK)&&(i<2000))   //发送状态,等到发送完成
	{
		i++;
	}
	if (i==2000)
	{
		CAN_CancelTransmit(CAN1, mailbox_num);   //超时关闭发送
		can_tx_fail = 1;
	}
	return can_tx_fail;
}

/************************************************
             CPU获取信息解析
***********************************************/
//介入密码及使能命令,命令0x0B(11)
void PASC(void)
{
	uint8_t i;
	uint8_t Signal = 0x00;
	Signal = Receive_Buf[3];
	
	
	i=(Signal & 0x0F);
	
	if( (password & 0x11) !=(Signal & 0x11) )//密码变更
	{
		password = (Signal & 0x11);//更新密码
		gFlag.PasswordChange=true;//密码改变,标志置1
	}
	else
	{
		gFlag.PasswordChange=false;//密码不变,标志置0
	}
	
	if ((password & 0x10) == 0x10)
	{
		gFlag.PasswordEnable=true;//锁键密码使能
		gFlag.PasswordLock=true;//MAU介入密码标志置1
		LEDStatus.Lock = 1;//开锁键指示灯
	}
	else if((password & 0x10) == 0x00)
	{
		gFlag.PasswordEnable=false;//无锁键密码
		gFlag.PasswordLock=false;//MAU介入密码标志置0
		LEDStatus.Lock = 0;//关锁键指示灯
	}
	else if((password & 0x01) == 0x01)
	{
		gFlag.Wait=true;//进入等待状态
		LEDStatus.Wait=1;//打开等待指示灯
		
	}
	else if((password & 0x01) == 0x00)
	{
		gFlag.Wait=false;//退出等待状态
		LEDStatus.Wait=0;//关闭等待指示灯
	}
	else
		memset(Receive_Buf,0x00,10);
}

//警告故障状态命令,命令0x141(321)
void WARC(void)
{
	uint8_t Signal = 0x00;
	Signal = Receive_Buf[3];

	if (Signal == 0xAA)
	{//信号机存在警告故障
		gFlag.WarningErr = true;  //故障标志设1
		LEDStatus.Fault=1;//故障灯打开
	}
	else if (Signal == 0x55)
	{//信号机不存在警告故障
		gFlag.WarningErr = false; //故障标志设0
		LEDStatus.Fault = 0;
	}
	else
		memset(Receive_Buf,0x00,10);
}

//严重故障状态,命令0x142(322)
void FATC(void)
{
	uint8_t Signal = 0x00;
	Signal = Receive_Buf[3];

	if (Signal == 0xAA)
	{//信号机存在严重故障
		gFlag.SeriousErr = true;  //故障标志设1     
		LEDStatus.Fault=1;//故障灯打开
	}
	else if (Signal == 0x55)
	{//信号机不存在严重故障
		gFlag.WarningErr = false; //故障标志设0      
		LEDStatus.Fault = 0;//故障灯关闭
	}
	else
		memset(Receive_Buf,0x00,10);
}

//外灯通道输出开关状态,命令0x143(323)
void LMSC(void)
{
	uint8_t Signal = 0x00;
	Signal = Receive_Buf[3];

	if (Signal == 0xAA)
	{//信号机通道输出为打开状态
		LEDStatus.Open = 1;//外灯打开指示灯点亮
	}
	else if (Signal == 0x55)
	{//信号机通道输出为关闭状态
		LEDStatus.Open = 0;//外灯打开指示灯关灭
	}
	else
		memset(Receive_Buf,0x00,10);
}

//控制源状态,命令0x144(324)
void CSOC(void)
{
	uint8_t Signal = 0x00;
	Signal = Receive_Buf[3];
	if(Signal==0xaa)//自动状态
	{
		LEDStatus.Auto=1;
		LEDStatus.Manual=0;
		gFlag.Ring=false;
	}
	else if(Signal==0x55)//其它状态
	{
		LEDStatus.Auto=0;
		if(SIGNAL2==0)
		{
			LEDStatus.Manual=1;
			gFlag.Ring=true;
		}
		else
		{
			LEDStatus.Manual=0;
			gFlag.Ring=false;
		}
	}
	else
		memset(Receive_Buf,0x00,10);
}
//闪灯状态状态,命令0x145(325) 黄闪
void FLSC(void)
{
	uint8_t Signal = 0x00;
	Signal = Receive_Buf[3];

	if (Signal == 0xAA)
	{
		LEDStatus.Y_Flash = 1;
	}
	else if (Signal == 0x55)
	{
		LEDStatus.Y_Flash = 0;
	}
	else
		memset(Receive_Buf,0x00,10);	
}
//信号机数字键指示状态0x147(327)
void KNMC(void)
{
	uint8_t Signal = 0x00;
	Signal = Receive_Buf[3];
	
	if( (Signal>8) && (Signal<17) )
	{
		LEDStatus.Fn=1;
		Signal-=8;
	}
	else
	{
		LEDStatus.Fn=0;
	}

	if (Signal==0x01)
	{
		LEDStatus.One = 1;
		
	}
	else
	{
		LEDStatus.One = 0;
	}

	if (Signal==0x02)
	{
		LEDStatus.Two = 1;

	}
	else
	{
		LEDStatus.Two = 0;
	}

	if (Signal==0x03)
	{
		LEDStatus.Three = 1;
	}
	else
	{
		LEDStatus.Three = 0;
	}

	if (Signal==0x04)
	{
		LEDStatus.Four = 1;
	}
	else
	{
		LEDStatus.Four = 0;
	}

	if (Signal==0x05)
	{
		LEDStatus.Five = 1;
	}
	else
	{
		LEDStatus.Five = 0;
	}

	if (Signal==0x06)
	{
		LEDStatus.Six = 1;
	}
	else
	{
		LEDStatus.Six = 0;
	}
	if (Signal==0x07)
	{
		LEDStatus.Seven = 1;
	}
	else
	{
		LEDStatus.Seven = 0;
	}

	if (Signal==0x08)
	{
		LEDStatus.Eight = 1;
	}
	else
	{
		LEDStatus.Eight = 0;
	}
	
	GetLedStatus();

	memset(Receive_Buf,0x00,10);
}
//信号机步进键指示状态0x148(328)
void KSPC(void)
{
	uint8_t Signal = 0x00;
	Signal = Receive_Buf[3];

	if (Signal == 0xAA)
	{
		LEDStatus.Step = 1;
	}
	else if (Signal == 0x55)
	{
		LEDStatus.Step = 0;
	}
	else
		memset(Receive_Buf,0x00,10);
}
//信号机全红指示状态0x149(329)
void KARC(void)
{
	uint8_t Signal = 0x00;
	Signal = Receive_Buf[3];

	if (Signal == 0xAA)
	{
		LEDStatus.Allred = 1;
		GetLedStatus();
	}
	else if (Signal == 0x55)
	{
		LEDStatus.Allred = 0;
		GetLedStatus();
	}
	else
		memset(Receive_Buf,0x00,10);
}

//MAU控制信号机开关灯命令0x15(21)
void MLMS(void)
{
	if (!SIGNAL1)
	{
		TxBuff[0] = 0xAA;
		LEDStatus.Open = 1;
	}
	else
	{
		TxBuff[0] = 0x55;
		LEDStatus.Open = 0;
	}
		

	MAUFunFlag=21;

	Send_CAN_DataFrame_Single(TxBuff,1);
}
//MAU手控/自动切换命令0x16(22)
void MMAS(void)
{
	if (!SIGNAL2)
	{
		TxBuff[0] = 0xAA;	//手控功能使能
		LEDStatus.Manual = 1;
//		gFlag.Ring=true;
	}
	else
	{
		TxBuff[0] = 0x55;//自动功能使能
		LEDStatus.Manual = 0;
//		gFlag.Ring=false;
	}
	
	MAUFunFlag=22;
	Send_CAN_DataFrame_Single(TxBuff,1);
}

//MAU按键控制命令0x19(25)
void MKYS(uint8_t Key,uint8_t KeyState)
{
//	Key = Read_KeyValue();
	switch (Key)
	{
	case KEY_YF:case KEY_YFFN:
		TxBuff[0] = 0x40;//黄闪
		break;
	case KEY_AR:case KEY_ARFN:
		TxBuff[0] = 0x80;//全红
		break;
	case KEY_STEP:case KEY_STEPFN:
		TxBuff[0] = 0x00;//步进
		break;
	case KEY_NUM1:
		TxBuff[0] = 0x01;//数字1
		break;
	case KEY_NUM2:
		TxBuff[0] = 0x02;//数字2
		break;
	case KEY_NUM3:
		TxBuff[0] = 0x03;//数字3
		break;
	case KEY_NUM4:
		TxBuff[0] = 0x04;//数字4
		break;
	case KEY_NUM5:
		TxBuff[0] = 0x05;//数字5
		break;
	case KEY_NUM6:
		TxBuff[0] = 0x06;//数字6
		break;
	case KEY_NUM7:
		TxBuff[0] = 0x07;//数字7
		break;
	case KEY_NUM8:
		TxBuff[0] = 0x08;//数字8
		break;
	case KEY_NUM9:
		TxBuff[0] = 0x09;//数字9
		break;
	case KEY_NUM10:
		TxBuff[0] = 0x0a;//数字10
		break;
	case KEY_NUM11:
		TxBuff[0] = 0x0b;//数字11
		break;
	case KEY_NUM12:
		TxBuff[0] = 0x0c;//数字12
		break;
	case KEY_NUM13:
		TxBuff[0] = 0x0d;//数字13
		break;
	case KEY_NUM14:
		TxBuff[0] = 0x0e;//数字14
		break;
	case KEY_NUM15:
		TxBuff[0] = 0x0f;//数字15
		break;
	case KEY_NUM16:
		TxBuff[0] = 0x10;//数字16
		break;

	default:
		memset(Receive_Buf,0x00,10);
		break;
	}
	
	if(KeyState==KEY_PRESS)
	{
		TxBuff[0] &=0xDF;
	}
	else
	{
		TxBuff[0] |=0x20;
	}

	MAUFunFlag=25;

	Send_CAN_DataFrame_Single(TxBuff,1);
}
//MAU软硬件版本信息0x1a(26)
void MAVS(void)
{
	TxBuff[0] = (HardVersions & 0xff);	//印制板版本
	TxBuff[1] = (SoftVersions & 0xff);	//固件版本
	MAUFunFlag=26;

	Send_CAN_DataFrame_Single(TxBuff,2);
}
/********************************************************************
             CAN接收数据针分析解析
********************************************************************/

//接收CAN数据帧，作功能识别
void Receive_CAN_DataFrame(CanRxMsg* canRx,uint8_t num)
{
	uint8_t i;
	uint16_t ID_temp;
	for (i=0; i<num; i++)
	{
		Receive_Buf[i] = canRx->Data[i];
	}
	ID_temp = Receive_Buf[2]<<8 | Receive_Buf[1];
	switch (ID_temp)
	{
	case 11:
		PASC();//介入密码及使能命令,命令0x0B(11)

		break;
	case 321:
		WARC();//警告故障状态,命令0x141(321)

		break;
	case 322:
		FATC();//严重故障状态,命令0x142(322)

		break;
	case 323:
		LMSC();//外灯通道输出开关状态,命令0x143(323)

		break;
	case 324:
		CSOC();//控制源状态,命令0x144(324)

		break;
	case 325:
		FLSC();//闪灯状态,命令0x145(325)黄闪

		break;

	case 327:
		KNMC();//数字键指示状态,命令0x147(327)

		break;
	case 328:
		KSPC();//手动步进点状态,命令0x148(328)

		break;
	case 329:
		KARC();//全红状态状态,命令0x149(329)

		break;

//自行增加测试备用命令

	default:
		memset(Receive_Buf,0x00,10);
		break;
	}
}
//CAN接收数据处理
void Can_Receive_CMD(void)
{
	Receive_CAN_DataFrame(&RxMessage,RxMessage.DLC);//接收CAN数据
}
////////////////////////////////////////////////LED功能函数///////////////////////////////////////////////////////////

void GetLedStatus(void)
{
	Led_Status[0] = LEDStatus.Fn;
	Led_Status[1] = LEDStatus.Y_Flash;
	Led_Status[2] = LEDStatus.Allred;
	Led_Status[3] = LEDStatus.Step;
	Led_Status[4] = LEDStatus.One;
	Led_Status[5] = LEDStatus.Two;
	Led_Status[6] = LEDStatus.Three;
	Led_Status[7] = LEDStatus.Four;
	Led_Status[8] = LEDStatus.Five;
	Led_Status[9] = LEDStatus.Six;
	Led_Status[10] = LEDStatus.Seven;
	Led_Status[11] = LEDStatus.Eight;
}
//状态指示灯开关
void LEDALL(uint8_t Mark)
{
	//指示灯全部打开
	if (Mark)
	{
		RUNLEDON;
		
		LEDONEON;
		LEDTWOON;
		LEDTHREEON;
		LEDFOURON;
		LEDFIVEON;
		LEDSIXON;
		LEDSEVENON;
		LEDEIGHTON;
		LEDALLREDON;
		LEDYFLASHON;
		LEDFNON;

		ERRLEDON;
		KEYLOCKON;
		AUTOON;
		WAITON;
		
		MANON;
		TUNONLEDON;
		LEDSTEPON;
	}
	else  //指示灯全部关闭
	{
		RUNLEDOFF;
		
		LEDONEOFF;
		LEDTWOOFF;
		LEDTHREEOFF;
		LEDFOUROFF;
		LEDFIVEOFF;
		LEDSIXOFF;
		LEDSEVENOFF;
		LEDEIGHTOFF;
		LEDALLREDOFF;
		LEDYFLASHOFF;
		LEDFNOFF;
		
		ERRLEDOFF;
		KEYLOCKOFF;
		WAITOFF;
		AUTOOFF;
		
		MANOFF;
		TUNONLEDOFF;
		LEDSTEPOFF;

		gFlag.Ring=false;
		gFlag.WarningErr=false;
		gFlag.SeriousErr=false;
	}
}
void LEDStatusFresh(void)
{
	if(Led_Status[0])
	{
		LEDFNON;
	}
	else
		LEDFNOFF;
	
	if(Led_Status[1])
	{
		LEDYFLASHON;
	}
	else
		LEDYFLASHOFF;
	
	if(Led_Status[2])
	{
		LEDALLREDON;
	}
	else
		LEDALLREDOFF;
	
	if(Led_Status[3])
	{
		LEDSTEPON;
	}
	else
		LEDSTEPOFF;
	
	if(Led_Status[4])
	{
		LEDONEON;
	}
	else
		LEDONEOFF;
	
	if(Led_Status[5])
	{
		LEDTWOON;
	}
	else
		LEDTWOOFF;
	
	if(Led_Status[6])
	{
		LEDTHREEON;
	}
	else
		LEDTHREEOFF;
	
	if(Led_Status[7])
	{
		LEDFOURON;
	}
	else
		LEDFOUROFF;
	
	if(Led_Status[8])
	{
		LEDFIVEON;
	}
	else
		LEDFIVEOFF;
	
	if(Led_Status[9])
	{
		LEDSIXON;
	}
	else
		LEDSIXOFF;
	
	if(Led_Status[10])
	{
		LEDSEVENON;
	}
	else
		LEDSEVENOFF;
	
	if(Led_Status[11])
	{
		LEDEIGHTON;
	}
	else
		LEDEIGHTOFF;
}
void StatusLedFresh(void)
{
	if(LEDStatus.Fault) ERRLEDON; else ERRLEDOFF;
	if(LEDStatus.Lock) KEYLOCKON; else KEYLOCKOFF;
	if(LEDStatus.Wait) WAITON; else WAITOFF;
	if(LEDStatus.Auto) AUTOON; else AUTOOFF;
	
	if(LEDStatus.Open) TUNONLEDON; else TUNONLEDOFF;
	if(LEDStatus.Manual) MANON; else MANOFF;
	
	if(LEDStatus.Run)RUNLEDON;else RUNLEDOFF;
}
//解锁过程中闪灯
void LedBLink_KeyLock(void)
{
	static uint8_t TimeCnt_LedBlink=0;
	if(gFlag.LedLockBlink)
	{
		TimeCnt_LedBlink++;
		if(TimeCnt_LedBlink==20)
		{
			TimeCnt_LedBlink=0;
			LEDStatus.Lock=!LEDStatus.Lock;
		}
	}
}
void RUNLED_FLASH(void)   //sysflag;系统状态LED闪烁
{
	if(TimeCnt_LedRun>=5)
	{
		TimeCnt_LedRun=0;
		LEDStatus.Run=!LEDStatus.Run;
	}
	
	
}
//刷新LED
void LED_REFRESH(void)
{
	LEDStatusFresh();
	StatusLedFresh();
}
///////////////////////////////////////////////////////////////////按键、开关功能函数//////////////////////////////////////////////////////////////////////////////////////
//按键事件处理
void Key_Event(uint8_t keyNum,KeyStateType KeyState)//keyNum:按键编号      KeyState：KEY_RELEASE：释放按键  KEY_PRESS：按下按键
{
	KeyHandle.KeyVal=keyNum;
	KeyHandle.KeyState=KeyState;
}
//读取按键IO数据
uint8_t KeyValGet(uint8_t keyNum)
{
	uint8_t key_state;
	switch(keyNum)
	{
		case KEY_FN:
			key_state=KEY_NUMFN_IN;
			break;
		case KEY_YF:
			key_state=KEY_NUMYF_IN;
			break;
		case KEY_AR:
			key_state=KEY_NUMAR_IN;
			break;
		case KEY_STEP:
			key_state=KEY_NUMSTEP_IN;
			break;
		case KEY_NUM1:
			key_state=KEY_NUM1_IN;
			break;
		case KEY_NUM2:
			key_state=KEY_NUM2_IN;
			break;
		case KEY_NUM3:
			key_state=KEY_NUM3_IN;
			break;
		case KEY_NUM4:
			key_state=KEY_NUM4_IN;
			break;
		case KEY_NUM5:
			key_state=KEY_NUM5_IN;
			break;
		case KEY_NUM6:
			key_state=KEY_NUM6_IN;
			break;
		case KEY_NUM7:
			key_state=KEY_NUM7_IN;
			break;
		case KEY_NUM8:
			key_state=KEY_NUM8_IN;
			break;
	}
	return key_state;
}
//检测每个按键
void KEY_Check(uint8_t keyNum)
{
	static uint8_t keyNum_Press=31;
	uint8_t CurrentKeyVal=1;
	if( (keyNum_Press!=31) && (keyNum_Press!=keyNum) )return;//如果有按键没有释放，等待按键释放后再检测其它按键
	CurrentKeyVal=KeyValGet(keyNum);
	if(CurrentKeyVal==0)//检测到按键按下
	{
		if(key_IsPress[keyNum]==false)
		{
			if(key_filter[keyNum]>=2)//20ms按键去抖
			{
				keyNum_Press=keyNum;
				key_IsPress[keyNum]=true;
				Key_Event(keyNum,KEY_PRESS);
			}
			else
			{
				key_filter[keyNum]++;
			}
		}
	}
	else
	{
		if(key_IsPress[keyNum]==true)
		{
			if(key_filter[keyNum]==0)
			{
				keyNum_Press=31;
				key_IsPress[keyNum]=false;
				Key_Event(keyNum,KEY_RELEASE);
			}
			else
			{
				key_filter[keyNum]--;
			}
		}
	}
}

//轮询所有按键
void KEY_Scan(void)
{
	uint8_t i;
	KeyHandle.KeyVal=31;
	KeyHandle.KeyState=KEY_NONE;
	for(i=0;i<12;i++)
	{
		KEY_Check(i);
	}
}
//检测单个开关
void SWITCH_Check(uint8_t keyNum)
{
	uint8_t CurrentSwithVal;
	if(keyNum==0)
	{
		CurrentSwithVal=!SIGNAL1;
	}
	else if(keyNum==1)
	{
		CurrentSwithVal=!SIGNAL2;
	}
	
	if(CurrentSwithVal!=SwitchMode[keyNum].State)
	{
		if(switch_filter[keyNum]>=2)
		{
			switch_filter[keyNum]=0;
			if(CurrentSwithVal==0)SwitchMode[keyNum].State=OFF;
			else SwitchMode[keyNum].State=ON;
			SwitchMode[keyNum].StateChange=true;
		}
		else switch_filter[keyNum]++;
	}
	
}
//轮询所有开关
void SWITCH_Scan(void)
{
	uint8_t i;
	for(i=0;i<2;i++)
	{
		SWITCH_Check(i);
	}
}
//解锁
void KeyPassword_Unlock(void)
{
	static uint8_t PasswordBit=0;
	uint8_t keyBitVal;
	uint8_t passwordVal=0x1f;//password;
	if(gFlag.PasswordChange)
	{
		PasswordBit=0;
	}
	if( (KeyHandle.KeyVal>=0) && (KeyHandle.KeyVal<=10) )//检测到按键
	{
		if(KeyHandle.KeyState==KEY_PRESS)//按键按下
		{
			gFlag.LedLockBlink=true;
			PasswordBit++;
			keyBitVal=KeyHandle.KeyVal;
			
			Led_Status[keyBitVal] = !Led_Status[keyBitVal];
		}
		else if(KeyHandle.KeyState==KEY_RELEASE)//释放按键
		{
			keyBitVal=KeyHandle.KeyVal;
			Led_Status[keyBitVal] = !Led_Status[keyBitVal];
			keyBitVal-=3;
			if (((  passwordVal>>(4-PasswordBit) ) & 0x01) != keyBitVal )
			{//判断键入密码是否与存储密码相同
				PasswordBit=0;
				gFlag.LedLockBlink=false;
				LEDStatus.Lock=1;
			}
			
			if (PasswordBit==4)
			{
				TimeCnt_KeyNone=0;
				gFlag.PasswordLock=false;
				gFlag.LedLockBlink=false;
				LEDStatus.Lock=false;
				PasswordBit=0;
			}
		}
	}
}

//按键命令
void KeyCMD(void)
{
	static uint8_t KeyFN_PressCnt=0;
	uint8_t KeyValCmd=31;
	if(KeyHandle.KeyVal!=31)
	{
		TimeCnt_KeyNone=0;//有按键按下则清零无按键触发计时
		Led_Status[KeyHandle.KeyVal] = !Led_Status[KeyHandle.KeyVal];
		
		if(KeyHandle.KeyVal==0 && KeyHandle.KeyState==KEY_RELEASE && KeyFN_PressCnt==0)//检测到FN按键
		{
			KeyFN_PressCnt++;
			gFlag.KeyFNFun=true;
			return;
		}
		//计算键值
		if(gFlag.KeyFNFun)
		{
			KeyValCmd=KeyHandle.KeyVal+12;
			if(KeyHandle.KeyState==KEY_RELEASE)//释放按键
			{
				Led_Status[0]=0;
				gFlag.KeyFNFun=false;
				KeyFN_PressCnt=0;
			}
		}
		else 
		{
			KeyValCmd=KeyHandle.KeyVal;
			KeyFN_PressCnt=0;
		}
		//按键命令
		if(KeyValCmd==KEY_TEST)
		{
			if(KeyHandle.KeyState==KEY_RELEASE)//释放按键
			{
				gFlag.LedAllOn=true;
			}
		}
		else if(KeyValCmd!=KEY_FN)
		{
			MKYS(KeyValCmd,KeyHandle.KeyState);
		}
		
	}
}

void SwitchCMD(void)
{
	uint8_t i;
	for(i=0;i<2;i++)
	{
		if(SwitchMode[i].StateChange)
		{
			SwitchMode[i].StateChange=false;
			if(i==0)
			{
				MLMS();//外灯开关灯命令
			}
			else if(i==1)
			{
				MMAS();//MAU手控/自动切换命令
			}
		}
	}
}

//键值处理
void KEY_Process(void)
{
	SwitchCMD();
	if(gFlag.KeyNone)
	{
		gFlag.KeyNone=false;
		gFlag.PasswordLock=true;
		LEDStatus.Lock=1;
	}
	if(gFlag.PasswordLock)//锁键
	{
		KeyPassword_Unlock();
		LedBLink_KeyLock();
	}
	else//未锁键
	{
		if(gFlag.Wait!=true)KeyCMD();
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//设备离线、上线功能函数
void Device_StateChange(void)
{
	if(gFlag.ActionOnOrOffLine==1)//设备离线
	{
		gFlag.ActionOnOrOffLine=0;
		
		if(gFlag.PasswordEnable)
		{
			gFlag.PasswordLock=true;
			gFlag.Wait=true;
			LEDStatus.Lock=1;
			LEDStatus.Wait=1;
		}
	}
	else if(gFlag.ActionOnOrOffLine==2)//设备上线
	{
		gFlag.ActionOnOrOffLine=0;
		
		//发送状态信息
		MLMS();
		MMAS();
		MAVS();
	}
}




/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

void USB_LP_CAN1_RX0_IRQHandler(void)
{
	uint8_t i;

	//初始化接收
	RxMessage.StdId = 0x00;  //标准标识符（取值 0~0x7FF）
	RxMessage.ExtId = 0x00;  //扩展标识符（取值 0~0x3FFFF）
	RxMessage.IDE = 0x00;    //标识符的类型
	RxMessage.DLC = 0x00;    //消息的帧长度（取值 0~0x8）
	RxMessage.FMI = 0x00;    //要通过的过滤器索引，这些消息存储于邮箱中（取值为0~0xFF）
	for (i=0; i<8; i++)
	{
		RxMessage.Data[i] = 0x00;//待传输数据（取值为0~0xFF）
	}
	//接收总线上的数据释放FIFO
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//接收
	//处理接收的数据
	if (RxMessage.IDE==CAN_ID_STD)
	{
		if (RxMessage.DLC>0)
		{
			gFlag.CanDataRx=true;
//			Receive_CAN_DataFrame(&RxMessage,RxMessage.DLC);//接收CAN数据
		}
		if (RxMessage.DLC==0)
		{
			
			if(gFlag.CPUOffLine)
			{
				gFlag.ActionOnOrOffLine=2;//设备上线标志
			}
			gFlag.CPUOffLine = false;//设备离线状态清0
//			RUNledstatus = !RUNledstatus;
			TimeCnt_GetCPUHeart = 0;//接收心跳包计数超时清0
		}

	}
}



//------------------------Timer2 IRQ----------------100ms定时---------------------------------
void TIM2_IRQHandler(void)
{
	static uint8_t TimeCnt_Heartsend=0;
	static uint8_t TimeCnt_Beep=0;
	
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//TIM2 interrupt has occured
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);//Clear interrupt pending bit
		
		/*******************************************************************************
				                          心跳状态判定
		********************************************************************************/
		if (TimeCnt_Heartsend<200)
		{//心跳帧循环发送200次以内
			TimeCnt_Heartsend++;
			if (TimeCnt_Heartsend == 10)
			{//等待10次发送1次，即周期1000ms
				TimeCnt_Heartsend=0;  //发送次数清零
				gFlag.SendHeartbeat = true;//发送心跳标志设1
			}
			
		}
		else TimeCnt_Heartsend=0;
		
      	/*******************************************************************************
				                          离线状态判断
		********************************************************************************/
		TimeCnt_GetCPUHeart++;
		if(TimeCnt_GetCPUHeart==12)//超过1.2ms
		{
			if(gFlag.CPUOffLine==false)
			{
				gFlag.ActionOnOrOffLine=1;
				gFlag.CPUOffLine=true;
			}
			
		}
		
		/*******************************************************************************
				                         FN组合键功能状态
		********************************************************************************/
		if(gFlag.KeyFNFun)
		{
			TimeCnt_KeyFN++;
			if(TimeCnt_KeyFN>=30)
			{
				TimeCnt_KeyFN=0;
				gFlag.KeyFNFun=false;
			}
		}
		
		/*******************************************************************************
				                        锁键解锁后计时，15min内无按键触发则锁键
		********************************************************************************/
		if(gFlag.PasswordEnable==true && gFlag.PasswordLock==false)
		{
			TimeCnt_KeyNone++;
			if(TimeCnt_KeyNone>=9000)
			{
				TimeCnt_KeyNone=0;
				gFlag.KeyNone=true;
			}
		}
		else
		{
			TimeCnt_KeyNone=0;
		}
		
		/*******************************************************************************
				                       运行指示灯闪烁时间计算
		********************************************************************************/
		TimeCnt_LedRun++;
		
		/*******************************************************************************
				                       响铃
		********************************************************************************/
		if(gFlag.Ring)
		{
			TimeCnt_Beep++;
			if (TimeCnt_Beep==1)
				TIM_Cmd(TIM3, ENABLE);											/* 开启响铃 */
			else if (TimeCnt_Beep==3) //time=ring_time*100ms(3s响一次)
			{
				TIM_Cmd(TIM3, DISABLE);
				BEEPOFF;
			}
			else if(TimeCnt_Beep>=30)TimeCnt_Beep=0;
		}
		else 
		{
			TIM_Cmd(TIM3, DISABLE);												/* 关闭响铃*/
			BEEPOFF;
			TimeCnt_Beep=0;
		}

	}
}
//------------------------Timer3 IRQ-------100us定时器-MAU蜂鸣器使能定时器-----------------------------------------
extern uint32_t time_count_test;
void TIM3_IRQHandler(void)
{
	static uint8_t TimeCnt_Ring=0;
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)//TIM2 interrupt has occured
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);//Clear interrupt pending bit
			
		//蜂鸣器打开，频率2kHz周期T=100us*5=500us
		//蜂鸣器打开，频率3.3kHz周期T=100us*3=300us
		//蜂鸣器打开，频率4kHz周期T=50us*5=250us
		/***************************************************
		            			_   _
		                 | | | |
		         ________| |_| |_______
		****************************************************/
		TimeCnt_Ring++;
		if (TimeCnt_Ring==1)
			BEEPON;
		else if (TimeCnt_Ring==3)
			BEEPOFF;
		else if (TimeCnt_Ring==6)
			TimeCnt_Ring=0;	
		
	}
}
//------------------------Timer4 IRQ-----10ms定时器--按键&拨动开关状态采集专用------------------------------------------
void TIM4_IRQHandler(void)
{
	static uint16_t TimeCnt_LedAllOn=0;
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)//TIM2 interrupt has occured
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);//Clear interrupt pending bit
		/*******************************************************************************
				                        按键轮询标志位置1
		********************************************************************************/
		gFlag.KeyScan=true;
		
		/*******************************************************************************
				                        刷新LED
		********************************************************************************/
		if(gFlag.LedAllOn)
		{
			TimeCnt_LedAllOn++;
			if(TimeCnt_LedAllOn>=300)//3S
			{
				TimeCnt_LedAllOn=0;
				gFlag.LedAllOn=false;
			}
		}
		else
		{
			gFlag.LedRefresh=true;
		}
		
	}

}

#include "global.h"


/* Private variables ---------------------------------------------------------*/
CanRxMsg RxMessage;					//CAN���ݽ��ջ�����
uint8_t Receive_Buf[10];
uint8_t TxBuff[10];

volatile uint8_t key_filter[12]={0,0,0,0,0,0,0,0,0,0,0,0};//������ʱ��������
volatile uint8_t switch_filter[2]={0,0};
volatile bool key_IsPress[12]={0,0,0,0,0,0,0,0,0,0,0,0};//����״̬

volatile _Bool Led_Status[12] = {0};

static uint8_t TimeCnt_GetCPUHeart=0;//��ȡCPU�����������ʱ����
static uint8_t TimeCnt_KeyFN=0;//FN��Ϲ��ܼ���ʱ����
static uint8_t TimeCnt_LedRun=0;//����ָʾ����˸Ƶ��
static uint16_t TimeCnt_KeyNone=0;//�������ް�����������

volatile uint8_t MAUFunFlag;//MAU������


uint8_t password;//��������

volatile FlagType gFlag;
volatile LED LEDStatus;
volatile KeyType KeyHandle;
volatile SwitchModeType SwitchMode[2];


void GetLedStatus(void);

////////////////////////////////////////////////Flash��ȡ���ܺ���///////////////////////////////////////////////////////////
//
void FLASH_PasswordSave(uint8_t value)
{
	FLASH_Unlock();//����

	FLASH_ErasePage(FLASHPASSWORD_ADDR);//ɾ
	FLASH_ProgramHalfWord(FLASHPASSWORD_ADDR,value);

	FLASH_Lock();//��
}
uint8_t FLASH_PasswordGet(void)
{
	uint8_t PWval;
	PWval= *(uint8_t*)FLASHPASSWORD_ADDR;
	return PWval;
}

////////////////////////////////////////////////CANͨ�Ź��ܺ���///////////////////////////////////////////////////////////
/************************************************
             CAN���ݷ���
***********************************************/
//����CAN����֡--��֡
uint8_t Send_CAN_DataFrame_Single(uint8_t *ptr, uint8_t len) //ptr: ����ָ��. len: ���ݳ���
{
	uint16_t i=0;
	uint16_t FuncNumber;//������
	uint8_t mailbox_num, can_tx_fail=0,FuncH,FuncL;
	uint32_t SENDID;
	CanTxMsg TxMessage;

	SENDID = (0x01<<7) | MAU_CAN_ID; //��ID.25����Ϊ1��1Ϊ���͵�ַ��0Ϊ���յ�ַ

	switch (MAUFunFlag)
	{
		case 21:
			FuncNumber = 21;//MLMS ��ƴ򿪡��ر�
			break;
		case 22:
			FuncNumber = 22;//MMAS �ֶ�/�Զ�
			break;
		case 25:
			FuncNumber = 25;//MKYS ��������
			break;
		case 26:
			FuncNumber = 26;//MAVS �汾��Ϣ
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

		TxMessage.Data[0] = 0x00;   //��ʼ֡,��֡����
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

	while ((CAN_TransmitStatus(CAN1, mailbox_num) != CANTXOK)&&(i<2000)) //����״̬,�ȵ��������
	{
		i++;
	}
	if (i==2000)
	{
		CAN_CancelTransmit(CAN1, mailbox_num);//��ʱ�رշ���
		can_tx_fail = 1;
	}
	return can_tx_fail;
}
//����CAN����֡--��֡���ݣ�10����Ϣ�ֽڣ���2֡
uint8_t Send_CAN_DataFrame_MAU(uint8_t *ptr) //ptr: ����ָ��
{
	uint16_t i=0;
	uint16_t FuncNumber;//������
	uint8_t mailbox_num, can_tx_fail=0,FuncH,FuncL;
	uint32_t SENDID;
//	uint16_t n,m;
	CanTxMsg TxMessage;
	if(gFlag.CPUOffLine!=true)
	{
		SENDID = (0x01<<7) | MAU_CAN_ID;//��ID.25����1
		switch (MAUFunFlag)
		{
			case 21:
				FuncNumber = 21;//MLMS ��ƴ򿪡��ر�
				break;
			case 22:
				FuncNumber = 22;//MMAS �ֶ�/�Զ�
				break;
			case 25:
				FuncNumber = 25;//MKYS ��������
				break;
			case 26:
				FuncNumber = 26;//MAVS �汾��Ϣ
				break;
			default:
				break;
		}
	//	FuncNumber = 501+(MAU_CAN_ID-97);//���ù��ܱ���501��????

		FuncH = (FuncNumber&0xFF00)>>8;
		FuncL = (uint8_t)(FuncNumber&0x00FF);

		TxMessage.StdId = SENDID;
		TxMessage.IDE = CAN_ID_STD;//CAN_ID_EXT;
		TxMessage.RTR = CAN_RTR_DATA;

		//frame 1
		TxMessage.DLC = 8;
		TxMessage.Data[0] = 0x01;   //��ʼ֡
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

		while ((CAN_TransmitStatus(CAN1, mailbox_num) != CANTXOK)&&(i<2000)) //����״̬,�ȵ��������
		{
			i++;
		}
		if (i==2000)
		{
			CAN_CancelTransmit(CAN1, mailbox_num);//��ʱ�رշ���
			can_tx_fail = 1;
		}

		//frame 2
		TxMessage.DLC = 8;
		TxMessage.Data[0] = 0x03;//����֡
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

		while ((CAN_TransmitStatus(CAN1, mailbox_num) != CANTXOK)&&(i<2000)) //����״̬,�ȵ��������
		{
			i++;
		}
		if (i==2000)
		{
			CAN_CancelTransmit(CAN1, mailbox_num);//��ʱ�رշ���
			can_tx_fail = 1;
		}
	}

	
	return can_tx_fail;
}
//����CAN������
uint8_t Send_CAN_HeartbeatFrame(void)
{
	uint16_t i=0;
	uint8_t mailbox_num, can_tx_fail=0;
	uint32_t SENDID;
	CanTxMsg TxMessage;

	SENDID = (0x01<<10)|(0x01<<7) | MAU_CAN_ID;//��ID.28��1��ID.25��1,ID28����֡�������ã�1Ϊ����֡��0Ϊ��Ϣ֡

	TxMessage.StdId = SENDID;
	TxMessage.IDE = CAN_ID_STD;   //��׼ID
	TxMessage.RTR = CAN_RTR_DATA;

	TxMessage.DLC = 0;

	mailbox_num = CAN_Transmit(CAN1, &TxMessage);


	while ((CAN_TransmitStatus(CAN1, mailbox_num) != CANTXOK)&&(i<2000))   //����״̬,�ȵ��������
	{
		i++;
	}
	if (i==2000)
	{
		CAN_CancelTransmit(CAN1, mailbox_num);   //��ʱ�رշ���
		can_tx_fail = 1;
	}
	return can_tx_fail;
}

/************************************************
             CPU��ȡ��Ϣ����
***********************************************/
//�������뼰ʹ������,����0x0B(11)
void PASC(void)
{
	uint8_t i;
	uint8_t Signal = 0x00;
	Signal = Receive_Buf[3];
	
	
	i=(Signal & 0x0F);
	
	if( (password & 0x11) !=(Signal & 0x11) )//������
	{
		password = (Signal & 0x11);//��������
		gFlag.PasswordChange=true;//����ı�,��־��1
	}
	else
	{
		gFlag.PasswordChange=false;//���벻��,��־��0
	}
	
	if ((password & 0x10) == 0x10)
	{
		gFlag.PasswordEnable=true;//��������ʹ��
		gFlag.PasswordLock=true;//MAU���������־��1
		LEDStatus.Lock = 1;//������ָʾ��
	}
	else if((password & 0x10) == 0x00)
	{
		gFlag.PasswordEnable=false;//����������
		gFlag.PasswordLock=false;//MAU���������־��0
		LEDStatus.Lock = 0;//������ָʾ��
	}
	else if((password & 0x01) == 0x01)
	{
		gFlag.Wait=true;//����ȴ�״̬
		LEDStatus.Wait=1;//�򿪵ȴ�ָʾ��
		
	}
	else if((password & 0x01) == 0x00)
	{
		gFlag.Wait=false;//�˳��ȴ�״̬
		LEDStatus.Wait=0;//�رյȴ�ָʾ��
	}
	else
		memset(Receive_Buf,0x00,10);
}

//�������״̬����,����0x141(321)
void WARC(void)
{
	uint8_t Signal = 0x00;
	Signal = Receive_Buf[3];

	if (Signal == 0xAA)
	{//�źŻ����ھ������
		gFlag.WarningErr = true;  //���ϱ�־��1
		LEDStatus.Fault=1;//���ϵƴ�
	}
	else if (Signal == 0x55)
	{//�źŻ������ھ������
		gFlag.WarningErr = false; //���ϱ�־��0
		LEDStatus.Fault = 0;
	}
	else
		memset(Receive_Buf,0x00,10);
}

//���ع���״̬,����0x142(322)
void FATC(void)
{
	uint8_t Signal = 0x00;
	Signal = Receive_Buf[3];

	if (Signal == 0xAA)
	{//�źŻ��������ع���
		gFlag.SeriousErr = true;  //���ϱ�־��1     
		LEDStatus.Fault=1;//���ϵƴ�
	}
	else if (Signal == 0x55)
	{//�źŻ����������ع���
		gFlag.WarningErr = false; //���ϱ�־��0      
		LEDStatus.Fault = 0;//���ϵƹر�
	}
	else
		memset(Receive_Buf,0x00,10);
}

//���ͨ���������״̬,����0x143(323)
void LMSC(void)
{
	uint8_t Signal = 0x00;
	Signal = Receive_Buf[3];

	if (Signal == 0xAA)
	{//�źŻ�ͨ�����Ϊ��״̬
		LEDStatus.Open = 1;//��ƴ�ָʾ�Ƶ���
	}
	else if (Signal == 0x55)
	{//�źŻ�ͨ�����Ϊ�ر�״̬
		LEDStatus.Open = 0;//��ƴ�ָʾ�ƹ���
	}
	else
		memset(Receive_Buf,0x00,10);
}

//����Դ״̬,����0x144(324)
void CSOC(void)
{
	uint8_t Signal = 0x00;
	Signal = Receive_Buf[3];
	if(Signal==0xaa)//�Զ�״̬
	{
		LEDStatus.Auto=1;
		LEDStatus.Manual=0;
		gFlag.Ring=false;
	}
	else if(Signal==0x55)//����״̬
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
//����״̬״̬,����0x145(325) ����
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
//�źŻ����ּ�ָʾ״̬0x147(327)
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
//�źŻ�������ָʾ״̬0x148(328)
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
//�źŻ�ȫ��ָʾ״̬0x149(329)
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

//MAU�����źŻ����ص�����0x15(21)
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
//MAU�ֿ�/�Զ��л�����0x16(22)
void MMAS(void)
{
	if (!SIGNAL2)
	{
		TxBuff[0] = 0xAA;	//�ֿع���ʹ��
		LEDStatus.Manual = 1;
//		gFlag.Ring=true;
	}
	else
	{
		TxBuff[0] = 0x55;//�Զ�����ʹ��
		LEDStatus.Manual = 0;
//		gFlag.Ring=false;
	}
	
	MAUFunFlag=22;
	Send_CAN_DataFrame_Single(TxBuff,1);
}

//MAU������������0x19(25)
void MKYS(uint8_t Key,uint8_t KeyState)
{
//	Key = Read_KeyValue();
	switch (Key)
	{
	case KEY_YF:case KEY_YFFN:
		TxBuff[0] = 0x40;//����
		break;
	case KEY_AR:case KEY_ARFN:
		TxBuff[0] = 0x80;//ȫ��
		break;
	case KEY_STEP:case KEY_STEPFN:
		TxBuff[0] = 0x00;//����
		break;
	case KEY_NUM1:
		TxBuff[0] = 0x01;//����1
		break;
	case KEY_NUM2:
		TxBuff[0] = 0x02;//����2
		break;
	case KEY_NUM3:
		TxBuff[0] = 0x03;//����3
		break;
	case KEY_NUM4:
		TxBuff[0] = 0x04;//����4
		break;
	case KEY_NUM5:
		TxBuff[0] = 0x05;//����5
		break;
	case KEY_NUM6:
		TxBuff[0] = 0x06;//����6
		break;
	case KEY_NUM7:
		TxBuff[0] = 0x07;//����7
		break;
	case KEY_NUM8:
		TxBuff[0] = 0x08;//����8
		break;
	case KEY_NUM9:
		TxBuff[0] = 0x09;//����9
		break;
	case KEY_NUM10:
		TxBuff[0] = 0x0a;//����10
		break;
	case KEY_NUM11:
		TxBuff[0] = 0x0b;//����11
		break;
	case KEY_NUM12:
		TxBuff[0] = 0x0c;//����12
		break;
	case KEY_NUM13:
		TxBuff[0] = 0x0d;//����13
		break;
	case KEY_NUM14:
		TxBuff[0] = 0x0e;//����14
		break;
	case KEY_NUM15:
		TxBuff[0] = 0x0f;//����15
		break;
	case KEY_NUM16:
		TxBuff[0] = 0x10;//����16
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
//MAU��Ӳ���汾��Ϣ0x1a(26)
void MAVS(void)
{
	TxBuff[0] = (HardVersions & 0xff);	//ӡ�ư�汾
	TxBuff[1] = (SoftVersions & 0xff);	//�̼��汾
	MAUFunFlag=26;

	Send_CAN_DataFrame_Single(TxBuff,2);
}
/********************************************************************
             CAN�����������������
********************************************************************/

//����CAN����֡��������ʶ��
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
		PASC();//�������뼰ʹ������,����0x0B(11)

		break;
	case 321:
		WARC();//�������״̬,����0x141(321)

		break;
	case 322:
		FATC();//���ع���״̬,����0x142(322)

		break;
	case 323:
		LMSC();//���ͨ���������״̬,����0x143(323)

		break;
	case 324:
		CSOC();//����Դ״̬,����0x144(324)

		break;
	case 325:
		FLSC();//����״̬,����0x145(325)����

		break;

	case 327:
		KNMC();//���ּ�ָʾ״̬,����0x147(327)

		break;
	case 328:
		KSPC();//�ֶ�������״̬,����0x148(328)

		break;
	case 329:
		KARC();//ȫ��״̬״̬,����0x149(329)

		break;

//�������Ӳ��Ա�������

	default:
		memset(Receive_Buf,0x00,10);
		break;
	}
}
//CAN�������ݴ���
void Can_Receive_CMD(void)
{
	Receive_CAN_DataFrame(&RxMessage,RxMessage.DLC);//����CAN����
}
////////////////////////////////////////////////LED���ܺ���///////////////////////////////////////////////////////////

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
//״ָ̬ʾ�ƿ���
void LEDALL(uint8_t Mark)
{
	//ָʾ��ȫ����
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
	else  //ָʾ��ȫ���ر�
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
//��������������
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
void RUNLED_FLASH(void)   //sysflag;ϵͳ״̬LED��˸
{
	if(TimeCnt_LedRun>=5)
	{
		TimeCnt_LedRun=0;
		LEDStatus.Run=!LEDStatus.Run;
	}
	
	
}
//ˢ��LED
void LED_REFRESH(void)
{
	LEDStatusFresh();
	StatusLedFresh();
}
///////////////////////////////////////////////////////////////////���������ع��ܺ���//////////////////////////////////////////////////////////////////////////////////////
//�����¼�����
void Key_Event(uint8_t keyNum,KeyStateType KeyState)//keyNum:�������      KeyState��KEY_RELEASE���ͷŰ���  KEY_PRESS�����°���
{
	KeyHandle.KeyVal=keyNum;
	KeyHandle.KeyState=KeyState;
}
//��ȡ����IO����
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
//���ÿ������
void KEY_Check(uint8_t keyNum)
{
	static uint8_t keyNum_Press=31;
	uint8_t CurrentKeyVal=1;
	if( (keyNum_Press!=31) && (keyNum_Press!=keyNum) )return;//����а���û���ͷţ��ȴ������ͷź��ټ����������
	CurrentKeyVal=KeyValGet(keyNum);
	if(CurrentKeyVal==0)//��⵽��������
	{
		if(key_IsPress[keyNum]==false)
		{
			if(key_filter[keyNum]>=2)//20ms����ȥ��
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

//��ѯ���а���
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
//��ⵥ������
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
//��ѯ���п���
void SWITCH_Scan(void)
{
	uint8_t i;
	for(i=0;i<2;i++)
	{
		SWITCH_Check(i);
	}
}
//����
void KeyPassword_Unlock(void)
{
	static uint8_t PasswordBit=0;
	uint8_t keyBitVal;
	uint8_t passwordVal=0x1f;//password;
	if(gFlag.PasswordChange)
	{
		PasswordBit=0;
	}
	if( (KeyHandle.KeyVal>=0) && (KeyHandle.KeyVal<=10) )//��⵽����
	{
		if(KeyHandle.KeyState==KEY_PRESS)//��������
		{
			gFlag.LedLockBlink=true;
			PasswordBit++;
			keyBitVal=KeyHandle.KeyVal;
			
			Led_Status[keyBitVal] = !Led_Status[keyBitVal];
		}
		else if(KeyHandle.KeyState==KEY_RELEASE)//�ͷŰ���
		{
			keyBitVal=KeyHandle.KeyVal;
			Led_Status[keyBitVal] = !Led_Status[keyBitVal];
			keyBitVal-=3;
			if (((  passwordVal>>(4-PasswordBit) ) & 0x01) != keyBitVal )
			{//�жϼ��������Ƿ���洢������ͬ
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

//��������
void KeyCMD(void)
{
	static uint8_t KeyFN_PressCnt=0;
	uint8_t KeyValCmd=31;
	if(KeyHandle.KeyVal!=31)
	{
		TimeCnt_KeyNone=0;//�а��������������ް���������ʱ
		Led_Status[KeyHandle.KeyVal] = !Led_Status[KeyHandle.KeyVal];
		
		if(KeyHandle.KeyVal==0 && KeyHandle.KeyState==KEY_RELEASE && KeyFN_PressCnt==0)//��⵽FN����
		{
			KeyFN_PressCnt++;
			gFlag.KeyFNFun=true;
			return;
		}
		//�����ֵ
		if(gFlag.KeyFNFun)
		{
			KeyValCmd=KeyHandle.KeyVal+12;
			if(KeyHandle.KeyState==KEY_RELEASE)//�ͷŰ���
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
		//��������
		if(KeyValCmd==KEY_TEST)
		{
			if(KeyHandle.KeyState==KEY_RELEASE)//�ͷŰ���
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
				MLMS();//��ƿ��ص�����
			}
			else if(i==1)
			{
				MMAS();//MAU�ֿ�/�Զ��л�����
			}
		}
	}
}

//��ֵ����
void KEY_Process(void)
{
	SwitchCMD();
	if(gFlag.KeyNone)
	{
		gFlag.KeyNone=false;
		gFlag.PasswordLock=true;
		LEDStatus.Lock=1;
	}
	if(gFlag.PasswordLock)//����
	{
		KeyPassword_Unlock();
		LedBLink_KeyLock();
	}
	else//δ����
	{
		if(gFlag.Wait!=true)KeyCMD();
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//�豸���ߡ����߹��ܺ���
void Device_StateChange(void)
{
	if(gFlag.ActionOnOrOffLine==1)//�豸����
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
	else if(gFlag.ActionOnOrOffLine==2)//�豸����
	{
		gFlag.ActionOnOrOffLine=0;
		
		//����״̬��Ϣ
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

	//��ʼ������
	RxMessage.StdId = 0x00;  //��׼��ʶ����ȡֵ 0~0x7FF��
	RxMessage.ExtId = 0x00;  //��չ��ʶ����ȡֵ 0~0x3FFFF��
	RxMessage.IDE = 0x00;    //��ʶ��������
	RxMessage.DLC = 0x00;    //��Ϣ��֡���ȣ�ȡֵ 0~0x8��
	RxMessage.FMI = 0x00;    //Ҫͨ���Ĺ�������������Щ��Ϣ�洢�������У�ȡֵΪ0~0xFF��
	for (i=0; i<8; i++)
	{
		RxMessage.Data[i] = 0x00;//���������ݣ�ȡֵΪ0~0xFF��
	}
	//���������ϵ������ͷ�FIFO
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//����
	//������յ�����
	if (RxMessage.IDE==CAN_ID_STD)
	{
		if (RxMessage.DLC>0)
		{
			gFlag.CanDataRx=true;
//			Receive_CAN_DataFrame(&RxMessage,RxMessage.DLC);//����CAN����
		}
		if (RxMessage.DLC==0)
		{
			
			if(gFlag.CPUOffLine)
			{
				gFlag.ActionOnOrOffLine=2;//�豸���߱�־
			}
			gFlag.CPUOffLine = false;//�豸����״̬��0
//			RUNledstatus = !RUNledstatus;
			TimeCnt_GetCPUHeart = 0;//����������������ʱ��0
		}

	}
}



//------------------------Timer2 IRQ----------------100ms��ʱ---------------------------------
void TIM2_IRQHandler(void)
{
	static uint8_t TimeCnt_Heartsend=0;
	static uint8_t TimeCnt_Beep=0;
	
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//TIM2 interrupt has occured
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);//Clear interrupt pending bit
		
		/*******************************************************************************
				                          ����״̬�ж�
		********************************************************************************/
		if (TimeCnt_Heartsend<200)
		{//����֡ѭ������200������
			TimeCnt_Heartsend++;
			if (TimeCnt_Heartsend == 10)
			{//�ȴ�10�η���1�Σ�������1000ms
				TimeCnt_Heartsend=0;  //���ʹ�������
				gFlag.SendHeartbeat = true;//����������־��1
			}
			
		}
		else TimeCnt_Heartsend=0;
		
      	/*******************************************************************************
				                          ����״̬�ж�
		********************************************************************************/
		TimeCnt_GetCPUHeart++;
		if(TimeCnt_GetCPUHeart==12)//����1.2ms
		{
			if(gFlag.CPUOffLine==false)
			{
				gFlag.ActionOnOrOffLine=1;
				gFlag.CPUOffLine=true;
			}
			
		}
		
		/*******************************************************************************
				                         FN��ϼ�����״̬
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
				                        �����������ʱ��15min���ް�������������
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
				                       ����ָʾ����˸ʱ�����
		********************************************************************************/
		TimeCnt_LedRun++;
		
		/*******************************************************************************
				                       ����
		********************************************************************************/
		if(gFlag.Ring)
		{
			TimeCnt_Beep++;
			if (TimeCnt_Beep==1)
				TIM_Cmd(TIM3, ENABLE);											/* �������� */
			else if (TimeCnt_Beep==3) //time=ring_time*100ms(3s��һ��)
			{
				TIM_Cmd(TIM3, DISABLE);
				BEEPOFF;
			}
			else if(TimeCnt_Beep>=30)TimeCnt_Beep=0;
		}
		else 
		{
			TIM_Cmd(TIM3, DISABLE);												/* �ر�����*/
			BEEPOFF;
			TimeCnt_Beep=0;
		}

	}
}
//------------------------Timer3 IRQ-------100us��ʱ��-MAU������ʹ�ܶ�ʱ��-----------------------------------------
extern uint32_t time_count_test;
void TIM3_IRQHandler(void)
{
	static uint8_t TimeCnt_Ring=0;
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)//TIM2 interrupt has occured
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);//Clear interrupt pending bit
			
		//�������򿪣�Ƶ��2kHz����T=100us*5=500us
		//�������򿪣�Ƶ��3.3kHz����T=100us*3=300us
		//�������򿪣�Ƶ��4kHz����T=50us*5=250us
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
//------------------------Timer4 IRQ-----10ms��ʱ��--����&��������״̬�ɼ�ר��------------------------------------------
void TIM4_IRQHandler(void)
{
	static uint16_t TimeCnt_LedAllOn=0;
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)//TIM2 interrupt has occured
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);//Clear interrupt pending bit
		/*******************************************************************************
				                        ������ѯ��־λ��1
		********************************************************************************/
		gFlag.KeyScan=true;
		
		/*******************************************************************************
				                        ˢ��LED
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

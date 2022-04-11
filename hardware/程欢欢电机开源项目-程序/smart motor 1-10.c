/*
1��10�������33.1776Mhz

*/
#include "STC15W.h"
//���ڸ�������ת
#define motor_CW motor_l=1;motor_r=0		//���Ƶ��������ת Ĭ��1��0
#define motor_CCW motor_l=0;motor_r=1  	//Ĭ��0��1
#define motor_CW_coder angle--					//��¼�Ƕȵ����� Ĭ��--
#define motor_CCW_coder angle++					//Ĭ��++
#define motor_CWCCW_flag 1 							//���õ�������תʱ��������Ĭ��1 �ɸ�Ϊ-1
//���ڸ���ת�ٱ�
#define angleTOreal 9/8 //���㣬��360/(���ٱ�*������һȦ�ż�*2)
#define realTOangle 8/9 //����һ���෴
#define angleSpeed 15/4 //�ٶ� (60*1000/50) / (���ٱ�*������һȦ�ż�*2)
//�¶ȱ���ֵ
#define temp_safety 814 //��Ӧ65��
#define temp_safety_temp 65

#define motor_brake motor_l=1;motor_r=1
#define motor_software_stop motor_l=0;motor_r=0;CCAP0H=0  //�����ɲ��
#define motor_nopower motor_l=0;motor_r=0;CCAP0H=0xff
#define led_g CCAP1H
#define led_y CCAP2H

unsigned char code IDcode_h=0;//�����Ϣ��
unsigned char code IDcode_l=1;//�����Ϣ������ʶ����ʲôӲ����1Ϊ���ٵ��
unsigned char code version_h=1;
unsigned char code version_l=0;//�̼��汾������λ���



unsigned int code temp_data[91]=
{9183,8721,8285,7873,7485,7118,6771,6443,6133,5840 //-25
,5562,5300,5051,4816,4593,4381,4181,3991,3811,3640 //-15
,3477,3323,3177,3038,2905,2780,2660,2546,2438,2335 //-5
,2237,2144,2055,1970,1890,1813,1739,1669,1602,1539 //5
,1478,1420,1364,1311,1261,1212,1166,1122,1079,1039 //15
,1000,963,927,893,861,830,800,771,743,717          //25
 ,692,667,644,622,600,580,560,541,523,505          //35
 ,488,472,457,442,427,413,400,387,375,363          //45
 ,351,340,330,319,309,300,291,282,273,265,257};    //55
                                                   //65
//������Ϣ
sbit motor_i = P1^2; //����������� AD2
sbit tem = P1^3; //�������� AD3
sbit power_v = P1^4; //�ܵ�Դ��ѹ AD4
//ָʾ��
sbit led_gg = P1^0; //�Ƶ� PWM1
sbit led_yy = P3^7; //�̵� PWM2
//���
sbit motor_l = P5^4; //�����ת ����Ч
sbit motor_r = P5^5; //�����ת ����Ч
sbit motor_pwm = P1^1; //����ٶ� PWM0
sbit motor_a = P3^6; //���������A
sbit motor_b = P3^2; //���������B
//ͨѶ
sbit io_t = P1^5; //���ͱ��λ
sbit io_r = P3^3; //���ձ��λ INT1
//�ٶ�
int xdata speed;//�ٶȣ���λ Ȧ/����
unsigned long xdata angle_record[11];  //ÿ5ms��¼һ���ٶȣ�ÿ50ms����������������ٶ�
unsigned char angle_record_n;//��¼��һ����������
//����Ƕ�
long angle=0;//���ʵʱ�Ƕ� 
long angle_real;//��¼���ǽǶȣ���λ�ȣ�����Ҫ���������Ƕ���ͣģʽ�¼������
long angle_stop=0;//���ֹͣ(����)�Ƕȣ�Ҳ��Ŀ����
//������ܱ���������ִ��ʱ�Ĳ���
unsigned char function=15;//����ִ�еĹ�����Ĭ��15Ϊ��ͣ
char motor_cw_ccw=1;//����ת��¼ѡ��(��¼Ŀ��ת�����򣬲�Ӱ������㷨)����ת1����ת-1���н��ճ�����λ
long angle_target;//����Ƕ�Ŀ�����������м�ֵ
long timer;//��ʱ
int power;//������ʰٷֱ�-150~150,����-50~50
bit motor_stop_record;//��ʱ��ģʽ�£���¼�״���ɵ�ǰ��ת��=1Ϊ���룬�������,�Զ����ã������Ԥ
bit motor_sangle_record=1;//ע�⣡�Ƕ�ģʽת��ǰ����1.�ڽǶ�ģʽ�£���¼���һ�ζ���(������ɺ�ִ�нǶȳ���)��=0Ϊ��� =1Ϊ������
//�����������
unsigned char content_function;//���յĹ������
unsigned char content[8];
unsigned char content_num;//����ָ��λ���
unsigned long content_target;//���յ������ݣ���ʱ�洢
//�����������
long send_content_long;//��������(��)
int send_content_int;//��������(��)
char send_xor[4];//�����������У��λ
//����״̬��¼����
int xdata voltage;//��ѹ����λ
int xdata electricity;//��������λ������
int xdata temp=0;//�¶ȣ���λ��
unsigned char main_ad_n=2;//ad��ȡ��λ����
unsigned char send_num;//��Ҫ���͵��������
//�����ܱ���
bit motor_program_run;//�ɶ�ʱ��֪ͨ�������еĵ���������С���=1�����У�������㡣
bit serial_t_busy;//���ڷ���æ��1Ϊæ��0Ϊ��
bit send_allow;//���ͱ�־λ��1Ϊ��Ҫ���ͣ�0Ϊ������
//ֹͣ����PID����
char stop_kp=8; //6,10
char stop_ki=20;//3,20ע���˴�ʹ�ó���
char stop_kd=40; //30,40
int stop_i;
int stop_d;
int stop_lasterr;
int stop_err;
//���ٳ���PID����
char code speed_kp=1;//5
char code speed_ki=2;//3
char code speed_kd=1;//5 ����
int speed_i;
int speed_d;
long speed_err;
long speed_lasterr;
long speed_lastangle;//��һ�νǶȣ����ڼ����ٶ�


//�Ӻ���Ŀ¼
void delay(unsigned int delay_i);//�ӳ�1����
//void SendString(char *SendString_n);//���ڷ��ͳ���
void send_long(long send_content_long);
void send_int(send_content_int);
void receive_angle_increment();//���ճ����еĽǶ�������¼����
void overflow();//�жϼ���ַ��������������ֹͣ����
void receive_angle_absolute();//���ճ����еĽǶȾ���ֵ��¼����
int temp_real(int temp_real_n);//�¶�ת�����򣬷����¶�
void speed_get();//�ٶȻ�ȡ����Ӧ������������
void motor_stop(angle_stop);//���ֹͣ��������
void motor_speed(speed_target);//���ٳ���
void Init();//��ʼ����
void UartInit();//��������
void PWMInit();//PWM����
void Timer0Init();//T0����
void ADInit();//AD����
void motor_move(int motor_move_n);//���Ƶ���ˣ���Χ-150 150��	

void main()
{ unsigned int main_n;
	unsigned char main_led;//��ɫLED��������˸���ñ���
	bit main_led_n;//ͬ��
	Init();//ȫ�����ó�ʼ��
	led_y=0xff;//�رջ�ɫLED	
	io_t=1;//æλ����
	while(1)
	{	
		if(send_allow)//��������
		{
			send_allow=0;
			switch(send_num)
			{
				case 1://�Ƕ�
					{	send_long((long)angle*angleTOreal);	}break;
				case 2://�ٶ�
					{	send_int(speed); }break;	
				case 3://��ѹ
					{	send_int(voltage); }break;	
				case 4://����
					{	send_int(electricity);}break;	
				case 5://�¶�
					{	send_int(temp_real(temp)); }break;	
				case 6://��λ
					{	IAP_CONTR=0xe0; }break;	
				case 7://ʵʱ���ʰٷֱȣ�/255��
				{
					if(CCAP0H==0)
					send_int(0);
					else
					{
						if(motor_l==1 && motor_r==0)
							send_int(255-CCAP0H);
						else if(motor_l==0 && motor_r==1)
							send_int((255-CCAP0H)*-1);
						else 
							send_int(0);
					}
				}break;
				case 255:
					{	send_long((unsigned long)IDcode_h<<24|(unsigned long)IDcode_l<<16|(unsigned long)version_h<<8|version_l);}break;
			}
		}
		if(motor_program_run)
		{ 
/*����ִ��ʱ��������*/
			main_n++;
			if(main_n==0xffff)main_n=0;
			motor_program_run=0;
/*led�����Ƴ���*/
			if(main_led_n)main_led--;
			else main_led++;
			led_g=main_led;
			if(main_led==255)main_led_n=1;
			else if(main_led==0)main_led_n=0;
	
			switch(function)
			{ 
				case 0://����0�����ʰٷֱȡ�������
				{ 
					motor_move(power);
					io_t=0;//æ�ź�λ
				}break;
				case 1://����1�����ʰٷֱȡ�ʱ�䡢��ͣ
				{	
					if(timer>5)
					{ 
						timer-=5;
						motor_move(power);
						motor_stop_record=1;
					}
					else
					{ 
						if(motor_stop_record)
						{
							motor_stop_record=0;
							angle_stop=angle;
						}
						motor_stop(angle_stop);
						io_t=1;//æλ����
					}		
				}break;
				case 2://����2�����ʰٷֱȡ�ʱ�䡢��ͣ
				{	
					if(timer>5)
					{
						timer-=5;
						motor_move(power);
					}
					else
					{ 
						motor_software_stop;//��ͣ
						io_t=1;//æλ����
					}		
				}break;		
				case 3://����3�����ʰٷֱȡ�ʱ�䡢����
				{	
					if(timer>5)
					{ 
						timer-=5;
						motor_move(power);
					}
					else
					{ 
						motor_nopower;
						io_t=1;//æλ����
					}		
				}break;			
				case 4://����4�����ʰٷֱȡ��Ƕȡ���ͣ
				{	
					if((angle_stop-angle)*motor_cw_ccw > (power/3)*motor_cw_ccw && motor_sangle_record)
					{ 
						motor_move(power);
					}
					else
					{ 
						motor_sangle_record=0;
						motor_stop(angle_stop);
						io_t=1;//æλ����
					}		
				}break;	
				case 5://����5�����ʰٷֱȡ��Ƕȡ���ͣ
				{	
					if(angle_stop*motor_cw_ccw > angle*motor_cw_ccw && motor_sangle_record)
					{
						motor_move(power);
					}
					else
					{
						motor_sangle_record=0;
						motor_software_stop;//��ͣ
						io_t=1;//æλ����
					}		
				}break;				
				case 6://����6�����ʰٷֱȡ��Ƕȡ�����
				{	
					if(angle_stop*motor_cw_ccw > angle*motor_cw_ccw && motor_sangle_record)
					{
						motor_move(power);
					}
					else
					{
						motor_sangle_record=0;
						motor_nopower;
						io_t=1;//æλ����
					}		
				}break;			
				case 10://����10�����١�������
				{	
					if(power==0 && motor_sangle_record)
					{
						motor_sangle_record=0;//��С�����
						angle_stop=angle;
						speed_i=0;speed_lasterr=0;speed_lastangle=angle;//����PID�м�¼�ϴνǶ�
						motor_stop(angle_stop);
						
					}
					else if(power==0)
					{
						motor_stop(angle_stop);
					}
					else if(main_n%5==0)
					{
						motor_speed(power);motor_sangle_record=1;
					}
						io_t=1;//æλ����
				}break;		
				case 11://����11�����١�ʱ�䡢��ͣ
				{	
					if(timer>5)
					{
						timer-=5;
						if(main_n%5==0)
							motor_speed(power);
						motor_stop_record=1;
					}
					else
					{
						if(motor_stop_record)
						{
							motor_stop_record=0;
							angle_stop=angle;
						}						
						motor_stop(angle_stop);
						io_t=1;//æλ����
					}	
				}break;		
				case 12://����12�����١�ʱ�䡢��ͣ
				{	
					if(timer>5)
					{
						timer-=5;
						if(main_n%5==0)
							motor_speed(power);
					}
					else
					{
						motor_software_stop;//��ͣ	
						io_t=1;//æλ����
					}
				}break;	
				case 13://����13�����١�ʱ�䡢����
				{	
					if(timer>5)
					{
						timer-=5;
						if(main_n%5==0)
							motor_speed(power);
					}
					else
					{
						motor_nopower;
						io_t=1;//æλ����
					}
				}break;	
				case 14://����14�����١��Ƕȡ���ͣ
				{	
					if((angle_stop-angle)*motor_cw_ccw > (power/2)*motor_cw_ccw && motor_sangle_record)
					{
						if(main_n%5==0)
							motor_speed(power);
					}
					else
					{ motor_sangle_record=0;
						motor_stop(angle_stop);
						io_t=1;//æλ����
					}		
				}break;	
				case 15://����15�����١��Ƕȡ���ͣ
				{	
					if(angle_stop*motor_cw_ccw > angle*motor_cw_ccw && motor_sangle_record)
					{
						if(main_n%5==0)
							motor_speed(power);
					}
					else
					{ motor_sangle_record=0;
						motor_software_stop;//��ͣ	
						io_t=1;//æλ����
					}		
				}break;					
				case 16://����16�����١��Ƕȡ�����
				{	
						if(angle_stop*motor_cw_ccw > angle*motor_cw_ccw && motor_sangle_record)
						{
							if(main_n%5==0)
								motor_speed(power);
						}
						else
						{ motor_sangle_record=0;
							motor_nopower;
							io_t=1;//æλ����
						}		
				}break;					
				case 20://����14����ͣ
				{
					if(motor_sangle_record)
						{angle_stop=angle;motor_sangle_record=0;}
					else
						{motor_stop(angle_stop);io_t=1;}//æλ����
				}break;					
				case 21://����15����ͣ
				{	
					motor_software_stop;io_t=1;//æλ����
				}break;					
				case 22://����16������
				{	
					motor_nopower;io_t=1;//æλ����
				}break;						
			}
//��ȡ�ٶȳ���			
			speed_get();
	    if(speed<0)
				speed_get();
/*ADת������*/
			ADC_CONTR=0x88+main_ad_n;
			main_ad_n++;
			if(main_ad_n==5)main_ad_n=2;
/*AD�¶ȱ�������*/
			while(temp>temp_safety)//����65��
			{ 
				while(1)
				{
					led_g=255;motor_nopower;
					led_y=0;delay(200);led_y=255;delay(200);
					/*ADת������*/
					ADC_CONTR=0x88+main_ad_n;
					main_ad_n++;
					if(main_ad_n==5)main_ad_n=2;
				}
			}
		}
	}
}
/*************************/
/*�¶�ת������*/
/*************************/
int temp_real(int temp_real_n)
{ int temp_real_nn=0;//���,Ҳ���¶�
	temp_real_n=1024000/temp_real_n-1000;
	while(1)
	{
		if(temp_data[temp_real_nn]<temp_real_n)
		{
			if(temp_real_nn-26>temp_safety_temp)
				return temp_safety_temp;
			else
				return temp_real_nn-26;
		}
		else
			temp_real_nn++;
	}
}

/*************************/
/*�ٶȻ����Ӧ������������*/
/*************************/
void speed_get()
{
			angle_record[angle_record_n]=angle;
			if(angle_record_n==10)
			{
				 if(angle_record[10]>=angle_record[0])
				 {speed=(angle_record[10]-angle_record[0])*angleSpeed;}
				 else
				 {speed=(angle_record[0]-angle_record[10])*angleSpeed;}
			}
			else
			{
				if(angle_record[angle_record_n]>=angle_record[angle_record_n+1])
				{speed=(angle_record[angle_record_n]-angle_record[angle_record_n+1])*angleSpeed;}
				else
				{speed=(angle_record[angle_record_n+1]-angle_record[angle_record_n])*angleSpeed;}
			}
			angle_record_n++;
			if(angle_record_n==11)angle_record_n=0;
}
/*************************/
/*�жϼ����Ƿ��������������ֹͣ���򣬻ƵƳ���*/
/*************************/
void overflow()
{
	if(OV==1)//������
	{EA=0;led_g=255;led_y=0;motor_software_stop;while(1);}			
}
/*************************/
/*���ճ����У��ǶȾ���ֵ��¼*/
/*************************/
void receive_angle_absolute()
{
	angle_real=angle_target;
	angle_stop=angle_target*realTOangle;
	if(angle_target>angle)
	{ 				
		motor_cw_ccw=1;//�����ת
		if(power<0)power*=-1;//��Ҫ��ת
	}
	else
	{				
		motor_cw_ccw=-1;//��Ƿ�ת
		if(power>0)power*=-1;//��Ҫ��ת
	}
}
/*************************/
/*���ճ����У��Ƕ�������¼*/
/*************************/
void receive_angle_increment()
{
	if(function==4 || function==14)//����ϴ�Ҳ�ǽǶ���ͣģʽ�����ϴνǶ�Ŀ����Ϊ�������ۼ�
	{
		if(power>0)//�������Ϊ��ת
		{
			if(angle_target>0)//����Ƕ�Ϊ��
			{
				motor_cw_ccw=1*motor_CWCCW_flag;//�����ת
				if(content[1]==4 || content[1]==14)//�����ǰ����Ϊ�Ƕ�������ͣ
				{ 
					angle_real+=angle_target;
					overflow();
					angle_stop=angle_real*realTOangle; 
				}
			}	
			else//����Ƕ�Ϊ��
			{ 
				motor_cw_ccw=-1*motor_CWCCW_flag;//��Ƿ�ת
				if(content[1]==4 || content[1]==14)//�����ǰ����Ϊ�Ƕ�������ͣ
				{ 
					angle_real+=angle_target;
					overflow();
					angle_stop=angle_real*realTOangle; 
				}
				power=power*-1;//���ʱ�Ϊ����
			}
		}		
		else//�������Ϊ��ת
		{
			if(angle_target>0)//����Ƕ�Ϊ��
			{
				motor_cw_ccw=-1*motor_CWCCW_flag;//��Ƿ�ת
				if(content[1]==4 || content[1]==14)//�����ǰ����Ϊ�Ƕ�������ͣ
				{ 
					angle_real-=angle_target;
					overflow();
					angle_stop=angle_real*realTOangle; 
				}
			}	
			else//����Ƕ�Ϊ��
			{
				motor_cw_ccw=1*motor_CWCCW_flag;//�����ת
				if(content[1]==4 || content[1]==14)//�����ǰ����Ϊ�Ƕ�������ͣ
				{ 
					angle_real-=angle_target;
					overflow();
					angle_stop=angle_real*realTOangle; 
				}	
				power=power*-1;//���ʱ�Ϊ��
			}	
		}
	}
	else//�ϴβ��ǽǶ�����ģʽ
	{
		if(power>0)//�������Ϊ��ת
		{
			if(angle_target>0)//����Ƕ�Ϊ��
			{
				motor_cw_ccw=1*motor_CWCCW_flag;//���Ϊ��ת
				angle_stop=angle+(angle_target*realTOangle);//�ۼӽǶ� angle_real
				overflow();
				if(content[1]==4 || content[1]==14)//�����ǰ����Ϊ�Ƕ�������ͣ
					{ angle_real=angle_stop*angleTOreal; }//���ǶȻ��㲢��¼
			}	
			else//����Ƕ�Ϊ��
			{ 
				motor_cw_ccw=-1*motor_CWCCW_flag;//��Ƿ�ת
				angle_stop=angle+(angle_target*realTOangle);//�ۼӸ��Ƕ�
				overflow();
				if(content[1]==4 || content[1]==14)//�����ǰ����Ϊ�Ƕ�������ͣ
					{ angle_real=angle_stop*angleTOreal; }//���ǶȻ��㲢��¼
				power*=-1;//���ʱ�Ϊ����
			}
		}		
		else//�������Ϊ��ת
		{
			if(angle_target>0)//����Ƕ�Ϊ��
			{
				motor_cw_ccw=-1*motor_CWCCW_flag;//��Ƿ�ת
				angle_stop=angle+(angle_target*-realTOangle);//���Ƕ�
				overflow();
				if(content[1]==4 || content[1]==14)//�����ǰ����Ϊ�Ƕ�������ͣ
					{ angle_real=angle_stop*angleTOreal; }//���ǶȻ��㲢��¼
			}	
			else//����Ƕ�Ϊ��
			{ 
				motor_cw_ccw=1*motor_CWCCW_flag;//�����ת
				angle_stop=angle+(angle_target*-realTOangle);//�����Ƕ�	
				overflow();
				if(content[1]==4 || content[1]==14)//�����ǰ����Ϊ�Ƕ�������ͣ
					{ angle_real=angle_stop*angleTOreal; }//���ǶȻ��㲢��¼				
				power*=-1;//���ʱ�Ϊ��
			}	
		}
	}
}

/*************************/
/*������ٳ���*/
/*************************/
void motor_speed(speed_target)
{	
	speed_err=speed_lastangle-angle+speed_target;
	speed_i+=speed_err;
	if(speed_i<-150)speed_i=-150;//���Ʋ�����
	else if(speed_i>150)speed_i=150;//���Ʋ�����
	speed_d=speed_err-speed_lasterr;
	motor_move(speed_err*speed_kp+speed_i*speed_ki+speed_d/speed_kd);
	speed_lasterr=speed_err;
	speed_lastangle=angle;
}
/*************************/
/*���ֹͣ����*/
/*************************/
void motor_stop(angle_stop)
{
	stop_err=angle_stop-angle;
	stop_i+=stop_err*0.8;
	stop_d=stop_err-stop_lasterr;
	motor_move(stop_err*stop_kp+stop_i/stop_ki+stop_kd*stop_d);
	stop_lasterr=stop_err;
}
/*************************/
/*33.1776Mhz �ӳ�1ms�ı���*/
/*************************/
void delay(unsigned int delay_i)
{
	unsigned int delay_j;
  while(delay_i--)
		for(delay_j=0;delay_j<860;delay_j++);
}
/*************************/
/*AD�жϣ���ȡ��ѹ���������¶�*/
/*************************/
void adc_isr() interrupt 5 using 1
{ 
	ADC_CONTR&=0x07;//AD��Դ�ͱ�־λ����
	switch(ADC_CONTR&0x07)
	{
		case 2://�������
		{ 
			electricity=ADC_RES*4+ADC_RESL;
		}break;
		case 3://��������
		{
			temp=ADC_RES*4+ADC_RESL-30;
		}break;
		case 4://��Դ��ѹ
		{
			voltage=ADC_RES*4+ADC_RESL;
		}break;
	}
}
/*************************/
/*���ж�0����ȡ���������*/
/*************************/
void motor_angle_int0() interrupt 0
{ 
	if(motor_a == motor_b)
		motor_CW_coder;
	else
		motor_CCW_coder;
} 
/**********************************/
/*���ж�1:������֪ͨ����ɽ���ͨѶ*/
/**********************************/
void motor_angle_int1() interrupt 2
{ 
	content_num=0;
} 
/*************************/
/*�����ж�*/
/*************************/
void uart() interrupt 4 using 1
{ 	
	if(RI)
	{
		RI=0;
	  if(!io_r && content_num<8)//����ѡλ����
		{ 
			content[content_num]=SBUF;
			content_num++;
			if(content[0]!=0xa5)//���û�յ���ʼλ��ֹͣ��������
				content_num=20;
		}
		if(content_num==8)//������
		{
			if(content[7]==0xa5 && content[6]==0xa5 && content[5]==0xa5 && content[4]==0xa5 && content[3]==0xa5 && content[2]==0xa5 && content[1]==0xa5)//ȫ��0xa5�Ļ����ش������Ϣ
			{
				send_allow=1;//����ش�
				send_num=255;//�ش��������
			}
			else if(content[7]==content[6]^content[5]^content[4]^content[3]^content[2]^content[1])//У��
			{ 
				content_target=0;//�洢����λ
				content_target|=content[3];
				content_target=content_target<<8|content[4];
				content_target=content_target<<8|content[5];
				content_target=content_target<<8|content[6];	
				
/*0~3*/	if(content[1]<4)//����0~3�����ʰٷֱȡ�ʱ���������
				{
					function=content[1];//����λ
					power=(char)content[2];//���ʰٷֱ�
					power=power*1.5;
					if(function==0)
						{timer=0;io_t=1;}//æλ����
					else if(timer>5)
						{timer=content_target;io_t=1;}//æλ����
					stop_i=0;stop_lasterr=0;//ֹͣ����PID��������
					if(function==0)//�����ǰ����Ϊ10���������ʱ��ȡ��æλ
						{timer=0;io_t=1;}
					else
						{timer=content_target;io_t=0;}
					motor_stop_record=1;//�л�Ϊδ���ת��	
				}
/*4~6*/	else if(content[1]<7)//����4~6�����ʰٷֱȡ��Ƕ�����
				{ 
					power=(char)content[2];//���ʰٷֱ�
					power=power*1.5;
				  angle_target=(long)content_target;//Ŀ��Ƕȣ���ʱ�洢��
					receive_angle_increment();//�����Ƕȼ�¼����	 
					function=content[1];//�ù���λ
					stop_i=0;stop_lasterr=0;//ֹͣ����PID��������	
					motor_sangle_record=1;//�л�Ϊδ���ת��	
					io_t=0;//æλ:æ
				}		
/*7~9*/	else if(content[1]<10)//����7~9�����ʰٷֱȡ��ǶȾ���ֵ
				{
					power=(char)content[2];//���ʰٷֱ�
					power*=1.5;
				  angle_target=(long)content_target;//Ŀ��Ƕȣ���ʱ�洢��
					receive_angle_absolute();//Ŀ��������ֵ��¼
					function=content[1]-3;//�ù���λ
					stop_i=0;stop_lasterr=0;//ֹͣ����PID��������	
					motor_sangle_record=1;//�л�Ϊδ���ת��	
					io_t=0;//æλ:æ
				}
/*10~13*/else if(content[1]<14)//����0~3�����١�ʱ���������
				{
					power=(char)content[2];//�ٶ�
					power=power*8/10;
					if(content[1]!=10)
					{stop_i=0;stop_lasterr=0;}//ֹͣ����PID��������
					//speed_lastangle=angle;//����PID�м�¼�ϴνǶ�
					//if(function<10 || function>19)//���ԭ�����Ǻ���ģʽ�����������PID����
					//	{speed_i=0;speed_lasterr=0;}
					function=content[1];//����λ
					if(function==10)//�����ǰ����Ϊ10���������ʱ��ȡ��æλ
						{timer=0;io_t=1;}
					else
						{timer=content_target;io_t=0;}
					motor_stop_record=1;//�л�Ϊδ���ת��	
				}				
/*14~16*/else if(content[1]<17)//����14~16�����١��Ƕ�����
				{
					power=(char)content[2];//�ٶ�
					power=power*8/10;
				  angle_target=(long)content_target;//Ŀ��Ƕȣ���ʱ�洢��
					receive_angle_increment();//�����Ƕȼ�¼����	 
					function=content[1];//�ù���λ
					stop_i=0;stop_lasterr=0;//ֹͣ����PID��������
					speed_i=0;speed_lasterr=0;speed_lastangle=angle;//����PID�м�¼�ϴνǶ�
					motor_sangle_record=1;//�л�Ϊδ���ת��	
					io_t=0;//æλ:æ				
				}
/*17~19*/else if(content[1]<20)//����17~19�����١��ǶȾ���ֵ
				{
					power=(char)content[2];//���ʰٷֱ�
					power/=2;
				  angle_target=(long)content_target;//Ŀ��Ƕȣ���ʱ�洢��
					receive_angle_absolute();//Ŀ��������ֵ��¼
					function=content[1]-3;//�ù���λ
					stop_i=0;stop_lasterr=0;//ֹͣ����PID��������	
					speed_i=0;speed_lasterr=0;speed_lastangle=angle;//����PID�м�¼�ϴνǶ�
					motor_sangle_record=1;//�л�Ϊδ���ת��	
					io_t=0;//æλ:æ
				}
/*20~22*/else if(content[1]<23)
				{
					function=content[1];//�ù���λ
					motor_sangle_record=1;//�л�Ϊδ���ת��	
					io_t=0;//æλ:æ
				}
/*23*/  else if(content[1]<24)				
				{
					send_allow=1;//����ش�
					send_num=content[2];//�ش��������
				}	
			}
			content_num=20;//���һ�ν��պ󣬽�������Ŵ��ң��ȴ���һ�ν��ղ���Ч
		}
	}
	if(TI)
	{
		TI=0;
		serial_t_busy=0;
	}
} 
/*************************/
/*���ڷ��ͳ�����*/
/*************************/
void send_long(long send_content_long)
{
	serial_t_busy=1;
	SBUF=0xa5;
			while(serial_t_busy);	serial_t_busy=1;
	SBUF=send_xor[0]=send_content_long>>24;
			while(serial_t_busy);	serial_t_busy=1;
	SBUF=send_xor[1]=send_content_long>>16;
			while(serial_t_busy);	serial_t_busy=1;
	SBUF=send_xor[2]=send_content_long>>8;
			while(serial_t_busy);	serial_t_busy=1;
	SBUF=send_xor[3]=send_content_long;
			while(serial_t_busy);	serial_t_busy=1;
	SBUF=send_xor[0]^send_xor[1]^send_xor[2]^send_xor[3];	//�������У��
			while(serial_t_busy);
}
/*************************/
/*���ڷ��Ͷ�����*/
/*************************/
void send_int(send_content_int)
{
	serial_t_busy=1;
	SBUF=0xa5;
			while(serial_t_busy);	serial_t_busy=1;
	SBUF=send_xor[0]=send_content_int>>8;
			while(serial_t_busy);	serial_t_busy=1;
	SBUF=send_xor[1]=send_content_int;
			while(serial_t_busy);	serial_t_busy=1;
	SBUF=send_xor[0]^send_xor[1];	//�������У��
			while(serial_t_busy);
}
/*************************/
/*T0�ж�*/
/*************************/
void tm0_isr()interrupt 1 using 1
{
	motor_program_run=1;
}
/****************/
/*ȫ����ʼ������*/
/****************/
void Init()
{
	P1M0=0x23; P1M1=0;
	P5M0=0xff; P5M1=0;
	P3M0=0x20; P3M1=0;
	
	EA=1;   //�����ж�
	ES=1;   //�򿪴����ж�
	EX0=1;  //�����ж�0
	IP=0x01;//�������ж�0Ϊ������ȼ�,����Ϊ�����ȼ�
	EX1=1;  //�����ж�1	
	ET0=1;  //��T0�ж�
	EADC=1; //��AD�ж�
	
	IT0=0;  //���ж�0�������ش��������ڼ�������
	IT1=1;  //���ж�1�����½��ش�����
	
  UartInit();//��������
	PWMInit(); //����PWM
	Timer0Init();//��ʱ��0����
	ADInit();//����AD
}
/*********/
/*����AD*/
/*********/
void ADInit()
{
	P1ASF=0x1c;//����AD4 AD3 AD2
}
/**********/
/*���ô���*/
/**********/
void UartInit(void)		//115200bps@33.1776MHz
{
	SCON = 0x50;		//8λ����,�ɱ䲨����
	AUXR |= 0x01;		//����1ѡ��ʱ��2Ϊ�����ʷ�����
	AUXR |= 0x04;		//��ʱ��2ʱ��ΪFosc,��1T
	T2L = 0xB8;		//�趨��ʱ��ֵ
	T2H = 0xFF;		//�趨��ʱ��ֵ
	AUXR |= 0x10;		//������ʱ��2
}

/*********/
/*����PWM*/
/*********/
void PWMInit()
{
	P_SW1=0; //���ڡ�PWM��SPI��Ĭ��IO
	CCON=0; //��ʼ��PCA���ƼĴ���
	CL = 0;
	CH = 0;
	CMOD=0x8c; //����PWMʱ��Ϊ:ϵͳʱ��/6
	CCAPM0=0x42;//PCA1����ΪPWMģʽ
	PCA_PWM0=0;//����Ϊ8λPWM
	CCAPM1=0x42;//PCA2����ΪPWMģʽ
	PCA_PWM1=0;//����Ϊ8λPWM
	CCAPM2=0x42;//PCA2����ΪPWMģʽ		
	PCA_PWM2=0;//����Ϊ8λPWM
	CR=1;			//PCA��ʱ����ʼ
}
/********/
/*����T0*/
/********/
void Timer0Init(void)		//5����@33.1776MHz
{
	AUXR &= 0x7F;		//��ʱ��ʱ��12Tģʽ
	TMOD &= 0xF0;		//���ö�ʱ��ģʽ
	TL0 = 0x00;		//���ö�ʱ��ֵ
	TH0 = 0xCA;		//���ö�ʱ��ֵ
	TF0 = 0;		//���TF0��־
	TR0 = 1;		//��ʱ��0��ʼ��ʱ
}
/********************************/
/*���Ƶ��������������Χ-150~150*/
/********************************/
void motor_move(int motor_move_n)
{
	if(motor_move_n<-150)
	{
		CCAP0H=0;motor_CCW;
	}
	else if(motor_move_n<0)
	{
		CCAP0H=150+motor_move_n;motor_CCW;
	}
	else if(motor_move_n==0)
	{
		CCAP0H=0; motor_brake;
	}
	else if(motor_move_n<150)
	{
		CCAP0H=150-motor_move_n;motor_CW;		
	}
	else
	{
		CCAP0H=0;motor_CW;		
	}
}

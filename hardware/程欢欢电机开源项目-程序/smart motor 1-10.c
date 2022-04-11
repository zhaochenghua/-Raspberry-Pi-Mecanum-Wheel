/*
1：10电机程序，33.1776Mhz

*/
#include "STC15W.h"
//用于更改正反转
#define motor_CW motor_l=1;motor_r=0		//控制电机的正反转 默认1、0
#define motor_CCW motor_l=0;motor_r=1  	//默认0、1
#define motor_CW_coder angle--					//记录角度的正反 默认--
#define motor_CCW_coder angle++					//默认++
#define motor_CWCCW_flag 1 							//设置电机相对旋转时的正反，默认1 可改为-1
//用于更改转速比
#define angleTOreal 9/8 //计算，用360/(减速比*编码器一圈磁极*2)
#define realTOangle 8/9 //与上一条相反
#define angleSpeed 15/4 //速度 (60*1000/50) / (减速比*编码器一圈磁极*2)
//温度保护值
#define temp_safety 814 //对应65度
#define temp_safety_temp 65

#define motor_brake motor_l=1;motor_r=1
#define motor_software_stop motor_l=0;motor_r=0;CCAP0H=0  //电机软刹车
#define motor_nopower motor_l=0;motor_r=0;CCAP0H=0xff
#define led_g CCAP1H
#define led_y CCAP2H

unsigned char code IDcode_h=0;//身份信息，
unsigned char code IDcode_l=1;//身份信息，用于识别是什么硬件，1为高速电机
unsigned char code version_h=1;
unsigned char code version_l=0;//固件版本，由两位组成



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
//保护信息
sbit motor_i = P1^2; //电机电流反馈 AD2
sbit tem = P1^3; //热敏电阻 AD3
sbit power_v = P1^4; //总电源电压 AD4
//指示灯
sbit led_gg = P1^0; //黄灯 PWM1
sbit led_yy = P3^7; //绿灯 PWM2
//电机
sbit motor_l = P5^4; //电机正转 高有效
sbit motor_r = P5^5; //电机反转 低有效
sbit motor_pwm = P1^1; //电机速度 PWM0
sbit motor_a = P3^6; //电机编码器A
sbit motor_b = P3^2; //电机编码器B
//通讯
sbit io_t = P1^5; //发送标记位
sbit io_r = P3^3; //接收标记位 INT1
//速度
int xdata speed;//速度，单位 圈/分钟
unsigned long xdata angle_record[11];  //每5ms记录一次速度，每50ms间隔的数据做差求速度
unsigned char angle_record_n;//记录上一个数组的序号
//电机角度
long angle=0;//电机实时角度 
long angle_real;//记录真是角度（单位度），主要用于连续角度锁停模式下减少误差
long angle_stop=0;//电机停止(锁定)角度，也作目标量
//电机功能变量，用于执行时的参数
unsigned char function=15;//用于执行的功能序，默认15为阻停
char motor_cw_ccw=1;//正反转记录选项(记录目标转动方向，不影响矫正算法)，正转1、反转-1，有接收程序置位
long angle_target;//电机角度目标量，仅作中间值
long timer;//计时
int power;//电机功率百分比-150~150,恒速-50~50
bit motor_stop_record;//在时间模式下，记录首次完成当前旋转，=1为进入，而后归零,自动设置，无需干预
bit motor_sangle_record=1;//注意！角度模式转动前需置1.在角度模式下，记录完成一次动作(动作完成后不执行角度程序)，=0为完成 =1为进行中
//接收命令变量
unsigned char content_function;//接收的功能序号
unsigned char content[8];
unsigned char content_num;//接收指令位序号
unsigned long content_target;//接收到的内容，临时存储
//发送命令变量
long send_content_long;//发送数据(长)
int send_content_int;//发送数据(短)
char send_xor[4];//发送数据异或校验位
//其他状态记录变量
int xdata voltage;//电压，单位
int xdata electricity;//电流，单位待定！
int xdata temp=0;//温度，单位度
unsigned char main_ad_n=2;//ad读取记位变量
unsigned char send_num;//需要发送的数据序号
//程序功能变量
bit motor_program_run;//由定时器通知主程序中的电机程序运行。当=1则运行，而后归零。
bit serial_t_busy;//串口发送忙，1为忙，0为闲
bit send_allow;//发送标志位，1为需要发送，0为不发送
//停止程序PID变量
char stop_kp=8; //6,10
char stop_ki=20;//3,20注：此处使用除法
char stop_kd=40; //30,40
int stop_i;
int stop_d;
int stop_lasterr;
int stop_err;
//恒速程序PID变量
char code speed_kp=1;//5
char code speed_ki=2;//3
char code speed_kd=1;//5 除法
int speed_i;
int speed_d;
long speed_err;
long speed_lasterr;
long speed_lastangle;//上一次角度，用于计算速度


//子函数目录
void delay(unsigned int delay_i);//延迟1毫秒
//void SendString(char *SendString_n);//串口发送程序
void send_long(long send_content_long);
void send_int(send_content_int);
void receive_angle_increment();//接收程序中的角度增量记录程序
void overflow();//判断计算持否溢出，如果溢出则停止程序
void receive_angle_absolute();//接收程序中的角度绝对值记录程序
int temp_real(int temp_real_n);//温度转换程序，返回温度
void speed_get();//速度获取程序，应用在主程序中
void motor_stop(angle_stop);//电机停止锁定程序
void motor_speed(speed_target);//恒速程序
void Init();//初始设置
void UartInit();//串口设置
void PWMInit();//PWM设置
void Timer0Init();//T0设置
void ADInit();//AD设置
void motor_move(int motor_move_n);//控制电机运（范围-150 150）	

void main()
{ unsigned int main_n;
	unsigned char main_led;//绿色LED呼吸灯闪烁所用变量
	bit main_led_n;//同上
	Init();//全部配置初始化
	led_y=0xff;//关闭黄色LED	
	io_t=1;//忙位：闲
	while(1)
	{	
		if(send_allow)//发送数据
		{
			send_allow=0;
			switch(send_num)
			{
				case 1://角度
					{	send_long((long)angle*angleTOreal);	}break;
				case 2://速度
					{	send_int(speed); }break;	
				case 3://电压
					{	send_int(voltage); }break;	
				case 4://电流
					{	send_int(electricity);}break;	
				case 5://温度
					{	send_int(temp_real(temp)); }break;	
				case 6://复位
					{	IAP_CONTR=0xe0; }break;	
				case 7://实时功率百分比（/255）
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
/*程序执行时间间隔变量*/
			main_n++;
			if(main_n==0xffff)main_n=0;
			motor_program_run=0;
/*led呼吸灯程序*/
			if(main_led_n)main_led--;
			else main_led++;
			led_g=main_led;
			if(main_led==255)main_led_n=1;
			else if(main_led==0)main_led_n=0;
	
			switch(function)
			{ 
				case 0://功能0：功率百分比、无限制
				{ 
					motor_move(power);
					io_t=0;//忙信号位
				}break;
				case 1://功能1：功率百分比、时间、锁停
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
						io_t=1;//忙位：闲
					}		
				}break;
				case 2://功能2：功率百分比、时间、阻停
				{	
					if(timer>5)
					{
						timer-=5;
						motor_move(power);
					}
					else
					{ 
						motor_software_stop;//阻停
						io_t=1;//忙位：闲
					}		
				}break;		
				case 3://功能3：功率百分比、时间、无阻
				{	
					if(timer>5)
					{ 
						timer-=5;
						motor_move(power);
					}
					else
					{ 
						motor_nopower;
						io_t=1;//忙位：闲
					}		
				}break;			
				case 4://功能4：功率百分比、角度、锁停
				{	
					if((angle_stop-angle)*motor_cw_ccw > (power/3)*motor_cw_ccw && motor_sangle_record)
					{ 
						motor_move(power);
					}
					else
					{ 
						motor_sangle_record=0;
						motor_stop(angle_stop);
						io_t=1;//忙位：闲
					}		
				}break;	
				case 5://功能5：功率百分比、角度、阻停
				{	
					if(angle_stop*motor_cw_ccw > angle*motor_cw_ccw && motor_sangle_record)
					{
						motor_move(power);
					}
					else
					{
						motor_sangle_record=0;
						motor_software_stop;//阻停
						io_t=1;//忙位：闲
					}		
				}break;				
				case 6://功能6：功率百分比、角度、无阻
				{	
					if(angle_stop*motor_cw_ccw > angle*motor_cw_ccw && motor_sangle_record)
					{
						motor_move(power);
					}
					else
					{
						motor_sangle_record=0;
						motor_nopower;
						io_t=1;//忙位：闲
					}		
				}break;			
				case 10://功能10：恒速、无限制
				{	
					if(power==0 && motor_sangle_record)
					{
						motor_sangle_record=0;//减小最大功率
						angle_stop=angle;
						speed_i=0;speed_lasterr=0;speed_lastangle=angle;//恒速PID中记录上次角度
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
						io_t=1;//忙位：闲
				}break;		
				case 11://功能11：恒速、时间、锁停
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
						io_t=1;//忙位：闲
					}	
				}break;		
				case 12://功能12：恒速、时间、阻停
				{	
					if(timer>5)
					{
						timer-=5;
						if(main_n%5==0)
							motor_speed(power);
					}
					else
					{
						motor_software_stop;//阻停	
						io_t=1;//忙位：闲
					}
				}break;	
				case 13://功能13：恒速、时间、无阻
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
						io_t=1;//忙位：闲
					}
				}break;	
				case 14://功能14：恒速、角度、锁停
				{	
					if((angle_stop-angle)*motor_cw_ccw > (power/2)*motor_cw_ccw && motor_sangle_record)
					{
						if(main_n%5==0)
							motor_speed(power);
					}
					else
					{ motor_sangle_record=0;
						motor_stop(angle_stop);
						io_t=1;//忙位：闲
					}		
				}break;	
				case 15://功能15：恒速、角度、阻停
				{	
					if(angle_stop*motor_cw_ccw > angle*motor_cw_ccw && motor_sangle_record)
					{
						if(main_n%5==0)
							motor_speed(power);
					}
					else
					{ motor_sangle_record=0;
						motor_software_stop;//阻停	
						io_t=1;//忙位：闲
					}		
				}break;					
				case 16://功能16：恒速、角度、无阻
				{	
						if(angle_stop*motor_cw_ccw > angle*motor_cw_ccw && motor_sangle_record)
						{
							if(main_n%5==0)
								motor_speed(power);
						}
						else
						{ motor_sangle_record=0;
							motor_nopower;
							io_t=1;//忙位：闲
						}		
				}break;					
				case 20://功能14：锁停
				{
					if(motor_sangle_record)
						{angle_stop=angle;motor_sangle_record=0;}
					else
						{motor_stop(angle_stop);io_t=1;}//忙位：闲
				}break;					
				case 21://功能15：阻停
				{	
					motor_software_stop;io_t=1;//忙位：闲
				}break;					
				case 22://功能16：无阻
				{	
					motor_nopower;io_t=1;//忙位：闲
				}break;						
			}
//获取速度程序			
			speed_get();
	    if(speed<0)
				speed_get();
/*AD转换程序*/
			ADC_CONTR=0x88+main_ad_n;
			main_ad_n++;
			if(main_ad_n==5)main_ad_n=2;
/*AD温度保护程序*/
			while(temp>temp_safety)//大于65度
			{ 
				while(1)
				{
					led_g=255;motor_nopower;
					led_y=0;delay(200);led_y=255;delay(200);
					/*AD转换程序*/
					ADC_CONTR=0x88+main_ad_n;
					main_ad_n++;
					if(main_ad_n==5)main_ad_n=2;
				}
			}
		}
	}
}
/*************************/
/*温度转换程序*/
/*************************/
int temp_real(int temp_real_n)
{ int temp_real_nn=0;//序号,也是温度
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
/*速度获程序，应用在主程序中*/
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
/*判断计算是否溢出，如果溢出则停止程序，黄灯长亮*/
/*************************/
void overflow()
{
	if(OV==1)//如果溢出
	{EA=0;led_g=255;led_y=0;motor_software_stop;while(1);}			
}
/*************************/
/*接收程序中，角度绝对值记录*/
/*************************/
void receive_angle_absolute()
{
	angle_real=angle_target;
	angle_stop=angle_target*realTOangle;
	if(angle_target>angle)
	{ 				
		motor_cw_ccw=1;//标记正转
		if(power<0)power*=-1;//需要正转
	}
	else
	{				
		motor_cw_ccw=-1;//标记反转
		if(power>0)power*=-1;//需要反转
	}
}
/*************************/
/*接收程序中，角度增量记录*/
/*************************/
void receive_angle_increment()
{
	if(function==4 || function==14)//如果上次也是角度锁停模式，按上次角度目标作为基础量累加
	{
		if(power>0)//如果功率为正转
		{
			if(angle_target>0)//如果角度为正
			{
				motor_cw_ccw=1*motor_CWCCW_flag;//标记正转
				if(content[1]==4 || content[1]==14)//如果当前功能为角度增量锁停
				{ 
					angle_real+=angle_target;
					overflow();
					angle_stop=angle_real*realTOangle; 
				}
			}	
			else//如果角度为负
			{ 
				motor_cw_ccw=-1*motor_CWCCW_flag;//标记反转
				if(content[1]==4 || content[1]==14)//如果当前功能为角度增量锁停
				{ 
					angle_real+=angle_target;
					overflow();
					angle_stop=angle_real*realTOangle; 
				}
				power=power*-1;//功率变为负数
			}
		}		
		else//如果功率为反转
		{
			if(angle_target>0)//如果角度为正
			{
				motor_cw_ccw=-1*motor_CWCCW_flag;//标记反转
				if(content[1]==4 || content[1]==14)//如果当前功能为角度增量锁停
				{ 
					angle_real-=angle_target;
					overflow();
					angle_stop=angle_real*realTOangle; 
				}
			}	
			else//如果角度为负
			{
				motor_cw_ccw=1*motor_CWCCW_flag;//标记正转
				if(content[1]==4 || content[1]==14)//如果当前功能为角度增量锁停
				{ 
					angle_real-=angle_target;
					overflow();
					angle_stop=angle_real*realTOangle; 
				}	
				power=power*-1;//功率变为正
			}	
		}
	}
	else//上次不是角度锁定模式
	{
		if(power>0)//如果功率为正转
		{
			if(angle_target>0)//如果角度为正
			{
				motor_cw_ccw=1*motor_CWCCW_flag;//标记为正转
				angle_stop=angle+(angle_target*realTOangle);//累加角度 angle_real
				overflow();
				if(content[1]==4 || content[1]==14)//如果当前功能为角度增量锁停
					{ angle_real=angle_stop*angleTOreal; }//将角度换算并记录
			}	
			else//如果角度为负
			{ 
				motor_cw_ccw=-1*motor_CWCCW_flag;//标记反转
				angle_stop=angle+(angle_target*realTOangle);//累加负角度
				overflow();
				if(content[1]==4 || content[1]==14)//如果当前功能为角度增量锁停
					{ angle_real=angle_stop*angleTOreal; }//将角度换算并记录
				power*=-1;//功率变为负数
			}
		}		
		else//如果功率为反转
		{
			if(angle_target>0)//如果角度为正
			{
				motor_cw_ccw=-1*motor_CWCCW_flag;//标记反转
				angle_stop=angle+(angle_target*-realTOangle);//减角度
				overflow();
				if(content[1]==4 || content[1]==14)//如果当前功能为角度增量锁停
					{ angle_real=angle_stop*angleTOreal; }//将角度换算并记录
			}	
			else//如果角度为负
			{ 
				motor_cw_ccw=1*motor_CWCCW_flag;//标记正转
				angle_stop=angle+(angle_target*-realTOangle);//减负角度	
				overflow();
				if(content[1]==4 || content[1]==14)//如果当前功能为角度增量锁停
					{ angle_real=angle_stop*angleTOreal; }//将角度换算并记录				
				power*=-1;//功率变为正
			}	
		}
	}
}

/*************************/
/*电机恒速程序*/
/*************************/
void motor_speed(speed_target)
{	
	speed_err=speed_lastangle-angle+speed_target;
	speed_i+=speed_err;
	if(speed_i<-150)speed_i=-150;//限制补回量
	else if(speed_i>150)speed_i=150;//限制补回量
	speed_d=speed_err-speed_lasterr;
	motor_move(speed_err*speed_kp+speed_i*speed_ki+speed_d/speed_kd);
	speed_lasterr=speed_err;
	speed_lastangle=angle;
}
/*************************/
/*电机停止程序*/
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
/*33.1776Mhz 延迟1ms的倍数*/
/*************************/
void delay(unsigned int delay_i)
{
	unsigned int delay_j;
  while(delay_i--)
		for(delay_j=0;delay_j<860;delay_j++);
}
/*************************/
/*AD中断：读取电压、电流、温度*/
/*************************/
void adc_isr() interrupt 5 using 1
{ 
	ADC_CONTR&=0x07;//AD电源和标志位清零
	switch(ADC_CONTR&0x07)
	{
		case 2://电机电流
		{ 
			electricity=ADC_RES*4+ADC_RESL;
		}break;
		case 3://热敏电阻
		{
			temp=ADC_RES*4+ADC_RESL-30;
		}break;
		case 4://电源电压
		{
			voltage=ADC_RES*4+ADC_RESL;
		}break;
	}
}
/*************************/
/*外中断0：读取电机编码器*/
/*************************/
void motor_angle_int0() interrupt 0
{ 
	if(motor_a == motor_b)
		motor_CW_coder;
	else
		motor_CCW_coder;
} 
/**********************************/
/*外中断1:主控器通知电机可进行通讯*/
/**********************************/
void motor_angle_int1() interrupt 2
{ 
	content_num=0;
} 
/*************************/
/*串口中断*/
/*************************/
void uart() interrupt 4 using 1
{ 	
	if(RI)
	{
		RI=0;
	  if(!io_r && content_num<8)//接收选位拉低
		{ 
			content[content_num]=SBUF;
			content_num++;
			if(content[0]!=0xa5)//如果没收到起始位则停止接收数据
				content_num=20;
		}
		if(content_num==8)//接收满
		{
			if(content[7]==0xa5 && content[6]==0xa5 && content[5]==0xa5 && content[4]==0xa5 && content[3]==0xa5 && content[2]==0xa5 && content[1]==0xa5)//全是0xa5的话，回传身份信息
			{
				send_allow=1;//允许回传
				send_num=255;//回传内容序号
			}
			else if(content[7]==content[6]^content[5]^content[4]^content[3]^content[2]^content[1])//校验
			{ 
				content_target=0;//存储内容位
				content_target|=content[3];
				content_target=content_target<<8|content[4];
				content_target=content_target<<8|content[5];
				content_target=content_target<<8|content[6];	
				
/*0~3*/	if(content[1]<4)//功能0~3，功率百分比、时间或无限制
				{
					function=content[1];//功能位
					power=(char)content[2];//功率百分比
					power=power*1.5;
					if(function==0)
						{timer=0;io_t=1;}//忙位：闲
					else if(timer>5)
						{timer=content_target;io_t=1;}//忙位：闲
					stop_i=0;stop_lasterr=0;//停止程序PID参数清零
					if(function==0)//如果当前功能为10，则清零计时，取消忙位
						{timer=0;io_t=1;}
					else
						{timer=content_target;io_t=0;}
					motor_stop_record=1;//切换为未完成转动	
				}
/*4~6*/	else if(content[1]<7)//功能4~6，功率百分比、角度增量
				{ 
					power=(char)content[2];//功率百分比
					power=power*1.5;
				  angle_target=(long)content_target;//目标角度（临时存储）
					receive_angle_increment();//增量角度记录程序	 
					function=content[1];//置功能位
					stop_i=0;stop_lasterr=0;//停止程序PID参数清零	
					motor_sangle_record=1;//切换为未完成转动	
					io_t=0;//忙位:忙
				}		
/*7~9*/	else if(content[1]<10)//功能7~9，功率百分比、角度绝对值
				{
					power=(char)content[2];//功率百分比
					power*=1.5;
				  angle_target=(long)content_target;//目标角度（临时存储）
					receive_angle_absolute();//目标量绝对值记录
					function=content[1]-3;//置功能位
					stop_i=0;stop_lasterr=0;//停止程序PID参数清零	
					motor_sangle_record=1;//切换为未完成转动	
					io_t=0;//忙位:忙
				}
/*10~13*/else if(content[1]<14)//功能0~3，恒速、时间或无限制
				{
					power=(char)content[2];//速度
					power=power*8/10;
					if(content[1]!=10)
					{stop_i=0;stop_lasterr=0;}//停止程序PID参数清零
					//speed_lastangle=angle;//恒速PID中记录上次角度
					//if(function<10 || function>19)//如果原来不是恒速模式，则清零恒速PID参数
					//	{speed_i=0;speed_lasterr=0;}
					function=content[1];//功能位
					if(function==10)//如果当前功能为10，则清零计时，取消忙位
						{timer=0;io_t=1;}
					else
						{timer=content_target;io_t=0;}
					motor_stop_record=1;//切换为未完成转动	
				}				
/*14~16*/else if(content[1]<17)//功能14~16，恒速、角度增量
				{
					power=(char)content[2];//速度
					power=power*8/10;
				  angle_target=(long)content_target;//目标角度（临时存储）
					receive_angle_increment();//增量角度记录程序	 
					function=content[1];//置功能位
					stop_i=0;stop_lasterr=0;//停止程序PID参数清零
					speed_i=0;speed_lasterr=0;speed_lastangle=angle;//恒速PID中记录上次角度
					motor_sangle_record=1;//切换为未完成转动	
					io_t=0;//忙位:忙				
				}
/*17~19*/else if(content[1]<20)//功能17~19，恒速、角度绝对值
				{
					power=(char)content[2];//功率百分比
					power/=2;
				  angle_target=(long)content_target;//目标角度（临时存储）
					receive_angle_absolute();//目标量绝对值记录
					function=content[1]-3;//置功能位
					stop_i=0;stop_lasterr=0;//停止程序PID参数清零	
					speed_i=0;speed_lasterr=0;speed_lastangle=angle;//恒速PID中记录上次角度
					motor_sangle_record=1;//切换为未完成转动	
					io_t=0;//忙位:忙
				}
/*20~22*/else if(content[1]<23)
				{
					function=content[1];//置功能位
					motor_sangle_record=1;//切换为未完成转动	
					io_t=0;//忙位:忙
				}
/*23*/  else if(content[1]<24)				
				{
					send_allow=1;//允许回传
					send_num=content[2];//回传内容序号
				}	
			}
			content_num=20;//完成一次接收后，将接收序号打乱，等待下一次接收才有效
		}
	}
	if(TI)
	{
		TI=0;
		serial_t_busy=0;
	}
} 
/*************************/
/*串口发送长数据*/
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
	SBUF=send_xor[0]^send_xor[1]^send_xor[2]^send_xor[3];	//加总异或校验
			while(serial_t_busy);
}
/*************************/
/*串口发送短数据*/
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
	SBUF=send_xor[0]^send_xor[1];	//加总异或校验
			while(serial_t_busy);
}
/*************************/
/*T0中断*/
/*************************/
void tm0_isr()interrupt 1 using 1
{
	motor_program_run=1;
}
/****************/
/*全部初始化配置*/
/****************/
void Init()
{
	P1M0=0x23; P1M1=0;
	P5M0=0xff; P5M1=0;
	P3M0=0x20; P3M1=0;
	
	EA=1;   //打开总中断
	ES=1;   //打开串口中断
	EX0=1;  //打开外中断0
	IP=0x01;//设置外中断0为最高优先级,其他为低优先级
	EX1=1;  //打开外中断1	
	ET0=1;  //打开T0中断
	EADC=1; //打开AD中断
	
	IT0=0;  //外中断0：上下沿触发，用于检测编码器
	IT1=1;  //外中断1：仅下降沿触发，
	
  UartInit();//串口设置
	PWMInit(); //设置PWM
	Timer0Init();//定时器0设置
	ADInit();//设置AD
}
/*********/
/*设置AD*/
/*********/
void ADInit()
{
	P1ASF=0x1c;//允许AD4 AD3 AD2
}
/**********/
/*设置串口*/
/**********/
void UartInit(void)		//115200bps@33.1776MHz
{
	SCON = 0x50;		//8位数据,可变波特率
	AUXR |= 0x01;		//串口1选择定时器2为波特率发生器
	AUXR |= 0x04;		//定时器2时钟为Fosc,即1T
	T2L = 0xB8;		//设定定时初值
	T2H = 0xFF;		//设定定时初值
	AUXR |= 0x10;		//启动定时器2
}

/*********/
/*设置PWM*/
/*********/
void PWMInit()
{
	P_SW1=0; //串口、PWM、SPI在默认IO
	CCON=0; //初始化PCA控制寄存器
	CL = 0;
	CH = 0;
	CMOD=0x8c; //设置PWM时钟为:系统时钟/6
	CCAPM0=0x42;//PCA1设置为PWM模式
	PCA_PWM0=0;//设置为8位PWM
	CCAPM1=0x42;//PCA2设置为PWM模式
	PCA_PWM1=0;//设置为8位PWM
	CCAPM2=0x42;//PCA2设置为PWM模式		
	PCA_PWM2=0;//设置为8位PWM
	CR=1;			//PCA定时器开始
}
/********/
/*设置T0*/
/********/
void Timer0Init(void)		//5毫秒@33.1776MHz
{
	AUXR &= 0x7F;		//定时器时钟12T模式
	TMOD &= 0xF0;		//设置定时器模式
	TL0 = 0x00;		//设置定时初值
	TH0 = 0xCA;		//设置定时初值
	TF0 = 0;		//清除TF0标志
	TR0 = 1;		//定时器0开始计时
}
/********************************/
/*控制电机函数。参数范围-150~150*/
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

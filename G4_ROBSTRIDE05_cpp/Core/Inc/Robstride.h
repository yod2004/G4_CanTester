#ifndef __RobStride_H__
#define __RobStride_H__

#ifdef __cplusplus
 extern "C" {
#endif

#ifdef __cplusplus
 }
#endif


#include "main.h"
//#include "can.h"
 
 #define Set_mode 		 'j'				//���ÿ���ģʽ
 #define Set_parameter 'p'				//���ò���
 //���ֿ���ģʽ
 #define move_control_mode  0	//�˿�ģʽ
 #define Pos_control_mode   1	//PPλ��ģʽ
 #define Speed_control_mode 2 //�ٶ�ģʽ
 #define Elect_control_mode 3 //����ģʽ
 #define Set_Zero_mode      4 //���ģʽ
 #define CSP_control_mode		5	//CSPλ��ģʽ
 //ͨ�ŵ�ַ
#define Communication_Type_Get_ID 0x00     							//��ȡ�豸��ID��64λMCUΨһ��ʶ��`
#define Communication_Type_MotionControl 0x01 					//�˿�ģʽ�������������Ϳ���ָ��
#define Communication_Type_MotorRequest 0x02						//���������������������״̬
#define Communication_Type_MotorEnable 0x03							//���ʹ������
#define Communication_Type_MotorStop 0x04								//���ֹͣ����
#define Communication_Type_SetPosZero 0x06							//���õ����е��λ
#define Communication_Type_Can_ID 0x07									//���ĵ�ǰ���CAN_ID
#define Communication_Type_Control_Mode 0x12						//���õ��ģʽ
#define Communication_Type_GetSingleParameter 0x11			//��ȡ��������
#define Communication_Type_SetSingleParameter 0x12			//�趨��������
#define Communication_Type_ErrorFeedback 0x15						//���Ϸ���֡
//ʹ������ģʽʱ��ע���������汾���Ƿ� >= 0.13.0		
#define Communication_Type_MotorDataSave 0x16						//������ݱ���֡
#define Communication_Type_BaudRateChange 0x17					//����������޸�֡�������ϵ���Ч
#define Communication_Type_ProactiveEscalationSet 0x18	//��������ϱ�
#define Communication_Type_MotorModeSet 0x19						//���Э���޸�֡�������ϵ���Ч


class data_read_write_one
{
	public:
		uint16_t index;
		float data;
};
static const uint16_t Index_List[] = {0X7005, 0X7006, 0X700A, 0X700B, 0X7010, 0X7011, 0X7014, 0X7016, 0X7017, 0X7018, 0x7019, 0x701A, 0x701B, 0x701C, 0x701D};
//18ͨ�����Ϳ���д��Ĳ����б�
//����������			������ַ			����			����			�ֽ���			��λ/˵��
class data_read_write			//�ɶ�д�Ĳ���
{
	
	public:
		data_read_write_one run_mode;				//0:�˿�ģʽ 1:λ��ģʽ 2:�ٶ�ģʽ 3:����ģʽ 4:���ģʽ uint8  1byte
		data_read_write_one iq_ref;					//����ģʽIqָ��   				float 	4byte 	-23~23A
		data_read_write_one spd_ref;				//ת��ģʽת��ָ�� 				float 	4byte 	-30~30rad/s 
		data_read_write_one imit_torque;		//ת������ 								float 	4byte 	0~12Nm  
		data_read_write_one cur_kp;					//������ Kp 							float 	4byte 	Ĭ��ֵ 0.125  
		data_read_write_one cur_ki;					//������ Ki 							float 	4byte 	Ĭ��ֵ 0.0158  
		data_read_write_one cur_filt_gain;	//�����˲�ϵ��filt_gain 	float 	4byte 	0~1.0��Ĭ��ֵ0.1  
		data_read_write_one loc_ref;				//λ��ģʽ�Ƕ�ָ��				float 	4byte 	rad  
		data_read_write_one limit_spd;			//λ��ģʽ�ٶ�����				float 	4byte 	0~30rad/s  
		data_read_write_one limit_cur;			//�ٶ�λ��ģʽ�������� 		float 	4byte 	0~23A
		//����ֻ�ɶ�
		data_read_write_one mechPos;				//���ض˼�Ȧ��е�Ƕ�			float 	4byte 	rad
		data_read_write_one iqf;						//iq �˲�ֵ 							float 	4byte 	-23~23A
		data_read_write_one	mechVel;				//���ض�ת��							float 	4byte 	-30~30rad/s 	
		data_read_write_one	VBUS;						//ĸ�ߵ�ѹ								float 	4byte 	V	
		data_read_write_one	rotation;				//Ȧ�� 										int16 	2byte   Ȧ��
		data_read_write(const uint16_t *index_list=Index_List);
};
typedef struct
{
	float Angle;
	float Speed;
	float Torque;
	float Temp;
	int pattern; //���ģʽ��0��λ1�궨2���У�
}Motor_Pos_RobStride_Info;
typedef struct
{
	int set_motor_mode;
	float set_current;
	float set_speed;
	float set_acceleration;
	float set_Torque;
	float set_angle;
	float set_limit_cur;
	float set_limit_speed;
	float set_Kp;
	float set_Ki;
	float set_Kd;
}Motor_Set;

enum MIT_TYPE
{
    operationControl = 0,
    positionControl = 1,
    speedControl = 2
};

//RobStride_Motor���
class RobStride_Motor
{
private:		
	uint8_t CAN_ID;						//CAN ID   (Ĭ��127(0x7f) ����ͨ����λ����ͨ������1�鿴)
	uint64_t Unique_ID;					//64λMCUΨһ��ʶ��
	uint16_t Master_CAN_ID;		//����ID  �����ڳ�ʼ���������趨Ϊ0x1F��
	float (*Motor_Offset_MotoFunc)(float Motor_Tar);

	Motor_Set Motor_Set_All;		//�趨ֵ
	uint8_t error_code;

	bool MIT_Mode;			//MITģʽ
	MIT_TYPE MIT_Type;		//MITģʽ����

	void Set_MIT_Mode(bool MIT_Mode);
	void Set_MIT_Type(MIT_TYPE MIT_Type);

public:
	float output;
	int Can_Motor;
	Motor_Pos_RobStride_Info Pos_Info;		//�ش�ֵ
	data_read_write drw;      						//�������
	RobStride_Motor(uint8_t CAN_Id, bool MIT_Mode);
	RobStride_Motor(float (*Offset_MotoFunc)(float Motor_Tar) , uint8_t CAN_Id, bool MIT_mode);
	void RobStride_Get_CAN_ID();
	void Set_RobStride_Motor_parameter(uint16_t Index, float Value, char Value_mode);
	void Get_RobStride_Motor_parameter(uint16_t Index);
	void RobStride_Motor_Analysis(uint8_t *DataFrame,uint32_t ID_ExtId);
	void RobStride_Motor_move_control(float Torque, float Angle, float Speed, float Kp, float Kd);
	void RobStride_Motor_Pos_control( float Speed, float Angle);
	void RobStride_Motor_CSP_control(float Angle, float limit_spd);
	void RobStride_Motor_Speed_control(float Speed, float limit_cur);
	void RobStride_Motor_current_control( float current);
	void RobStride_Motor_Set_Zero_control();
	void RobStride_Motor_MotorModeSet(uint8_t F_CMD);
	void Enable_Motor();
	void Disenable_Motor( uint8_t clear_error);
	void Set_CAN_ID(uint8_t Set_CAN_ID);
	void Set_ZeroPos();
	

	bool Get_MIT_Mode();
	MIT_TYPE get_MIT_Type();
	void RobStride_Motor_MIT_Control(float Angle, float Speed, float Kp, float Kd, float Torque);
	void RobStride_Motor_MIT_PositionControl(float position_rad, float speed_rad_per_s);
	void RobStride_Motor_MIT_SpeedControl(float speed_rad_per_s, float current_limit);
	void RobStride_Motor_MIT_Enable();
	void RobStride_Motor_MIT_Disable();
	void RobStride_Motor_MIT_SetZeroPos();
	void RobStride_Motor_MIT_ClearOrCheckError(uint8_t F_CMD);
	void RobStride_Motor_MIT_SetMotorType(uint8_t F_CMD);
	void RobStride_Motor_MIT_SetMotorId(uint8_t F_CMD);
	void RobStride_Motor_MotorDataSave();
	void RobStride_Motor_BaudRateChange(uint8_t F_CMD);
	void RobStride_Motor_ProactiveEscalationSet(uint8_t F_CMD);
	void RobStride_Motor_MIT_MotorModeSet(uint8_t F_CMD);
};






#endif

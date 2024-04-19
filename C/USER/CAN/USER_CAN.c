#include "USER_CAN.h"

#define Yaw_Position 2000


//�ж�CAN1���յ�����Ϣ�����ڵ��̵��
#define can1_chassis_motor(id)  ( id == 0x201 || \
                                  id == 0x202 || \
                                  id == 0x203 || \
                                  id == 0x204 )

//�ж�CAN1���յ���������CAN1��yaw����������
#define can1_gimbal_motor(id) 	( id == 0x205 || \
                                  id == 0x206 || \
                                  id == 0x207 || \
                                  id == 0x208 )

//�ж�CAN2���յ�����Ϣ�����ڵ��̵��
#define can2_chassis_motor(id)  ( id == 0x201 || \
                                  id == 0x202 || \
                                  id == 0x203 || \
                                  id == 0x204 )

//�ж�CAN2���յ���������CAN2��yaw����������
#define can2_gimbal_motor(id) 	( id == 0x205 || \
                                  id == 0x206 || \
                                  id == 0x207 || \
                                  id == 0x208 )

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static uint8_t CAN1_TX_200[8]={0};//����CAN1 ��������1
static uint8_t CAN1_TX_1FF[8]={0};//����CAN1 ��������2

static uint8_t CAN2_TX_200[8]={0};//����CAN2 ��������1
static uint8_t CAN2_TX_1FF[8]={0};//����CAN2 ��������2

MotorBack CAN1_Motor[CAN1_Motor_NUM];	//CAN1�ϵ����е��������Ϣ

MotorBack CAN2_Motor[CAN2_Motor_NUM];	//CAN2�ϵ����е��������Ϣ

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
  * @brief          ��ʼ��CAN���߹�����
  * @retval         none
  */
void CAN_Filter_Init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;//����ģʽ
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;//32λ��ȡ�ֽ�
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;	//������0
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;//ѡ������FIFO 0
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;//����ɸѡ��
    can_filter_st.FilterBank = 14;					//������14
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

//=================================================================================//
/**
  * @brief          ��CAN1���������������
	* @param[in]			id: ���͵���ֵ��ID  0x200����1~4�ŵ����0x1FF����5~8�ŵ��
  * @retval         none
  */
//=================================================================================//
void CAN1_to_Motor(uint16_t id)
{
	uint32_t TxMailbox; //��������
	CAN_TxHeaderTypeDef     tx_message;//���巢�ͽṹ��

	tx_message.StdId = id;//  ��ʶ��ID
	tx_message.IDE   = CAN_ID_STD;
	tx_message.RTR   = CAN_RTR_DATA;
	tx_message.DLC   = 0x08;

	switch(id)
	{
		case 0x200:
		{
			HAL_CAN_AddTxMessage(&hcan1,&tx_message,CAN1_TX_200,&TxMailbox);//����
			break;
		}
		case 0x222:
		{
			HAL_CAN_AddTxMessage(&hcan1,&tx_message,CAN1_TX_200,&TxMailbox);//����
			break;
		}
		case 0x1FF:
		{
			HAL_CAN_AddTxMessage(&hcan1,&tx_message,CAN1_TX_1FF,&TxMailbox);//����
			break;
		}
		default: break;
	}
}

//=================================================================================//
/**
  * @brief          ��CAN2���������������
	* @param[in]			id: ���͵���ֵ��ID  0x200����1~4�ŵ����0x1FF����5~8�ŵ��
  * @retval         none
  */
//=================================================================================//
void CAN2_to_Motor(uint16_t id)
{
	uint32_t TxMailbox; //��������
	CAN_TxHeaderTypeDef     tx_message;//���巢�ͽṹ��

	tx_message.StdId = id;//  ��ʶ��ID
	tx_message.IDE   = CAN_ID_STD;
	tx_message.RTR   = CAN_RTR_DATA;
	tx_message.DLC   = 0x08;

	switch(id)
	{
		case 0x200:
		{
			HAL_CAN_AddTxMessage(&hcan2,&tx_message,CAN2_TX_200,&TxMailbox);//����
			break;
		}
		case 0x1FF:
		{
			HAL_CAN_AddTxMessage(&hcan2,&tx_message,CAN2_TX_1FF,&TxMailbox);//����
			break;
		}
		default: break;
	}
}

//=================================================================================//
/**
  * @brief          �޸�CAN1�ϵ�ĳ������ĵ���ֵ
	* @param[in]			id: ��Ҫ�޸ĵ���ֵ�ĵ��ID  ��Χ1~8
  * @retval         none
  */
//=================================================================================//
void CAN1_OPerate_Motor(uint8_t id,short i)
{
	switch(id)
	{
		case 1:
		{
			CAN1_TX_200[0] = (uint8_t)(i>>8);
			CAN1_TX_200[1] = (uint8_t)(i&0x00ff);
			break;
		}
		case 2:
		{
			CAN1_TX_200[2] = (uint8_t)(i>>8);
			CAN1_TX_200[3] = (uint8_t)(i&0x00ff);
			break;
		}
		case 3:
		{
			CAN1_TX_200[4] = (uint8_t)(i>>8);
			CAN1_TX_200[5] = (uint8_t)(i&0x00ff);
			break;
		}
		case 4:
		{
			CAN1_TX_200[6] = (uint8_t)(i>>8);
			CAN1_TX_200[7] = (uint8_t)(i&0x00ff);
			break;
		}
		case 5:
		{
			CAN1_TX_1FF[0] = (uint8_t)(i>>8);
			CAN1_TX_1FF[1] = (uint8_t)(i&0x00ff);
			break;
		}
		case 6:
		{
			CAN1_TX_1FF[2] = (uint8_t)(i>>8);
			CAN1_TX_1FF[3] = (uint8_t)(i&0x00ff);
			break;
		}
		case 7:
		{
			CAN1_TX_1FF[4] = (uint8_t)(i>>8);
			CAN1_TX_1FF[5] = (uint8_t)(i&0x00ff);
			break;
		}
		case 8:
		{
			CAN1_TX_1FF[6] = (uint8_t)(i>>8);
			CAN1_TX_1FF[7] = (uint8_t)(i&0x00ff);
			break;
		}
		default: break;
	}
	
}

//=================================================================================//
/**
  * @brief          �޸�CAN2�ϵ�ĳ������ĵ���ֵ
	* @param[in]			id: ��Ҫ�޸ĵ���ֵ�ĵ��ID  ��Χ1~8
  * @retval         none
  */
//=================================================================================//
void CAN2_OPerate_Motor(uint8_t id,short i)
{
	switch(id)
	{
		case 1:
		{
			CAN2_TX_200[0] = (uint8_t)(i>>8);
			CAN2_TX_200[1] = (uint8_t)(i&0x00ff);
			break;
		}
		case 2:
		{
			CAN2_TX_200[2] = (uint8_t)(i>>8);
			CAN2_TX_200[3] = (uint8_t)(i&0x00ff);
			break;
		}
		case 3:
		{
			CAN2_TX_200[4] = (uint8_t)(i>>8);
			CAN2_TX_200[5] = (uint8_t)(i&0x00ff);
			break;
		}
		case 4:
		{
			CAN2_TX_200[6] = (uint8_t)(i>>8);
			CAN2_TX_200[7] = (uint8_t)(i&0x00ff);
			break;
		}
		case 5:
		{
			CAN2_TX_1FF[0] = (uint8_t)(i>>8);
			CAN2_TX_1FF[1] = (uint8_t)(i&0x00ff);
			break;
		}
		case 6:
		{
			CAN2_TX_1FF[2] = (uint8_t)(i>>8);
			CAN2_TX_1FF[3] = (uint8_t)(i&0x00ff);
			break;
		}
		case 7:
		{
			CAN2_TX_1FF[4] = (uint8_t)(i>>8);
			CAN2_TX_1FF[5] = (uint8_t)(i&0x00ff);
			break;
		}
		case 8:
		{
			CAN2_TX_1FF[6] = (uint8_t)(i>>8);
			CAN2_TX_1FF[7] = (uint8_t)(i&0x00ff);
			break;
		}
		default: break;
	}
}

//=================================================================================//
/**
  * @brief          CAN�����жϻص�����
  * @param[in]      hcan: �����жϵ�CAN
  * @retval         none
  */
//=================================================================================//
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//���ջص�����
{
	CAN_RxHeaderTypeDef		rx_message;//������սṹ��
	uint8_t rx_data[8];
	uint8_t id = 0;
	if(hcan ==&hcan1)//�ص�CAN1 RX
	{	    
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_message, rx_data);//��ȡ������ɱ�־    
		//���������can1��������
		id = (uint8_t)(rx_message.StdId - 0x201);
		if(can1_chassis_motor(rx_message.StdId))	//���յ��̵������
		{
			CAN1_Motor[id].Last_Speed = CAN1_Motor[id].Speed;
			CAN1_Motor[id].Speed = (short)(rx_data[2]<<8 | rx_data[3]);
			CAN1_Motor[id].Last_Angle = CAN1_Motor[id].Angle;
			CAN1_Motor[id].Angle = (short)(rx_data[0]<<8 | rx_data[1]);
			CAN1_Motor[id].Given_Current = (short)(rx_data[4]<<8 | rx_data[5]);
			CAN1_Motor[id].Temperate = rx_data[6];         			
		}
		else if(can1_gimbal_motor(rx_message.StdId))	//������̨�������
		{
             
			CAN1_Motor[id].Last_Speed = CAN1_Motor[id].Speed;
			CAN1_Motor[id].Speed = (short)(rx_data[2]<<8 | rx_data[3]);
			CAN1_Motor[id].Last_Angle = CAN1_Motor[id].Angle;
			CAN1_Motor[id].Angle = (short)(rx_data[0]<<8 | rx_data[1]);
			CAN1_Motor[id].Given_Current = (short)(rx_data[4]<<8 | rx_data[5]);
			CAN1_Motor[id].Temperate = rx_data[6];
            
                                        /*diversity_Angle ��С������*/
			if(Yaw_Position-CAN1_Motor[id].Angle>-4100&&Yaw_Position-CAN1_Motor[id].Angle<2300)//-2500��6100-8192=-2092 
			{
				CAN1_Motor[id].diversity_Angle=Yaw_Position-CAN1_Motor[id].Angle;
			}
			else
			{
				CAN1_Motor[id].diversity_Angle=Yaw_Position+8192-CAN1_Motor[id].Angle;
			} 
								 /*Angle_Radian ���Ƚ�*/
			CAN1_Motor[id].Angle_Radian=Yaw_Position-CAN1_Motor[id].Angle>=0?
			Yaw_Position-CAN1_Motor[id].Angle:8192+Yaw_Position-CAN1_Motor[id].Angle;//3436+8192   //8912
									/*CircleȦ�����㡤*/
			if((CAN1_Motor[id].Angle-CAN1_Motor[id].Last_Angle)>4100)
				CAN1_Motor[id].Circle--;
			else if((CAN1_Motor[id].Angle-CAN1_Motor[id].Last_Angle)<-4100)
				CAN1_Motor[id].Circle++;

			//���Ե������ǶȵĲ�ֵ����4100��ʱ�򣬿϶���ǰһ�νǶ�Ϊ8000���ң���һ�νǶ�Ϊ200���ң���ֵ�ʹ���4100�������ʱ��Ҳ�պ�ת��һȦ��
							/*�����Ǹı�Continuous_Angle And Angle_Difference��ʼ�ǶȺ�Ԥ���ǶȲ�ֵ*/
			CAN1_Motor[id].Continuous_Angle=(int)CAN1_Motor[id].Circle*8192+
																			CAN1_Motor[id].Angle-CAN1_Motor[id].Last_Continuous_Angle;    //�����ǣ������ܹ����˶��ٵĽǶȣ�ȫ���Ƕ�֮�ͣ���Ȧ���͵�ǰ�ǶȻ��г�ʼ�Ƕ��йء�
					
			if(CAN1_Motor[id].Flag==0)    //��������ִֻ��һ�Σ������жϵ���ĳ�ʼֵ
			{
				CAN1_Motor[id].Flag=1;
				CAN1_Motor[id].Last_Continuous_Angle=CAN1_Motor[id].Continuous_Angle;     //�����"Last_Continuous_Angle"�൱��һ����ֵ�������ʼ��ʱ��ĳ�ʼ�Ƕ�
				CAN1_Motor[id].Angle_Difference=(Yaw_Position-CAN1_Motor[id].Angle);//
			}            
		}
		else
		{
//      Receive_Data(&rx_message,rx_data);
			CAN1_Motor[id].Last_Angle = CAN1_Motor[id].Angle;
			CAN1_Motor[id].Last_Speed = CAN1_Motor[id].Speed;
			CAN1_Motor[id].Angle = (short)(rx_data[0]<<8 | rx_data[1]);
			CAN1_Motor[id].Speed = (short)(rx_data[2]<<8 | rx_data[3]);
			CAN1_Motor[id].Given_Current = (short)(rx_data[4]<<8 | rx_data[5]);
			CAN1_Motor[id].Temperate = rx_data[6];
		}
		//��Ҫ��д�жϱ�־λ����
		__HAL_CAN_CLEAR_FLAG(&hcan1,CAN_IER_FMPIE0);							
	}
	else//�ص�CAN2 RX
	{	    
		if ( HAL_OK == HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_message, rx_data))//��ȡ������ɱ�־
		{
			//���������can2��������
			id = (uint8_t)(rx_message.StdId - 0x201);
			CAN2_Motor[id].Last_Speed = CAN2_Motor[id].Speed;
			CAN2_Motor[id].Speed = (short)(rx_data[2]<<8 | rx_data[3]);
			CAN2_Motor[id].Last_Angle = CAN2_Motor[id].Angle;
			CAN2_Motor[id].Angle = (short)(rx_data[0]<<8 | rx_data[1]);
			CAN2_Motor[id].Given_Current = (short)(rx_data[4]<<8 | rx_data[5]);
			CAN2_Motor[id].Temperate = rx_data[6];			
			//��Ҫ��д�жϱ�־λ����
			__HAL_CAN_CLEAR_FLAG(&hcan2,CAN_IER_FMPIE0);
		}	 								
	}
}

